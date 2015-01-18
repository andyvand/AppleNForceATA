/*
 * Copyright (c) 2004 Apple Computer, Inc. All rights reserved.
 *
 * @APPLE_LICENSE_HEADER_START@
 * 
 * The contents of this file constitute Original Code as defined in and
 * are subject to the Apple Public Source License Version 1.1 (the
 * "License").  You may not use this file except in compliance with the
 * License.  Please obtain a copy of the License at
 * http://www.apple.com/publicsource and read it before using this file.
 * 
 * This Original Code and all software distributed under the License are
 * distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE OR NON-INFRINGEMENT.  Please see the
 * License for the specific language governing rights and limitations
 * under the License.
 * 
 * @APPLE_LICENSE_HEADER_END@
 */

#include <sys/systm.h>    // snprintf
#include <IOKit/assert.h>
#include <IOKit/IOMessage.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/storage/IOStorageProtocolCharacteristics.h>
#include "AppleNForceATA.h"

#ifdef FEDE_DEBUG
#define FEDE_LOG(fmt,args...) IOLog(fmt, ## args); kprintf(fmt, ## args);
#define FEDE_RELOG(fmt,args...) IOLog(fmt, ## args); kprintf(fmt, ## args);
#else
#define FEDE_LOG(fmt,args...) 
#define FEDE_RELOG(fmt,args...)
#endif
// for wait U8Status, loop time in uS
#define kStatusDelayTime  5
// how many times through the loop for a MS.
#define kStatusDelayLoopMS  1000 / kStatusDelayTime

#define super IOPCIATA
OSDefineMetaClassAndStructors( AppleNForceATA, IOPCIATA )

#define kPIOModeMask   ((1 << kPIOModeCount) - 1)
#define kDMAModeMask   ((1 << kDMAModeCount) - 1)
#define kUDMAModeMask  (fProvider->getUltraDMAModeMask())
#define kATA_SSTATUS   (0x00 + (fChannelNumber << 6))
#define kATA_SERROR    (0x04 + (fChannelNumber << 6))
#define kATA_SCONTROL  (0x08 + (fChannelNumber << 6))

#define DRIVE_IS_PRESENT(u) \
        (_devInfo[u].type != kUnknownATADeviceType)

#define TIMING_PARAM_IS_VALID(p) \
        ((p) != 0)

// Increase the PRD table size to one full page or 4096 descriptors for
// large transfers via DMA.  2048 are required for 1 megabyte transfers
// assuming no fragmentation and no alignment issues on the buffer.  We
// allocate twice that since there are more issues than simple alignment
// for this DMA engine.

#define kMaxPRDCount           512
#define kMaxPRDSegmentSize     0x10000

#define kATAXferDMADesc  512
#define kATAMaxDMADesc   kATAXferDMADesc * 2

// up to 2048 ATA sectors per transfer
#define kMaxATAXfer      512 * 2048

/*
// Increase the PRD table size to two full page or 8192 descriptors for
// large transfers via DMA.  2048 are required for 1 megabyte transfers
// assuming no fragmentation and no alignment issues on the buffer.  We
// allocate twice that since there are more issues than simple alignment
// for this DMA engine.
#define kMaxPRDCount           1024
#define kMaxPRDSegmentSize     0x10000

#define kATAXferDMADesc  1024
#define kATAMaxDMADesc   kATAXferDMADesc

// up to 2048 ATA sectors per transfer
#define kMaxATAXfer      512 * 4096
*/
/*---------------------------------------------------------------------------
 *
 * Start the single-channel NVIDIA ATA controller driver.
 *
 ---------------------------------------------------------------------------*/
 
bool AppleNForceATA::start( IOService * provider )
{
    bool superStarted = false;
	UInt8 bitmask, cable/*, mask*/;
	UInt16 udma;

    DEBUG_LOG("%s: %s( %p, %p )\n", getName(), __FUNCTION__, this, provider);

    // Our provider is a 'nub' that represents a single channel PCI ATA
    // controller, and not an IOPCIDevice.

    fProvider = OSDynamicCast( AppleNForceATAChannel, provider );
    if ( fProvider == 0 )
        goto fail;

    // Retain and open our provider.

    fProvider->retain();

    if ( fProvider->open( this ) != true )
    {
        DEBUG_LOG("%s: provider open failed\n", getName());
        goto fail;
    }

    // Create a work loop.

    fWorkLoop = IOWorkLoop::workLoop();
    if ( fWorkLoop == 0 )
    {
        DEBUG_LOG("%s: new work loop failed\n", getName());
        goto fail;
    }

    // Cache static controller properties.

    fChannelNumber = fProvider->getChannelNumber();
    if ( fChannelNumber > SEC_CHANNEL_ID )
    {
        DEBUG_LOG("%s: bad ATA channel number %ld\n", getName(),
                  fChannelNumber);
        goto fail;
    }
	
	// Check 80-pin cable
	bitmask = fChannelNumber == SEC_CHANNEL_ID ? 0xC0 : 0x03;
	/*mask = fChannelNumber == SEC_CHANNEL_ID ? 0x02 : 0x01;
	
	if ((fProvider->pciConfigRead8(PCI_IDE_ENABLE) & mask) != mask)
	{
        DEBUG_LOG("%s: ATA IDE not enabled on channel number %ld\n", getName(),
                  fChannelNumber);
        goto fail;
	}*/
		
	cable = fProvider->pciConfigRead8(PCI_CABLE_DETECT);
	f80PinCablePresent = (cable & bitmask);
	
	/* We now have to double check because the Nvidia boxes BIOS
	   doesn't always set the cable bits but does set mode bits */

	udma = fProvider->pciConfigRead16(0x60 + (2 * (fChannelNumber == SEC_CHANNEL_ID ? 1 : 0)));
	if ((udma & 0xC4) == 0xC4 || (udma & 0xC400) == 0xC400)
		f80PinCablePresent = true;	

    /*f80PinCable[0] = ((readTimingRegister(kTimingRegUltra, 0) & 0x10) != 0);
    f80PinCable[1] = ((readTimingRegister(kTimingRegUltra, 1) & 0x10) != 0);*/


	// Get the base address for the bus master registers in I/O space.
    if ( getBMBaseAddress( fChannelNumber, &fBMBaseAddr ) != true )
    {
        DEBUG_LOG("%s: invalid bus-master base address\n", getName());
        goto fail;
    }

    // Must setup these variables inherited from IOPCIATA before it is started.

    _bmCommandReg   = IOATAIOReg8::withAddress( fBMBaseAddr + BM_COMMAND );
    _bmStatusReg    = IOATAIOReg8::withAddress( fBMBaseAddr + BM_STATUS );
    _bmPRDAddresReg = IOATAIOReg32::withAddress( fBMBaseAddr + BM_PRD_TABLE );
	
    // Reset bus timings for both drives.
    initializeHardware();
	/*if (fProvider->getHardwareType() >= PCI_HW_SATA && _mmapaddr == NULL)
		goto fail;*/
    resetBusTimings();

    // Override P-ATA reporting in IOATAController::start()
    // for SystemProfiler.

    if (fProvider->getHardwareType() == PCI_HW_SATA)
    {
        setProperty( kIOPropertyPhysicalInterconnectTypeKey,
                     kIOPropertyPhysicalInterconnectTypeSerialATA );
    }


    // Now we are ready to call super::start

    if ( super::start(_provider) == false )
    {
        goto fail;
    }
    superStarted = true;

    // This driver will handle interrupts using a work loop.
    // Create interrupt event source that will signal the
    // work loop (thread) when a device interrupt occurs.

	if ( fProvider->getInterruptVector() == 14 ||
         fProvider->getInterruptVector() == 15 )
    {
        // Legacy IRQ are never shared, no need for an interrupt filter.
        fInterruptSource = IOInterruptEventSource::interruptEventSource(
                           this, &interruptOccurred,
                           fProvider, 0 );
    }
    else
    {
		fInterruptSource = IOFilterInterruptEventSource::filterInterruptEventSource(
							this, &interruptOccurred, &interruptFilter,
							fProvider, 0 );
	}

    if ( !fInterruptSource ||
         (fWorkLoop->addEventSource(fInterruptSource) != kIOReturnSuccess) )
    {
        DEBUG_LOG("%s: interrupt registration error\n", getName());
        goto fail;
    }
	
    fInterruptSource->enable();

    // Attach to power management.

    initForPM( provider );

    // For each device discovered on the ATA bus (by super),
    // create a nub for that device and call registerService() to
    // trigger matching against that device.

    for ( UInt32 i = 0; i < kMaxDriveCount; i++ )
    {
        if ( _devInfo[i].type != kUnknownATADeviceType )
        {
            ATADeviceNub * nub;

            nub = ATADeviceNub::ataDeviceNub( (IOATAController*) this,
                                              (ataUnitID) i,
                                              _devInfo[i].type );

            if ( nub )
            {
                if ( _devInfo[i].type == kATAPIDeviceType )
                {

                    nub->setProperty( kIOMaximumSegmentCountReadKey,
                                      kMaxPRDCount / 2, 64 );

                    nub->setProperty( kIOMaximumSegmentCountWriteKey,
                                      kMaxPRDCount / 2, 64 );

                    nub->setProperty( kIOMaximumSegmentByteCountReadKey,
                                      kMaxPRDSegmentSize, 64 );

                    nub->setProperty( kIOMaximumSegmentByteCountWriteKey,
                                      kMaxPRDSegmentSize, 64 );
                }

                if ( nub->attach( this ) )
                {
                    _nub[i] = (IOATADevice *) nub;
                    _nub[i]->retain();
                    _nub[i]->registerService();
                }
                nub->release();
            }
        }
    }

    // Successful start, announce useful properties.

	IOLog("%s: NVIDIA %s (CMD 0x%x, CTR 0x%x, IRQ %u, BM 0x%x)\n", getName(),
          fProvider->getHardwareName(),
          fProvider->getCommandBlockAddress(),
          fProvider->getControlBlockAddress(),
          (unsigned int)fProvider->getInterruptVector(),
          fBMBaseAddr);

    return true;

fail:
    if ( fProvider )
        fProvider->close( this );

    if ( superStarted )
        super::stop( provider );

    return false;
}

/*---------------------------------------------------------------------------
 *
 * Stop the single-channel NVIDIA ATA controller driver.
 *
 ---------------------------------------------------------------------------*/

void AppleNForceATA::stop( IOService * provider )
{		
    PMstop();
    super::stop( provider );
}

/*---------------------------------------------------------------------------
 *
 * Release resources before this driver is destroyed.
 *
 ---------------------------------------------------------------------------*/

void AppleNForceATA::free( void )
{
#define RELEASE(x) do { if(x) { (x)->release(); (x) = 0; } } while(0)

    DEBUG_LOG("%s::%s( %p )\n", getName(), __FUNCTION__, this);

    // Release resources created by start().

    if (fInterruptSource && fWorkLoop)
    {
        fWorkLoop->removeEventSource(fInterruptSource);
    }

    RELEASE( fProvider        );
    RELEASE( fInterruptSource );
    RELEASE( fWorkLoop        );
    RELEASE( _nub[0]          );
    RELEASE( _nub[1]          );
    RELEASE( _bmCommandReg    );
    RELEASE( _bmStatusReg     );
    RELEASE( _bmPRDAddresReg  );
	//RELEASE( fInterruptStatus );

    // Release registers created by configureTFPointers().

    RELEASE( _tfDataReg       );
    RELEASE( _tfFeatureReg    );
    RELEASE( _tfSCountReg     );
    RELEASE( _tfSectorNReg    );
    RELEASE( _tfCylLoReg      );
    RELEASE( _tfCylHiReg      );
    RELEASE( _tfSDHReg        );
    RELEASE( _tfStatusCmdReg  );
    RELEASE( _tfAltSDevCReg   );

    // IOATAController should release this.

    if ( _doubleBuffer.logicalBuffer )
    {
        IOFree( (void *) _doubleBuffer.logicalBuffer,
                         _doubleBuffer.bufferSize );
        _doubleBuffer.bufferSize     = 0;
        _doubleBuffer.logicalBuffer  = 0;
        _doubleBuffer.physicalBuffer = 0;
    }

    // What about _cmdGate, and _timer in the superclass?

    super::free();
}

/*---------------------------------------------------------------------------
 *
 * Return the driver's work loop
 *
 ---------------------------------------------------------------------------*/

IOWorkLoop * AppleNForceATA::getWorkLoop( void ) const
{
    return fWorkLoop;
}

/*---------------------------------------------------------------------------
 *
 * Override IOATAController::synchronousIO()
 *
 ---------------------------------------------------------------------------*/

IOReturn AppleNForceATA::synchronousIO( void )
{
    // IOATAController::synchronousIO() asserts nIEN bit in order to disable
    // drive interrupts during polled mode command execution. The problem is
    // that this will float the INTRQ line and put it in high impedance state,
    // which on certain systems has the undesirable effect of latching a false
    // interrupt on the interrupt controller. Perhaps those systems lack a
    // strong pull down resistor on the INTRQ line. Experiment shows that the
    // interrupt event source is signalled, and its producerCount incremented
    // after every synchronousIO() call. This false interrupt can become
    // catastrophic after reverting to async operations since software can
    // issue a command, handle the false interrupt, and issue another command
    // to the drive before the actual completion of the first command, leading
    // to a irrecoverable bus hang. This function is called after an ATA bus
    // reset. Waking from system sleep will exercise this path.
    // The workaround is to mask the interrupt line while the INTRQ line is
    // floating (or bouncing).
	
	IOReturn err = kATANoErr;

    if (fInterruptSource) fInterruptSource->disable();
	err = super::synchronousIO();
    if (fInterruptSource) fInterruptSource->enable();

    return err;
}

/*---------------------------------------------------------------------------
 *
 * Determine the start of the I/O mapped Bus-Master registers.
 *
 ---------------------------------------------------------------------------*/

bool AppleNForceATA::getBMBaseAddress( UInt32   channel,
                                          UInt16 * baseAddr )
{
    UInt32 bmiba;

    DEBUG_LOG("%s::%s( %p, %ld, %p )\n", getName(), __FUNCTION__,
              this, channel, baseAddr);

    bmiba = fProvider->pciConfigRead32( PCI_BMIBA );

    if ((bmiba & PCI_BMIBA_RTE) == 0)
    {
        DEBUG_LOG("%s: PCI BAR 0x%02x (0x%08lx) is not an I/O range\n",
                  getName(), PCI_BMIBA, bmiba);
        return false;
    }

    bmiba &= PCI_BMIBA_MASK;  // get the address portion
    if (bmiba == 0)
    {
        DEBUG_LOG("%s: BMIBA is zero\n", getName());
        return false;
    }

    if (channel == SEC_CHANNEL_ID)
        bmiba += BM_SEC_OFFSET;

    *baseAddr = (UInt16) bmiba;
    DEBUG_LOG("%s: BMBaseAddr = %04x\n", getName(), *baseAddr);

    return true;
}

/*---------------------------------------------------------------------------
 *
 * Reset all timing registers to the slowest (most compatible) timing.
 * DMA modes are disabled.
 *
 ---------------------------------------------------------------------------*/

void AppleNForceATA::resetBusTimings( void )
{
    DEBUG_LOG("%s::%s( %p )\n", getName(), __FUNCTION__, this);

    memset(&fBusTimings[0], 0, sizeof(fBusTimings));

    fBusTimings[0].pioTiming = &PIOTimingTable[0];
    fBusTimings[1].pioTiming = &PIOTimingTable[0];

    programTimingRegisters();
}

/*---------------------------------------------------------------------------
 *
 * Setup the location of the task file registers.
 *
 ---------------------------------------------------------------------------*/

bool AppleNForceATA::configureTFPointers( void )
{
    DEBUG_LOG("%s::%s( %p )\n", getName(), __FUNCTION__, this);

    UInt16 cmdBlockAddr = fProvider->getCommandBlockAddress();
    UInt16 ctrBlockAddr = fProvider->getControlBlockAddress();

    _tfDataReg      = IOATAIOReg16::withAddress( cmdBlockAddr + 0 );
    _tfFeatureReg   = IOATAIOReg8::withAddress(  cmdBlockAddr + 1 );
    _tfSCountReg    = IOATAIOReg8::withAddress(  cmdBlockAddr + 2 );
    _tfSectorNReg   = IOATAIOReg8::withAddress(  cmdBlockAddr + 3 );
    _tfCylLoReg     = IOATAIOReg8::withAddress(  cmdBlockAddr + 4 );
    _tfCylHiReg     = IOATAIOReg8::withAddress(  cmdBlockAddr + 5 );
    _tfSDHReg       = IOATAIOReg8::withAddress(  cmdBlockAddr + 6 );
    _tfStatusCmdReg = IOATAIOReg8::withAddress(  cmdBlockAddr + 7 );
    _tfAltSDevCReg  = IOATAIOReg8::withAddress(  ctrBlockAddr + 2 );
	
    if ( !_tfDataReg || !_tfFeatureReg || !_tfSCountReg ||
         !_tfSectorNReg || !_tfCylLoReg || !_tfCylHiReg ||
         !_tfSDHReg || !_tfStatusCmdReg || !_tfAltSDevCReg )
    {
        return false;
    }

    return true;
}

/*---------------------------------------------------------------------------
 *
 * The work loop based interrupt handler called by our interrupt event
 * source.
 *
 ---------------------------------------------------------------------------*/

IOReturn AppleNForceATA::handleDeviceInterrupt( void )
{
	IOReturn result = super::handleDeviceInterrupt();

	// clear the edge-trigger bit
	*_bmStatusReg = BM_STATUS_INT;
	OSSynchronizeIO();
	
	return result;
}

void AppleNForceATA::interruptOccurred( OSObject *               owner,
                                           IOInterruptEventSource * source,
                                           int                      count )
{
    AppleNForceATA * self = (AppleNForceATA *) owner;
	
    // Let our superclass handle the interrupt to advance to the next state
    // in the state machine.

    self->handleDeviceInterrupt();
}

/*---------------------------------------------------------------------------
 *
 * Filter interrupts that are not originated by our hardware. This will help
 * prevent waking up our work loop thread when sharing a interrupt line with
 * another driver.
 *
 ---------------------------------------------------------------------------*/

bool AppleNForceATA::interruptFilter( OSObject * owner,
                                         IOFilterInterruptEventSource * src )
{
    AppleNForceATA * self = (AppleNForceATA *) owner;
	
	return self->interruptIsValid( src );
}

bool AppleNForceATA::interruptIsValid( IOFilterInterruptEventSource * src )
{
	bool ret = false;
	OSSynchronizeIO();
	if ( (*_bmStatusReg) & BM_STATUS_INT ) 
		ret = true;
	
	return ret;
}

bool AppleNForceATA::ata_sata_connect( void )
{
	volatile uint32_t status;
	int timeout;
	
	for (timeout = 0; timeout < 100; timeout++)
	{
        /*
         * status 0x00000113 or 0x00000123 Is SATA Port Connected
         * status 0x00000000 Is SATA Port Not Connected
         */
		status = OSReadLittleInt32( _mmapaddr, kATA_SSTATUS );
		if ((status & 0x00000fff) == 0x00000113 ||
			(status & 0x00000fff) == 0x00000123)
			break;
		IODelay(1000);
	}
	if (timeout >= 100) {
		DEBUG_LOG("%s: warning: phy connection failed. status=0x%08x\n", getName(), status);
		return false;
	}
	
	// clear sata error register
	OSWriteLittleInt32( _mmapaddr, kATA_SERROR, OSReadLittleInt32( _mmapaddr, kATA_SERROR ));
	return true;
}

bool AppleNForceATA::ata_sata_phy_reset( void )
{
	int loop, retry;
	
	if ((OSReadLittleInt32(_mmapaddr, kATA_SCONTROL) & 0x00000000f) == 0x00000000)
		return ata_sata_connect();
		
	for (retry = 0; retry < 10; retry++)
	{
		for (loop = 0; loop < 10; loop++)
		{
			OSWriteLittleInt32( _mmapaddr, kATA_SCONTROL, 0x00000001 );
			IODelay(10);
			if ((OSReadLittleInt32( _mmapaddr, kATA_SCONTROL ) & 0x0000000f) == 0x00000001)
				break;
		}
		IODelay(500);
		for (loop = 0; loop < 10; loop++)
		{
			OSWriteLittleInt32( _mmapaddr, kATA_SCONTROL, 0x00000300 );
			IODelay(10);
			if ((OSReadLittleInt32( _mmapaddr, kATA_SCONTROL ) & 0x0000000f) == 0x00000000)
				return ata_sata_connect();
		}
	}
	
	ERROR_LOG("%s: warning: sata phy reset failed!\n", getName());
	return false;
}

/*---------------------------------------------------------------------------
 *
 * Extend the implementation of scanForDrives() from IOATAController
 * to issue a soft reset before scanning for ATA/ATAPI drive signatures.
 *
 ---------------------------------------------------------------------------*/

bool AppleNForceATA::ATAPISlaveExists( void )
{
	/*if (fProvider->getHardwareType() == PCI_HW_SATA)
		return false;*/
	
	return super::ATAPISlaveExists();
}

UInt32 AppleNForceATA::scanForDrives( void )
{
/*******************************************************************
 * IDE & SATA All devices (HD & DVD)
 *******************************************************************/

    //DEBUG MSG
    if (_mmapaddr != NULL)
        if (ata_sata_phy_reset())
            DEBUG_LOG("%s: sata phy reset done.\n", getName());

    *_tfAltSDevCReg = mATADCRReset; //ATA Reset
    IODelay( 100 );
    *_tfAltSDevCReg = 0x0;
    IOSleep( 10 );

    *_tfSDHReg = 0x00; //Initialize device selection to device 0.
    
    return super::scanForDrives();

/*******************************************************************
 * SATA devices Only (HD & DVD)
 *******************************************************************/
/*
    //Reset SATA Port Only
    if (_mmapaddr == NULL) return 0;

    *_tfAltSDevCReg = mATADCRReset; //ATA Reset
    IODelay( 100 );
    *_tfAltSDevCReg = 0x0;
    IOSleep( 10 );

    *_tfSDHReg = 0x00; //Initialize device selection to device 0.

    return super::scanForDrives();
*/
/*******************************************************************
 * SATA Devices HD Only (No CD/DVD)
 *******************************************************************/
/*
    //Reset SATA Port Only
    if (_mmapaddr == NULL) return 0;

    UInt32 unitsFound=0;
    UInt32 sataStat = OSReadLittleInt32( _mmapaddr, kATA_SSTATUS );
    UInt32 devicesFound = 0;

    switch( (sataStat & 0x00000007) )
    {
        case 3:
            devicesFound = 1;
        break;
    }
    if( devicesFound )
    {
        *_tfAltSDevCReg = mATADCRReset; //ATA Reset
        IODelay( 100);
        *_tfAltSDevCReg = 0x0;
        IOSleep( 10);
        for ( int unit = 0; ((unit < 2) && (unitsFound < 2)); unit++ )
        {
            if ( _devInfo[unit].type == kUnknownATADeviceType )
            {
                UInt32 milsSpent;
                for ( milsSpent = 0; milsSpent < 100; )//10000
                {
                    *_tfSDHReg = ( unit << 4 );
                    IODelay( 10 );
                    if ( (*_tfStatusCmdReg & mATABusy) == 0x00 ) break;
                    IOSleep( 10 );
                    milsSpent += 10;
                }
                if ( milsSpent >= 100 ) break;//10000
                if ( (*_tfCylLoReg == 0x00) && (*_tfCylHiReg == 0x00) &&
                    (*_tfSCountReg == 0x01) && (*_tfSectorNReg == 0x01) &&
                    ( (*_tfAltSDevCReg & 0x50) == 0x50) )
                {
                    _devInfo[unit].type = kATADeviceType;
                    _devInfo[unit].packetSend = kATAPIUnknown;
                    unitsFound++;
                }
            }
        }
        *_tfSDHReg = 0x00; //Initialize device selection to device 0.
        return unitsFound;
    }
    OSWriteLittleInt32( _mmapaddr, kATA_SCONTROL, 0x3 );
    return 0;
*/
}

/*---------------------------------------------------------------------------
 *
 * Provide information on the ATA bus capability.
 *
 ---------------------------------------------------------------------------*/

IOReturn AppleNForceATA::provideBusInfo( IOATABusInfo * infoOut )
{
    DEBUG_LOG("%s::%s( %p, %p )\n", getName(), __FUNCTION__, this, infoOut);

    if ( infoOut == 0 )
    {
        DEBUG_LOG("%s: %s bad argument\n", getName(), __FUNCTION__);
        return -1;
    }

    infoOut->zeroData();

    if (fProvider->getHardwareType() == PCI_HW_SATA)
        infoOut->setSocketType( kInternalSATA );
    else
        infoOut->setSocketType( kInternalATASocket );

    for ( UInt32 i = 0; i < kMaxDriveCount; i++ )
    {
        if ( _devInfo[i].type == kATADeviceType )
        {
            infoOut->setPIOModes( kPIOModeMask );
            infoOut->setDMAModes( kDMAModeMask );
            infoOut->setUltraModes( kUDMAModeMask );
        }
        else
        {
            infoOut->setPIOModes( kPIOModeMask );
        }
    }
    infoOut->setExtendedLBA( true );
    infoOut->setMaxBlocksExtended( 0x0800 );  // 2048 sectors for ext LBA
	//infoOut->setMaxBlocksExtended( 0x1000 ); // 4096 sectors for ext LBA

    UInt8 units = 0;
    if ( _devInfo[0].type != kUnknownATADeviceType ) units++;
    if ( _devInfo[1].type != kUnknownATADeviceType ) units++;
    infoOut->setUnits( units );

    return kATANoErr;
}

/*---------------------------------------------------------------------------
 *
 * Returns the currently configured timings for the drive unit.
 *
 ---------------------------------------------------------------------------*/

IOReturn AppleNForceATA::getConfig( IOATADevConfig * configOut,
                                       UInt32           unit )
{
    DEBUG_LOG("%s::%s( %p, %p, %ld )\n", getName(), __FUNCTION__,
              this, configOut, unit);

    if ((configOut == 0) || (unit > kATADevice1DeviceID))
    {
        DEBUG_LOG("%s: %s bad argument\n", getName(), __FUNCTION__);
        return -1;
    }

    configOut->setPIOMode( 0 );
    configOut->setDMAMode( 0 );
    configOut->setUltraMode( 0 );

    // Note that we need to report the bitmap of each mode,
    // not its mode number.

    if (TIMING_PARAM_IS_VALID(fBusTimings[unit].pioTiming))
    {
        configOut->setPIOMode( 1 << fBusTimings[unit].pioModeNumber );
        configOut->setPIOCycleTime( fBusTimings[unit].pioTiming->cycle );
    }

    if (TIMING_PARAM_IS_VALID(fBusTimings[unit].dmaTiming))
    {
        configOut->setDMAMode( 1 << fBusTimings[unit].dmaModeNumber );
        configOut->setDMACycleTime( fBusTimings[unit].dmaTiming->cycle );
    }

    if (fBusTimings[unit].ultraEnabled)
    {
        configOut->setUltraMode( 1 << fBusTimings[unit].ultraModeNumber );
    }

    configOut->setPacketConfig( _devInfo[unit].packetSend );

    return kATANoErr;
}

/*---------------------------------------------------------------------------
 *
 * Select the bus timings for a given drive unit.
 *
 ---------------------------------------------------------------------------*/

IOReturn AppleNForceATA::selectConfig( IOATADevConfig * configRequest,
                                          UInt32           unit )
{
    DEBUG_LOG("%s::%s( %p, %p, %ld )\n", getName(), __FUNCTION__,
              this, configRequest, unit);

    if ((configRequest == 0) || (unit > kATADevice1DeviceID))
    {
        DEBUG_LOG("%s: %s bad argument\n", getName(), __FUNCTION__);
        return -1;
    }

    // All config requests must include a supported PIO mode

    if ((configRequest->getPIOMode() & kPIOModeMask) == 0)
    {
        DEBUG_LOG("%s: PIO mode unsupported\n", getName());
        return kATAModeNotSupported;
    }

    if (configRequest->getDMAMode() & ~kDMAModeMask)
    {
        DEBUG_LOG("%s: DMA mode unsupported (0x%x)\n",
                  getName(), configRequest->getDMAMode());
        return kATAModeNotSupported;
    }

    if (configRequest->getUltraMode() & ~kUDMAModeMask)
    {
        DEBUG_LOG("%s: UDMA mode unsupported (0x%x)\n",
                  getName(), configRequest->getUltraMode());
        return kATAModeNotSupported;
    }

    if (configRequest->getDMAMode() && configRequest->getUltraMode())
    {
        DEBUG_LOG("%s: multiple DMA mode selection error\n", getName());
        return kATAModeNotSupported;
    }

    _devInfo[unit].packetSend = configRequest->getPacketConfig();

    selectTimingParameter( configRequest, unit );

    return getConfig( configRequest, unit );
}

/*---------------------------------------------------------------------------
 *
 * Select timing parameters based on config request.
 *
 ---------------------------------------------------------------------------*/

void AppleNForceATA::selectTimingParameter( IOATADevConfig * configRequest,
                                               UInt32           unit )
{
    DEBUG_LOG("%s::%s( %p, %d )\n", getName(), __FUNCTION__, this, (int)unit);

    // Reset existing parameters for this unit.

    fBusTimings[unit].pioTiming = &PIOTimingTable[0];
    fBusTimings[unit].dmaTiming = 0;
    fBusTimings[unit].ultraEnabled = false;

    if ( configRequest->getPIOMode() )
    {
        UInt32  pioModeNumber;
        UInt32  pioCycleTime;
        UInt32  pioTimingEntry = 0;

        pioModeNumber = bitSigToNumeric( configRequest->getPIOMode() );
        pioModeNumber = min(pioModeNumber, kPIOModeCount - 1);

        // Use a default cycle time if the device didn't report a time to use.
    
        pioCycleTime = configRequest->getPIOCycleTime();
        pioCycleTime = max(pioCycleTime, PIOMinCycleTime[pioModeNumber]);

        // Look for the fastest entry in the PIOTimingTable with a cycle time
        // which is larger than or equal to pioCycleTime.
    
        for (int i = kPIOTimingCount - 1; i > 0; i--)
        {
            if ( PIOTimingTable[i].cycle >= pioCycleTime )
            {
                pioTimingEntry = i;
                break;
            }
        }

        fBusTimings[unit].pioTiming = &PIOTimingTable[pioTimingEntry];
        fBusTimings[unit].pioModeNumber = pioModeNumber;
        DEBUG_LOG("%s: selected PIO mode %d\n", getName(), (int)pioModeNumber);
        setDriveProperty(unit, kSelectedPIOModeKey, pioModeNumber, 8);
    }

    if ( configRequest->getDMAMode() )
    {
        UInt32  dmaModeNumber;
        UInt32  dmaCycleTime;
        UInt32  dmaTimingEntry = 0;

        dmaModeNumber = bitSigToNumeric( configRequest->getDMAMode() );
        dmaModeNumber = min(dmaModeNumber, kDMAModeCount - 1);

        dmaCycleTime = configRequest->getDMACycleTime();
        dmaCycleTime = max(dmaCycleTime, DMAMinCycleTime[dmaModeNumber]);

        // Look for the fastest entry in the DMATimingTable with a cycle time
        // which is larger than or equal to dmaCycleTime.
    
        for (int i = kDMATimingCount - 1; i > 0; i--)
        {
            if ( DMATimingTable[i].cycle >= dmaCycleTime )
            {
                dmaTimingEntry = i;
                break;
            }
        }
        
        fBusTimings[unit].dmaTiming = &DMATimingTable[dmaTimingEntry];
        fBusTimings[unit].dmaModeNumber = dmaModeNumber;
        DEBUG_LOG("%s: selected DMA mode %d\n", getName(), (int)dmaModeNumber);
        setDriveProperty(unit, kSelectedDMAModeKey, dmaModeNumber, 8);
    }

    if ( configRequest->getUltraMode() )
    {
        UInt32  ultraModeNumber;

        ultraModeNumber = bitSigToNumeric( configRequest->getUltraMode() );
        ultraModeNumber = min(ultraModeNumber, kUDMAModeCount - 1);

        // For Ultra DMA mode 3 or higher, 80 pin cable must be present.
        // Otherwise, the drive will be limited to UDMA mode 2.

        if ( fProvider->getHardwareType() != PCI_HW_SATA && 
             ultraModeNumber > 2 )
        {
            if ( f80PinCablePresent == false )
            {
                DEBUG_LOG("%s: 80-conductor cable not detected\n", getName());
                ultraModeNumber = 2;
            }
        }

        fBusTimings[unit].ultraEnabled = true;
        fBusTimings[unit].ultraModeNumber = ultraModeNumber;
        DEBUG_LOG("%s: selected Ultra mode %d\n", getName(), (int)ultraModeNumber);
        setDriveProperty(unit, kSelectedUltraDMAModeKey, ultraModeNumber, 8);
    }

    programTimingRegisters();
}

/*---------------------------------------------------------------------------
 *
 * Program timing registers for both drives.
 *
 ---------------------------------------------------------------------------*/

static void mergeTimings( TimingParameter *       dst,
                          const TimingParameter * src )
{
    if (TIMING_PARAM_IS_VALID(dst) == false ||
        TIMING_PARAM_IS_VALID(src) == false)
        return;

    dst->cycle    = max(dst->cycle, src->cycle);
    dst->setup    = max(dst->setup, src->setup);
    dst->active   = max(dst->active, src->active);
    dst->recovery = max(dst->recovery, src->recovery);
}

void AppleNForceATA::programTimingRegisters( void )
{
    if (fProvider->getHardwareType() != PCI_HW_SATA)
    {
        TimingParameter  timingCommand;  // shared between both drives
        TimingParameter  timingData[2];

        memset(&timingCommand, 0, sizeof(timingCommand));
        memset(&timingData[0], 0, sizeof(timingData));

        for (int unit = 0; unit < 2; unit++)
        {
            if (DRIVE_IS_PRESENT(unit) == false)
                continue;

            mergeTimings( &timingCommand,    fBusTimings[unit].pioTiming );
            mergeTimings( &timingData[unit], fBusTimings[unit].pioTiming );
            mergeTimings( &timingData[unit], fBusTimings[unit].dmaTiming );
        }

        // We now have all the information need to program the registers.

        for (int unit = 0; unit < 2; unit++)
        {
            if (DRIVE_IS_PRESENT(unit) == false)
                continue;

            writeTimingIntervalNS( kTimingRegCommandActive,
                                unit, timingCommand.active );

            writeTimingIntervalNS( kTimingRegCommandRecovery,
                                unit, timingCommand.recovery );

            writeTimingIntervalNS( kTimingRegAddressSetup,
                                unit, timingData[unit].setup );

            writeTimingIntervalNS( kTimingRegDataActive,
                                unit, timingData[unit].active );

            writeTimingIntervalNS( kTimingRegDataRecovery,
                                unit, timingData[unit].recovery );

            if (fBusTimings[unit].ultraEnabled)
            {
                UInt8 mode = fBusTimings[unit].ultraModeNumber;
                writeTimingRegister( kTimingRegUltra, unit,
                                    UltraTimingTable[mode]); 
            }
            else
            {
                writeTimingRegister( kTimingRegUltra, unit, 0x8b ); 
            }        
        }
    }

    dumpHardwareRegisters();
}

/*---------------------------------------------------------------------------
 *
 * Read and write timing registers.
 *
 ---------------------------------------------------------------------------*/

void AppleNForceATA::writeTimingIntervalNS( TimingReg reg,
                                               UInt32       unit,
                                               UInt32       timeNS )
{
    const UInt32 clockPeriodPS = 30000; // 30ns @ 33MHz PCI
    UInt32 periods = ((timeNS * 1000) + clockPeriodPS - 1) / clockPeriodPS;
	UInt32 shifts = TimingRegInfo[reg].shift;
	
	if (reg == kTimingRegAddressSetup)
        shifts -= ((unit << 1) + (fChannelNumber << 2));

    periods =  min(periods, TimingRegInfo[reg].maxValue);
    periods =  max(periods, TimingRegInfo[reg].minValue);
    periods -= TimingRegInfo[reg].minValue;
    periods &= TimingRegInfo[reg].mask;
	periods <<= shifts;

    fProvider->pciConfigWrite8(TimingRegOffset[reg][fChannelNumber][unit],
                                 periods, TimingRegInfo[reg].mask << shifts);

    DEBUG_LOG("%s: CH%d DRV%d wrote 0x%02x to offset 0x%02x\n",
              getName(), (int)fChannelNumber, (int)unit, (unsigned int)periods,
              TimingRegOffset[reg][fChannelNumber][unit]);
}

void AppleNForceATA::writeTimingRegister( TimingReg reg,
                                             UInt32       unit,
                                             UInt8        periods )
{
    fProvider->pciConfigWrite8( TimingRegOffset[reg][fChannelNumber][unit],
                                periods, 0xFF );

    DEBUG_LOG("%s: CH%d DRV%d wrote 0x%02x to offset 0x%02x\n",
              getName(), (int)fChannelNumber, (int)unit, (unsigned int)periods,
              TimingRegOffset[reg][fChannelNumber][unit]);
}

UInt32 AppleNForceATA::readTimingIntervalNS( TimingReg reg, UInt32 unit )
{
    UInt32 time;
    UInt32 shifts = TimingRegInfo[reg].shift;

    if (reg == kTimingRegAddressSetup)
        shifts -= ((unit << 1) + (fChannelNumber << 2));

    time =   readTimingRegister( reg, unit );
    time >>= shifts;
	time &=  TimingRegInfo[reg].mask;
    time +=  TimingRegInfo[reg].minValue;
	time *=  30;

    return time;
}

UInt8 AppleNForceATA::readTimingRegister( TimingReg reg, UInt32 unit )
{
    return fProvider->pciConfigRead8(
                      TimingRegOffset[reg][fChannelNumber][unit]);
}

/*---------------------------------------------------------------------------
 *
 * Hardware initialization.
 *
 ---------------------------------------------------------------------------*/

void AppleNForceATA::initializeHardware( void )
{	
	IOPCIDevice* _pcidevice;
	
	// AppleNForceATAChannel->AppleNForceATARoot->IOPCIDevice
	
	_pcidevice = OSDynamicCast( IOPCIDevice, fProvider->getProvider()->getProvider() );
	if (_pcidevice == NULL)
	{
		DEBUG_LOG("%s: no pci access!\n", getName());
		return;
	}
	
	if (fProvider->getHardwareType() >= PCI_HW_SATA)
	{
		int offset = fProvider->getHardwareFlags() & NV4 ? 0x0440 : 0x0010;

		_mmap = _pcidevice->mapDeviceMemoryWithRegister( kIOPCIConfigBaseAddress5 );
		if (_mmap == NULL) return;
		_mmapaddr = (void*)_mmap->getVirtualAddress();
		if (_mmapaddr == 0) return;

		DEBUG_LOG("%s::%s mapped memory at 0x%x\n", getName(), __FUNCTION__, _mmapaddr);
		
		// enable control access
		_pcidevice->configWrite8( 0x50, _pcidevice->configRead8( 0x50 ) | 0x04 );

		if (fProvider->getHardwareFlags() & NVQ)
		{
			// clear interrupt status
			OSWriteLittleInt32( _mmapaddr, offset, 0x00ff00ff );
			// enable device and phy state change
			OSWriteLittleInt32( _mmapaddr, offset + 4, 0x000d000d );
			// disable ncq support
			OSWriteLittleInt32( _mmapaddr, 0x0400, OSReadLittleInt32( _mmapaddr, 0x0400 ) & 0xfffffff9 );
		}
		else
		{
			// clear interrupt status
			OSWriteLittleInt( _mmapaddr, offset, 0xff );
			// enable device and phy state change
			OSWriteLittleInt( _mmapaddr, offset + 1, 0xdd );
		}
		
		// clear error register
		//OSWriteLittleInt32( _mmapaddr, kATA_SERROR, OSReadLittleInt32( _mmapaddr, kATA_SERROR ) );
		// disable phy slumber and partial modes
		//OSWriteLittleInt32( _mmapaddr, kATA_SCONTROL, 0x00000300 );
		OSSynchronizeIO();
		
		// enable pci interrupt
		_pcidevice->configWrite16( 0x04, _pcidevice->configRead16( 0x04 ) & ~0x0400 );
	}
	else
	{
		_pcidevice->configWrite8( 0x51, _pcidevice->configRead8( 0x51 ) & 0x0f );
	}
	
	//*_bmStatusReg = 0x04;
}

/*---------------------------------------------------------------------------
 *
 * Dynamically select the bus timings for a drive unit.
 *
 ---------------------------------------------------------------------------*/

void AppleNForceATA::selectIOTiming( ataUnitID unit )
{
    /* Timings was already applied by selectConfig() */
}

/*---------------------------------------------------------------------------
 *
 * Flush the outstanding commands in the command queue.
 * Implementation borrowed from MacIOATA in IOATAFamily.
 *
 ---------------------------------------------------------------------------*/

IOReturn AppleNForceATA::handleQueueFlush( void )
{
    UInt32 savedQstate = _queueState;

    DEBUG_LOG("%s::%s()\n", getName(), __FUNCTION__);

    _queueState = IOATAController::kQueueLocked;

    IOATABusCommand * cmdPtr = 0;

    while ( (cmdPtr = dequeueFirstCommand()) )
    {
        cmdPtr->setResult( kIOReturnError );
        cmdPtr->executeCallback();
    }

    _queueState = savedQstate;

    return kATANoErr;
}

/*---------------------------------------------------------------------------
 *
 * Handle termination notification from the provider.
 *
 ---------------------------------------------------------------------------*/

IOReturn AppleNForceATA::message( UInt32      type,
                                      IOService * provider,
                                      void *      argument )
{
    if ( ( provider == fProvider ) &&
         ( type == kIOMessageServiceIsTerminated ) )
    {
        fProvider->close( this );
        return kIOReturnSuccess;
    }

    return super::message( type, provider, argument );
}

/*---------------------------------------------------------------------------
 *
 * Publish a numeric property pertaining to a drive to the registry.
 *
 ---------------------------------------------------------------------------*/

bool AppleNForceATA::setDriveProperty( UInt32       driveUnit,
                                          const char * key,
                                          UInt32       value,
                                          UInt32       numberOfBits)
{
    char keyString[40];
    
    snprintf(keyString, 40, "Drive %u %s", (unsigned int)driveUnit, key);
    
    return super::setProperty( keyString, value, numberOfBits );
}

//---------------------------------------------------------------------------

IOReturn AppleNForceATA::createChannelCommands( void )
{
	FEDE_RELOG("FEDE - %s: %s( %p)\n", getName(), __FUNCTION__, this);

	FEDE_RELOG("FEDE Casteo a IOAABUSCommand64\n");
	IOATABusCommand64* currentCommand64 = OSDynamicCast( IOATABusCommand64, _currentCommand );
	FEDE_RELOG("FEDE Obtengo el comaqndo DMA\n ");
	IODMACommand* currentDMACmd = currentCommand64->GetDMACommand();
    IODMACommand::Segment32 elSegmento;
	IOReturn DMAStatus = 0;
	
	if ( NULL == currentDMACmd
		|| currentDMACmd->getMemoryDescriptor() == NULL)
    {
        FEDE_RELOG("FEDE DMA BUFFER NOT SET ON COMMAND\n ");
		IOLog("%s: DMA buffer not set on command\n", getName());
		return -1;
    }


	FEDE_RELOG("FEDE obtengo el descriptor de memoria\n ");
    IOMemoryDescriptor* descriptor = _currentCommand->getBuffer();
    IOMemoryCursor::PhysicalSegment physSegment;
    UInt32 index = 0;
    UInt8  *xferDataPtr, *ptr2EndData, *next64KBlock, *starting64KBlock;
    UInt32 xferCount, count2Next64KBlock;
    
    if ( !descriptor )
    {
        return -1;
    }

    // This form of DMA engine can only do 1 pass.
    // It cannot execute multiple chains.
	UInt32 numSegmentos = 1;

    IOByteCount bytesRemaining = _currentCommand->getByteCount() ;
    IOByteCount xfrPosition    = _currentCommand->getPosition() ;
    UInt64  transferSize  = 0; 

    // There's a unique problem with pci-style controllers, in that each
    // dma transaction is not allowed to cross a 64K boundary. This leaves
    // us with the yucky task of picking apart any descriptor segments that
    // cross such a boundary ourselves.  

//   while ( _DMACursor->getPhysicalSegments(
//                          /* descriptor */ descriptor,
//                          /* position   */ xfrPosition,
//                          /* segments   */ &physSegment,
//                           /* max segs   */ 1,
//                           /* max xfer   */ bytesRemaining,
//                           /* xfer size  */ &transferSize) )
//    {
     FEDE_RELOG("FEDE entro al ciclo\n");
	while ( bytesRemaining )
	{
		FEDE_RELOG("FEDE genero 32IOVMSegments\n");
		DMAStatus = currentDMACmd->gen32IOVMSegments( &transferSize, &elSegmento, &numSegmentos);
		if ( ( DMAStatus != kIOReturnSuccess ) || ( numSegmentos != 1 ) || ( elSegmento.fLength == 0 ) )
		{
			
			panic ( "AppleNForceATA::createChannelCommands [%u] status %x segs %d phys %x:%x \n", __LINE__, (unsigned int)DMAStatus, (unsigned int)numSegmentos, (unsigned int)elSegmento.fIOVMAddr, (unsigned int)elSegmento.fLength );
		    break;
		    
		}

        xferDataPtr = (UInt8 *) ((UInt64)elSegmento.fIOVMAddr);
        xferCount   = elSegmento.fLength;

        if ( (uintptr_t) xferDataPtr & 0x01 )
        {
            IOLog("%s: DMA buffer %p not 2 byte aligned\n",
                  getName(), xferDataPtr);
            return kIOReturnNotAligned;        
        }

        if ( xferCount & 0x01 )
        {
            IOLog("%s: DMA buffer length %u is odd\n",
                  getName(), (unsigned int)xferCount);
        }

        // Update bytes remaining count after this pass.
        bytesRemaining -= xferCount;
        xfrPosition += xferCount;
            
        // Examine the segment to see whether it crosses (a) 64k boundary(s)
        starting64KBlock = (UInt8*) ( (uintptr_t) xferDataPtr & 0xffff0000);
        ptr2EndData  = xferDataPtr + xferCount;
        next64KBlock = starting64KBlock + 0x10000;

        // Loop until this physical segment is fully accounted for.
        // It is possible to have a memory descriptor which crosses more
        // than one 64K boundary in a single span.
        
        while ( xferCount > 0 )
        {
            if (ptr2EndData > next64KBlock)
            {
                count2Next64KBlock = next64KBlock - xferDataPtr;
                if ( index < kATAMaxDMADesc )
                {
                    setPRD( xferDataPtr, (UInt16)count2Next64KBlock,
                            &_prdTable[index], kContinue_PRD);
                    
                    xferDataPtr = next64KBlock;
                    next64KBlock += 0x10000;
                    xferCount -= count2Next64KBlock;
                    index++;
                }
                else
                {
                    IOLog("%s: PRD table exhausted error 1\n", getName());
                    _dmaState = kATADMAError;
                    return -1;
                }
            }
            else
            {
                if (index < kATAMaxDMADesc)
                {
                    setPRD( xferDataPtr, (UInt16) xferCount,
                            &_prdTable[index],
                            (bytesRemaining == 0) ? kLast_PRD : kContinue_PRD);
                    xferCount = 0;
                    index++;
                }
                else
                {
                    IOLog("%s: PRD table exhausted error 2\n", getName());
                    _dmaState = kATADMAError;
                    return -1;
                }
            }
        }
    } // end of segment counting loop.

    if (index == 0)
    {
        IOLog("%s: rejected command with zero PRD count (0x%x bytes)\n",
              getName(), (unsigned int)_currentCommand->getByteCount());
        return kATADeviceError;
    }

    // Transfer is satisfied and only need to check status on interrupt.
    _dmaState = kATADMAStatus;
    
    // Chain is now ready for execution.
    return kATANoErr;
}

//---------------------------------------------------------------------------

bool AppleNForceATA::allocDMAChannel( void )
{
	_prdBuffer = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
																  kernel_task,
																  kIODirectionInOut | kIOMemoryPhysicallyContiguous,
																  sizeof(PRD) * kATAMaxDMADesc,
																  0xFFFF0000UL );
    
    if ( !_prdBuffer )
    {
        IOLog("%s: PRD buffer allocation failed\n", getName());
        return false;
    }
	
	_prdBuffer->prepare ( );
	
	_prdTable			= (PRD *) _prdBuffer->getBytesNoCopy();
	_prdTablePhysical	= _prdBuffer->getPhysicalAddress();
	
    _DMACursor = IONaturalMemoryCursor::withSpecification(
                          /* max segment size  */ 0x10000,
                          /* max transfer size */ kMaxATAXfer );
    
    if ( !_DMACursor )
    {
        freeDMAChannel();
        IOLog("%s: Memory cursor allocation failed\n", getName());
        return false;
    }

    // fill the chain with stop commands to initialize it.    
    initATADMAChains( _prdTable );

    return true;
}

//---------------------------------------------------------------------------

bool AppleNForceATA::freeDMAChannel( void )
{
    if ( _prdBuffer )
    {
        // make sure the engine is stopped.
        stopDMA();
		
        // free the descriptor table.
		_prdBuffer->complete();
        _prdBuffer->release();
        _prdBuffer = NULL;
        _prdTable = NULL;
        _prdTablePhysical = 0;
    }
	
    return true;
}

//---------------------------------------------------------------------------

void AppleNForceATA::initATADMAChains( PRD * descPtr )
{
    UInt32 i;

    /* Initialize the data-transfer PRD channel command descriptors. */

    for (i = 0; i < kATAMaxDMADesc; i++)
    {
        descPtr->bufferPtr = 0;
        descPtr->byteCount = 1;
        descPtr->flags = OSSwapHostToLittleConstInt16( kLast_PRD );
        descPtr++;
    }
}

//---------------------------------------------------------------------------

enum {
    kPCIPowerStateOff = 0,
    kPCIPowerStateDoze,
    kPCIPowerStateOn,
    kPCIPowerStateCount
};

void AppleNForceATA::initForPM( IOService * provider )
{
    static const IOPMPowerState powerStates[ kPCIPowerStateCount ] =
    {
        { 1, 0, 0,             0,             0, 0, 0, 0, 0, 0, 0, 0 },
        { 1, 0, IOPMSoftSleep, IOPMSoftSleep, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 1, 0, IOPMPowerOn,   IOPMPowerOn,   0, 0, 0, 0, 0, 0, 0, 0 }
    };

    PMinit();

    registerPowerDriver( this, (IOPMPowerState *) powerStates,
                         kPCIPowerStateCount );

    provider->joinPMtree( this );
}

//---------------------------------------------------------------------------

IOReturn AppleNForceATA::setPowerState( unsigned long stateIndex,
                                           IOService *   whatDevice )
{
    if ( stateIndex == kPCIPowerStateOff )
    {
        fHardwareLostContext = true;
    }
    else if ( fHardwareLostContext )
    {
        initializeHardware();
        programTimingRegisters();
        fHardwareLostContext = false;
    }

    return IOPMAckImplied;
}

//---------------------------------------------------------------------------

void AppleNForceATA::dumpHardwareRegisters( void )
{
    DEBUG_LOG("PCI_IDE_ENABLE   0x%02x\n", fProvider->pciConfigRead8(PCI_IDE_ENABLE));
    DEBUG_LOG("PCI_IDE_CONFIG   0x%02x\n", fProvider->pciConfigRead8(PCI_IDE_CONFIG));
    //DEBUG_LOG("PCI_CABLE_DETECT 0x%02x\n", fProvider->pciConfigRead8(PCI_CABLE_DETECT));
    DEBUG_LOG("PCI_FIFO_CONFIG  0x%02x\n", fProvider->pciConfigRead8(PCI_FIFO_CONFIG));
    DEBUG_LOG("PCI_ULTRA_TIMING 0x%08x\n", (unsigned int)fProvider->pciConfigRead32(PCI_ULTRA_TIMING));

    for (int unit = 0; unit < kMaxDriveCount; unit++)
    {
        if (DRIVE_IS_PRESENT(unit) == false) continue;

        DEBUG_LOG("[ Ch%ld Drive%ld ]\n", fChannelNumber, (long int)unit);
        DEBUG_LOG("Command Active   %ld ns\n", readTimingIntervalNS(kTimingRegCommandActive, unit));
        DEBUG_LOG("Command Recovery %ld ns\n", readTimingIntervalNS(kTimingRegCommandRecovery, unit));
        DEBUG_LOG("Address Setup    %ld ns\n", readTimingIntervalNS(kTimingRegAddressSetup, unit));
        DEBUG_LOG("Data Active      %ld ns\n", readTimingIntervalNS(kTimingRegDataActive, unit));
        DEBUG_LOG("Data Recovery    %ld ns\n", readTimingIntervalNS(kTimingRegDataRecovery, unit));

        if (fBusTimings[unit].ultraEnabled)
        {
            DEBUG_LOG("UDMA Timing      0x%02x\n", readTimingRegister(kTimingRegUltra, unit));
        }
    }
}

void
AppleNForceATA::activateDMAEngine(void)
{
	// clear error bit prior to starting.
	*_bmStatusReg = (UInt8) mBMStatusError | mBMStatusInt | (_currentCommand->getUnit() == 0 ? mBMStatusDrv0 : mBMStatusDrv1);
	OSSynchronizeIO();
	
	// set the address pointer.
	*_bmPRDAddresReg = OSSwapHostToLittleInt32((UInt32) _prdTablePhysical);
	OSSynchronizeIO();
	
	// activate the DMA engine.
	UInt8 theCommand = (_currentCommand->getFlags() & mATAFlagIORead) ? mBMCmdStartInput : mBMCmdStartOutput;
	
	*_bmCommandReg = theCommand;
	OSSynchronizeIO();
}

#pragma mark - nforce controller fix -

IOReturn AppleNForceATA::startDMA( void )
{
	//DEBUG_LOG("%s::%s staring dma...\n", getName(), __FUNCTION__);

	IOReturn err = kATANoErr;

	// first make sure the engine is stopped.
	stopDMA();
	
	
	// reality check the memory descriptor in the current command
	// state flag
	_dmaState = kATADMAStarting;
	
	// create the channel commands
	err = createChannelCommands();
	
	if(	err )
	{
		DEBUG_LOG("%s: error createChannelCmds err = %ld\n", getName(), (long int)err);
		stopDMA();
		return err;
	}
	
	if (fProvider->getHardwareType() != PCI_HW_SATA)
		activateDMAEngine();
	
	return err;

}

IOReturn AppleNForceATA::asyncCommand(void)
{
	IOReturn err = kATANoErr;

	// if DMA, program and activate the DMA channel
	if( (_currentCommand->getFlags() & mATAFlagUseDMA ) == mATAFlagUseDMA )
	{
		err = startDMA();	
	}

	if( err )
	{
		stopDMA();
		return err;
	}

	//DEBUG_LOG("ATAController: command flags = %lx , packet size = %d\n",_currentCommand->getFlags(), _currentCommand->getPacketSize() );

	err = issueCommand();	
	if( err )
		return err;

	// if the command is an atapi command, set state to issue packet and return

	if( (_currentCommand->getFlags() & mATAFlagProtocolATAPI) == mATAFlagProtocolATAPI
		&& _currentCommand->getPacketSize() > 0)

	{
		// set to packet state
		_currentCommand->state = IOATAController::kATAPICmd;			
		return err;
	}

	// if DMA operation, activate the engine then return with status pending.
	
	if( (_currentCommand->getFlags() & mATAFlagUseDMA ) == mATAFlagUseDMA )
	{
		if (fProvider->getHardwareType() == PCI_HW_SATA)
			activateDMAEngine();
		_currentCommand->state = IOATAController::kATAStatus;	
		return err;
	}

	// if PIO write operation, wait for DRQ and send the first sector
	// or sectors if multiple
	if( (_currentCommand->getFlags() 
		& (mATAFlagIOWrite | mATAFlagUseDMA | mATAFlagProtocolATAPI) ) 
		== mATAFlagIOWrite )
	{
	
		// mark the command as data tx state.		
		_currentCommand->state = IOATAController::kATADataTx;
		// send first data segment.
		return asyncData();				
	}
	
	if( (_currentCommand->getFlags() & mATAFlagIORead ) == mATAFlagIORead )
	{
		// read data on next phase.
		_currentCommand->state = IOATAController::kATADataTx;	
	
	}	else {  
	
		// this is a PIO non-data command or a DMA command the next step is to check status.
		_currentCommand->state = IOATAController::kATAStatus;	
	}

	return err;

}
