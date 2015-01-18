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

#include <IOKit/IOLib.h>
#include <IOKit/IODeviceTreeSupport.h>
#include "AppleNForceATARoot.h"
#include "AppleNForceATAChannel.h"
#include "AppleNForceATAHardware.h"

static const struct HardwareInfo {
    UInt32       deviceID;
    UInt8        hwType;
    UInt8        hwFlags;
    const char * name;
} HWInfos[] =
{
     /*** IDE ***/
     { PCI_NFORCE1,         PCI_HW_UDMA_100, 0x00, "nForce" },
     { PCI_NFORCE2,         PCI_HW_UDMA_133, 0x00, "nForce2" },
     { PCI_NFORCE2_PRO,     PCI_HW_UDMA_133, 0x00, "nForce2 Pro" },
     { PCI_NFORCE3,         PCI_HW_UDMA_133, 0x00, "nForce3" },
     { PCI_NFORCE3_PRO,     PCI_HW_UDMA_133, 0x00, "nForce3 Pro" },
     { PCI_NFORCE_MCP04,    PCI_HW_UDMA_133, 0x00, "nForce MCP" },
     { PCI_NFORCE_CK804,    PCI_HW_UDMA_133, 0x00, "nForce CK804" },
     { PCI_NFORCE_MCP51,    PCI_HW_UDMA_133, 0x00, "nForce MCP51" },
     { PCI_NFORCE_MCP55,    PCI_HW_UDMA_133, 0x00, "nForce MCP55" },
     { PCI_NFORCE_MCP61,    PCI_HW_UDMA_133, 0x00, "nForce MCP61" },
     { PCI_NFORCE_MCP65,    PCI_HW_UDMA_133, 0x00, "nForce MCP65" },
     { PCI_NFORCE_MCP67,    PCI_HW_UDMA_133, 0x00, "nForce MCP67" },
     { PCI_NFORCE_MCP73,    PCI_HW_UDMA_133, 0x00, "nForce MCP73" },
     { PCI_NFORCE_MCP77,    PCI_HW_UDMA_133, 0x00, "nForce MCP77" },
     /*** SATA ***/
     { PCI_NFORCE2_PRO_S1,  PCI_HW_SATA, 0x00,     "nForce2 Pro Serial ATA" },
     { PCI_NFORCE3_PRO_S1,  PCI_HW_SATA, 0x00,     "nForce3 Pro Serial ATA" },
     { PCI_NFORCE3_PRO_S2,  PCI_HW_SATA, 0x00,     "nForce3 Pro Serial ATA" },
     { PCI_NFORCE_MCP04_S1, PCI_HW_SATA, NV4,      "nForce MCP Serial ATA" },
     { PCI_NFORCE_MCP04_S2, PCI_HW_SATA, NV4,      "nForce MCP Serial ATA" },
     { PCI_NFORCE_CK804_S1, PCI_HW_SATA, NV4,      "nForce CK804 Serial ATA" },
     { PCI_NFORCE_CK804_S2, PCI_HW_SATA, NV4,      "nForce CK804 Serial ATA" },
     { PCI_NFORCE_MCP51_S1, PCI_HW_SATA, NV4|NVQ,  "nForce MCP51 Serial ATA" },
     { PCI_NFORCE_MCP51_S2, PCI_HW_SATA, NV4|NVQ,  "nForce MCP51 Serial ATA" },
     { PCI_NFORCE_MCP55_S1, PCI_HW_SATA, NV4|NVQ,  "nForce MCP55 Serial ATA" },
     { PCI_NFORCE_MCP55_S2, PCI_HW_SATA, NV4|NVQ,  "nForce MCP55 Serial ATA" },
     { PCI_NFORCE_MCP61_S1, PCI_HW_SATA, NV4|NVQ,  "nForce MCP61 Serial ATA" },
     { PCI_NFORCE_MCP61_S2, PCI_HW_SATA, NV4|NVQ,  "nForce MCP61 Serial ATA" },
     { PCI_NFORCE_MCP61_S3, PCI_HW_SATA, NV4|NVQ,  "nForce MCP61 Serial ATA" },
     /*** AHCI ***/
     { PCI_NFORCE_MCP65_A0, PCI_HW_SATA, NVAHCI,   "nForce MCP65 Serial ATA" },
     { PCI_NFORCE_MCP65_A1, PCI_HW_SATA, NVAHCI,   "nForce MCP65 Serial ATA" },
     { PCI_NFORCE_MCP65_A2, PCI_HW_SATA, NVAHCI,   "nForce MCP65 Serial ATA" },
     { PCI_NFORCE_MCP65_A3, PCI_HW_SATA, NVAHCI,   "nForce MCP65 Serial ATA" },
     { PCI_NFORCE_MCP65_A4, PCI_HW_SATA, NVAHCI,   "nForce MCP65 Serial ATA" },
     { PCI_NFORCE_MCP65_A5, PCI_HW_SATA, NVAHCI,   "nForce MCP65 Serial ATA" },
     { PCI_NFORCE_MCP65_A6, PCI_HW_SATA, NVAHCI,   "nForce MCP65 Serial ATA" },
     { PCI_NFORCE_MCP65_A7, PCI_HW_SATA, NVAHCI,   "nForce MCP65 Serial ATA" },
     { PCI_NFORCE_MCP67_A0, PCI_HW_SATA, NVAHCI,   "nForce MCP67 Serial ATA" },
     { PCI_NFORCE_MCP67_A1, PCI_HW_SATA, NVAHCI,   "nForce MCP67 Serial ATA" },
     { PCI_NFORCE_MCP67_A2, PCI_HW_SATA, NVAHCI,   "nForce MCP67 Serial ATA" },
     { PCI_NFORCE_MCP67_A3, PCI_HW_SATA, NVAHCI,   "nForce MCP67 Serial ATA" },
     { PCI_NFORCE_MCP67_A4, PCI_HW_SATA, NVAHCI,   "nForce MCP67 Serial ATA" },
     { PCI_NFORCE_MCP67_A5, PCI_HW_SATA, NVAHCI,   "nForce MCP67 Serial ATA" },
     { PCI_NFORCE_MCP67_A6, PCI_HW_SATA, NVAHCI,   "nForce MCP67 Serial ATA" },
     { PCI_NFORCE_MCP67_A7, PCI_HW_SATA, NVAHCI,   "nForce MCP67 Serial ATA" },
     { PCI_NFORCE_MCP67_A8, PCI_HW_SATA, NVAHCI,   "nForce MCP67 Serial ATA" },
     { PCI_NFORCE_MCP67_A9, PCI_HW_SATA, NVAHCI,   "nForce MCP67 Serial ATA" },
     { PCI_NFORCE_MCP67_AA, PCI_HW_SATA, NVAHCI,   "nForce MCP67 Serial ATA" },
     { PCI_NFORCE_MCP67_AB, PCI_HW_SATA, NVAHCI,   "nForce MCP67 Serial ATA" },
     { PCI_NFORCE_MCP67_AC, PCI_HW_SATA, NVAHCI,   "nForce MCP67 Serial ATA" },
     { PCI_NFORCE_MCP73_A0, PCI_HW_SATA, NVAHCI,   "nForce MCP73 Serial ATA" },
     { PCI_NFORCE_MCP73_A1, PCI_HW_SATA, NVAHCI,   "nForce MCP73 Serial ATA" },
     { PCI_NFORCE_MCP73_A2, PCI_HW_SATA, NVAHCI,   "nForce MCP73 Serial ATA" },
     { PCI_NFORCE_MCP73_A3, PCI_HW_SATA, NVAHCI,   "nForce MCP73 Serial ATA" },
     { PCI_NFORCE_MCP73_A4, PCI_HW_SATA, NVAHCI,   "nForce MCP73 Serial ATA" },
     { PCI_NFORCE_MCP73_A5, PCI_HW_SATA, NVAHCI,   "nForce MCP73 Serial ATA" },
     { PCI_NFORCE_MCP73_A6, PCI_HW_SATA, NVAHCI,   "nForce MCP73 Serial ATA" },
     { PCI_NFORCE_MCP73_A7, PCI_HW_SATA, NVAHCI,   "nForce MCP73 Serial ATA" },
     { PCI_NFORCE_MCP73_A8, PCI_HW_SATA, NVAHCI,   "nForce MCP73 Serial ATA" },
     { PCI_NFORCE_MCP73_A9, PCI_HW_SATA, NVAHCI,   "nForce MCP73 Serial ATA" },
     { PCI_NFORCE_MCP73_AA, PCI_HW_SATA, NVAHCI,   "nForce MCP73 Serial ATA" },
     { PCI_NFORCE_MCP73_AB, PCI_HW_SATA, NVAHCI,   "nForce MCP73 Serial ATA" },
     { PCI_NFORCE_MCP77_A0, PCI_HW_SATA, NVAHCI,   "nForce MCP77 Serial ATA" },
     { PCI_NFORCE_MCP77_A1, PCI_HW_SATA, NVAHCI,   "nForce MCP77 Serial ATA" },
     { PCI_NFORCE_MCP77_A2, PCI_HW_SATA, NVAHCI,   "nForce MCP77 Serial ATA" },
     { PCI_NFORCE_MCP77_A3, PCI_HW_SATA, NVAHCI,   "nForce MCP77 Serial ATA" },
     { PCI_NFORCE_MCP77_A4, PCI_HW_SATA, NVAHCI,   "nForce MCP77 Serial ATA" },
     { PCI_NFORCE_MCP77_A5, PCI_HW_SATA, NVAHCI,   "nForce MCP77 Serial ATA" },
     { PCI_NFORCE_MCP77_A6, PCI_HW_SATA, NVAHCI,   "nForce MCP77 Serial ATA" },
     { PCI_NFORCE_MCP77_A7, PCI_HW_SATA, NVAHCI,   "nForce MCP77 Serial ATA" },
     { PCI_NFORCE_MCP77_A8, PCI_HW_SATA, NVAHCI,   "nForce MCP77 Serial ATA" },
     { PCI_NFORCE_MCP77_A9, PCI_HW_SATA, NVAHCI,   "nForce MCP77 Serial ATA" },
     { PCI_NFORCE_MCP77_AA, PCI_HW_SATA, NVAHCI,   "nForce MCP77 Serial ATA" },
     { PCI_NFORCE_MCP77_AB, PCI_HW_SATA, NVAHCI,   "nForce MCP77 Serial ATA" },
     { PCI_NFORCE_MCP79_A0, PCI_HW_SATA, NVAHCI,   "nForce MCP79 Serial ATA" },
     { PCI_NFORCE_MCP79_A1, PCI_HW_SATA, NVAHCI,   "nForce MCP79 Serial ATA" },
     { PCI_NFORCE_MCP79_A2, PCI_HW_SATA, NVAHCI,   "nForce MCP79 Serial ATA" },
     { PCI_NFORCE_MCP79_A3, PCI_HW_SATA, NVAHCI,   "nForce MCP79 Serial ATA" },
     { PCI_NFORCE_MCP79_A4, PCI_HW_SATA, NVAHCI,   "nForce MCP79 Serial ATA" },
     { PCI_NFORCE_MCP79_A5, PCI_HW_SATA, NVAHCI,   "nForce MCP79 Serial ATA" },
     { PCI_NFORCE_MCP79_A6, PCI_HW_SATA, NVAHCI,   "nForce MCP79 Serial ATA" },
     { PCI_NFORCE_MCP79_A7, PCI_HW_SATA, NVAHCI,   "nForce MCP79 Serial ATA" },
     { PCI_NFORCE_MCP79_A8, PCI_HW_SATA, NVAHCI,   "nForce MCP79 Serial ATA" },
     { PCI_NFORCE_MCP79_A9, PCI_HW_SATA, NVAHCI,   "nForce MCP79 Serial ATA" },
     { PCI_NFORCE_MCP79_AA, PCI_HW_SATA, NVAHCI,   "nForce MCP79 Serial ATA" },
     { PCI_NFORCE_MCP79_AB, PCI_HW_SATA, NVAHCI,   "nForce MCP79 Serial ATA" },
     { PCI_NFORCE_MCP89_A0, PCI_HW_SATA, NVAHCI,   "nForce MCP89 Serial ATA" },
     { PCI_NFORCE_MCP89_A1, PCI_HW_SATA, NVAHCI|NVNOFORCE,   "nForce MCP89 Serial ATA" },
     { PCI_NFORCE_MCP89_A2, PCI_HW_SATA, NVAHCI,   "nForce MCP89 Serial ATA" },
     { PCI_NFORCE_MCP89_A3, PCI_HW_SATA, NVAHCI,   "nForce MCP89 Serial ATA" },
     { PCI_NFORCE_MCP89_A4, PCI_HW_SATA, NVAHCI,   "nForce MCP89 Serial ATA" },
     { PCI_NFORCE_MCP89_A5, PCI_HW_SATA, NVAHCI,   "nForce MCP89 Serial ATA" },
     { PCI_NFORCE_MCP89_A6, PCI_HW_SATA, NVAHCI,   "nForce MCP89 Serial ATA" },
     { PCI_NFORCE_MCP89_A7, PCI_HW_SATA, NVAHCI,   "nForce MCP89 Serial ATA" },
     { PCI_NFORCE_MCP89_A8, PCI_HW_SATA, NVAHCI,   "nForce MCP89 Serial ATA" },
     { PCI_NFORCE_MCP89_A9, PCI_HW_SATA, NVAHCI,   "nForce MCP89 Serial ATA" },
     { PCI_NFORCE_MCP89_AA, PCI_HW_SATA, NVAHCI,   "nForce MCP89 Serial ATA" },
     { PCI_NFORCE_MCP89_AB, PCI_HW_SATA, NVAHCI,   "nForce MCP89 Serial ATA" },
     { 0,                   PCI_HW_UDMA_NONE, 0x00,    "UNKNOWN" }, // we will always return PCI_HW_UDMA_133 for this type of hw.
};

#define super IOService
OSDefineMetaClassAndStructors( AppleNForceATARoot, IOService )

//---------------------------------------------------------------------------
//
// Probe for PCI device and verify that I/O space decoding is enabled.
//

IOService * AppleNForceATARoot::probe( IOService * provider, SInt32 * score )
{
    IOPCIDevice * pciDevice;

    // Let superclass probe first.

    if (super::probe( provider, score ) == 0)
    {
        return 0;
    }

    // Verify the provider is an IOPCIDevice.

    pciDevice = OSDynamicCast( IOPCIDevice, provider );
    if (pciDevice == 0)
    {
        return 0;
    }

    // Fail if I/O space decoding is disabled.

    if ((pciDevice->configRead16( kIOPCIConfigCommand ) &
         kIOPCICommandIOSpace) == 0)
    {
        return 0;
    }

    return this;
}

//---------------------------------------------------------------------------
//
// Start the Root ATA driver. Probe both primary and secondary ATA channels.
//

static void registerClientApplier( IOService * service, void * context )
{
    if ( service ) service->registerService();
}

bool AppleNForceATARoot::start( IOService * provider )
{
#if 0
    OSDictionary * match;
#endif

    if (super::start(provider) != true)
        return false;

    fProvider = OSDynamicCast( IOPCIDevice, provider );
    if (fProvider == 0)
        return false;

    fProvider->retain();
	
	// get device info
	UInt32 hwDeviceID = fProvider->configRead32( kIOPCIConfigVendorID );
	const HardwareInfo* info = &HWInfos[0];
	while ( info->deviceID )
	{
		if ( info->deviceID == hwDeviceID )
			break;
		info++;
	}

	bool is_ahci =
    (
        hwDeviceID == PCI_NFORCE_MCP65_A0 || hwDeviceID == PCI_NFORCE_MCP65_A0 ||
        hwDeviceID == PCI_NFORCE_MCP65_A1 || hwDeviceID == PCI_NFORCE_MCP65_A2 ||
        hwDeviceID == PCI_NFORCE_MCP65_A3 || hwDeviceID == PCI_NFORCE_MCP65_A4 ||
        hwDeviceID == PCI_NFORCE_MCP65_A5 || hwDeviceID == PCI_NFORCE_MCP65_A6 ||
        hwDeviceID == PCI_NFORCE_MCP65_A7 || hwDeviceID == PCI_NFORCE_MCP67_A0 ||
        hwDeviceID == PCI_NFORCE_MCP67_A1 || hwDeviceID == PCI_NFORCE_MCP67_A2 ||
        hwDeviceID == PCI_NFORCE_MCP67_A3 || hwDeviceID == PCI_NFORCE_MCP67_A4 ||
        hwDeviceID == PCI_NFORCE_MCP67_A5 || hwDeviceID == PCI_NFORCE_MCP67_A6 ||
        hwDeviceID == PCI_NFORCE_MCP67_A7 || hwDeviceID == PCI_NFORCE_MCP67_A8 ||
        hwDeviceID == PCI_NFORCE_MCP67_A9 || hwDeviceID == PCI_NFORCE_MCP67_AA ||
        hwDeviceID == PCI_NFORCE_MCP67_AB || hwDeviceID == PCI_NFORCE_MCP67_AC ||
        hwDeviceID == PCI_NFORCE_MCP73_A0 || hwDeviceID == PCI_NFORCE_MCP73_A1 ||
        hwDeviceID == PCI_NFORCE_MCP73_A2 || hwDeviceID == PCI_NFORCE_MCP73_A3 ||
        hwDeviceID == PCI_NFORCE_MCP73_A4 || hwDeviceID == PCI_NFORCE_MCP73_A5 ||
        hwDeviceID == PCI_NFORCE_MCP73_A6 || hwDeviceID == PCI_NFORCE_MCP73_A7 ||
        hwDeviceID == PCI_NFORCE_MCP73_A8 || hwDeviceID == PCI_NFORCE_MCP73_A9 ||
        hwDeviceID == PCI_NFORCE_MCP73_AA || hwDeviceID == PCI_NFORCE_MCP73_AB ||
        hwDeviceID == PCI_NFORCE_MCP77_A0 || hwDeviceID == PCI_NFORCE_MCP77_A1 ||
        hwDeviceID == PCI_NFORCE_MCP77_A2 || hwDeviceID == PCI_NFORCE_MCP77_A3 ||
        hwDeviceID == PCI_NFORCE_MCP77_A4 || hwDeviceID == PCI_NFORCE_MCP77_A5 ||
        hwDeviceID == PCI_NFORCE_MCP77_A6 || hwDeviceID == PCI_NFORCE_MCP77_A7 ||
        hwDeviceID == PCI_NFORCE_MCP77_A8 || hwDeviceID == PCI_NFORCE_MCP77_A9 ||
        hwDeviceID == PCI_NFORCE_MCP77_AA || hwDeviceID == PCI_NFORCE_MCP77_AB ||
        hwDeviceID == PCI_NFORCE_MCP79_A0 || hwDeviceID == PCI_NFORCE_MCP79_A1 ||
        hwDeviceID == PCI_NFORCE_MCP79_A2 || hwDeviceID == PCI_NFORCE_MCP79_A3 ||
        hwDeviceID == PCI_NFORCE_MCP79_A4 || hwDeviceID == PCI_NFORCE_MCP79_A5 ||
        hwDeviceID == PCI_NFORCE_MCP79_A6 || hwDeviceID == PCI_NFORCE_MCP79_A7 ||
        hwDeviceID == PCI_NFORCE_MCP79_A8 || hwDeviceID == PCI_NFORCE_MCP79_A9 ||
        hwDeviceID == PCI_NFORCE_MCP79_AA || hwDeviceID == PCI_NFORCE_MCP79_AB ||
        hwDeviceID == PCI_NFORCE_MCP89_A0 || hwDeviceID == PCI_NFORCE_MCP89_A1 ||
        hwDeviceID == PCI_NFORCE_MCP89_A2 || hwDeviceID == PCI_NFORCE_MCP89_A3 ||
        hwDeviceID == PCI_NFORCE_MCP89_A4 || hwDeviceID == PCI_NFORCE_MCP89_A5 ||
        hwDeviceID == PCI_NFORCE_MCP89_A6 || hwDeviceID == PCI_NFORCE_MCP89_A7 || 
        hwDeviceID == PCI_NFORCE_MCP89_A8 || hwDeviceID == PCI_NFORCE_MCP89_A9 ||
        hwDeviceID == PCI_NFORCE_MCP89_AA || hwDeviceID == PCI_NFORCE_MCP89_AB
    );
	
	if (info->deviceID == 0)
	{
		ERROR_LOG("%s: Error: your chipset [0x%08x]%s\n", getName(), (unsigned int)hwDeviceID, 
			is_ahci ? " is ahci compliant. This driver can't be used." : 
					  " isn't supported by this kext.");
		return false;
	}
	
	setProperty(kHardwareNameKey, info->name);
	fHardwareID = info->deviceID;
	fHardwareName = info->name;
	fHardwareType = info->hwType;
	fHardwareFlags = info->hwFlags;

    // Enable bus master.

    fProvider->setBusMasterEnable( true );

    // Allocate a mutex to serialize access to PCI config space between
    // the primary and secondary ATA channels.

    fPCILock = IOLockAlloc();
    if (fPCILock == 0)
        return false;

    fIsSATA = (getProperty(kSerialATAKey) == kOSBooleanTrue);
	
    fChannels = createATAChannels();
    if (fChannels == 0)
        return false;

    fOpenChannels = OSSet::withCapacity( fChannels->getCount() );
    if (fOpenChannels == 0)
        return false;

	applyToClients( registerClientApplier, 0 );

    return true;
}

//---------------------------------------------------------------------------
//
// Release allocated resources before this object is destroyed.
//

void AppleNForceATARoot::free( void )
{
    if (fChannels)
    {
        fChannels->release();
        fChannels = 0;
    }

    if (fOpenChannels)
    {
        fOpenChannels->release();
        fOpenChannels = 0;
    }

    if (fProvider)
    {
        fProvider->release();
        fProvider = 0;
    }

    if (fPCILock)
    {
        IOLockFree( fPCILock );
        fPCILock = 0;
    }

    super::free();
}

//---------------------------------------------------------------------------
//
// Locate an entry in the device tree that correspond to the channels
// behind the ATA controller. This allows discovery of the ACPI entry
// for ACPI method evaluation, and also uses the ACPI assigned device
// name for a persistent path to the root device.
//

IORegistryEntry * AppleNForceATARoot::getDTChannelEntry( int channelID )
{
    IORegistryEntry * entry = 0;
    const char *      location;

    OSIterator * iter = fProvider->getChildIterator( gIODTPlane );
    if (iter == 0) return 0;

    while (( entry = (IORegistryEntry *) iter->getNextObject() ))
    {
        location = entry->getLocation();
        if ( location && strtol(location, 0, 10) == channelID )
        {
            entry->retain();
            break;
        }
    }

    iter->release();
    
    return entry;  // retain held on the entry
}

//---------------------------------------------------------------------------
//
// Create nubs based on the channel information in the driver personality.
//

OSSet * AppleNForceATARoot::createATAChannels( void )
{
    OSSet *           nubSet;
    OSDictionary *    channelInfo;
    IORegistryEntry * dtEntry;
	char* debugInfo;

    do {
        nubSet = OSSet::withCapacity(2);
        if (nubSet == 0)
            break;

        if (fProvider->open(this) != true)
            break;

        for ( UInt32 channelID = 0; channelID < 2; channelID++ )
        {        
            // Create a dictionary for the channel info. Use native mode
            // settings if possible, else default to legacy mode.

			debugInfo = (char *)"native";
            channelInfo = createNativeModeChannelInfo( channelID );
            if (channelInfo == 0) 
			{
				debugInfo = (char *)"legacy";
                channelInfo = createLegacyModeChannelInfo( channelID );
			}
            if (channelInfo == 0)
                continue;
				
			DEBUG_LOG( "%s::%s() [this=%p] created channel %d in %s mode.\n", 
				getName(), __FUNCTION__, this, (int)channelID, debugInfo );

            // Create a nub for each ATA channel.

            AppleNForceATAChannel * nub = new AppleNForceATAChannel;
            if ( nub )
            {
                dtEntry = getDTChannelEntry( channelID );

                // Invoke special init method in channel nub.

                if (nub->init( this, channelInfo, dtEntry ) &&
                    nub->attach( this ))
                {
                    nubSet->setObject( nub );
                }

                if ( dtEntry )
                {
                    dtEntry->release();
                }
                else
                {
                    // Platform did not create a device tree entry for
                    // this ATA channel. Do it here.

                    char channelName[5] = {'C','H','N','_','\0'};

                    channelName[3] = '0' + channelID;
                    nub->setName( channelName );

                    if (fProvider->inPlane(gIODTPlane))
                    {
                        nub->attachToParent( fProvider, gIODTPlane );
                    }
                }

                nub->release();
            }

            channelInfo->release();
        }
        
        fProvider->close( this );
    }
    while ( false );

    // Release and invalidate an empty set.

    if (nubSet && (nubSet->getCount() == 0))
    {
        nubSet->release();
        nubSet = 0;
    }

    return nubSet;
}

//---------------------------------------------------------------------------

OSDictionary *
AppleNForceATARoot::createNativeModeChannelInfo( UInt32 ataChannel )
{
    UInt8  pi = fProvider->configRead8( PCI_PI );
    UInt16 cmdPort = 0;
    UInt16 ctrPort = 0;

//    // Force native mode configuration for SATA.
//    if (fIsSATA) pi = 0xFF;
	
	// this is not needed, cause the sata will be correctly recognized as native.
	// (checked on ck8-04) -- medevil

    switch ( ataChannel )
    {
        case PRI_CHANNEL_ID:
            if ( pi & 0x3 )
            {
                cmdPort = fProvider->configRead16( kIOPCIConfigBaseAddress0 );
                ctrPort = fProvider->configRead16( kIOPCIConfigBaseAddress1 );
                cmdPort &= ~0x1;  // clear PCI I/O space indicator bit
                ctrPort &= ~0x1;

                // Programming interface byte indicate that native mode
                // is supported and active, but the controller has been
                // assigned legacy ranges. Force legacy mode configuration
                // which is safest. PCI INT# interrupts are not wired
                // properly for some machines in this state.

                if ( cmdPort == PRI_CMD_ADDR &&
                     ctrPort == PRI_CTR_ADDR )
                {
                     cmdPort = ctrPort = 0;
                }
            }
            break;

        case SEC_CHANNEL_ID:
            if ( pi & 0xc )
            {
                cmdPort = fProvider->configRead16( kIOPCIConfigBaseAddress2 );
                ctrPort = fProvider->configRead16( kIOPCIConfigBaseAddress3 );
                cmdPort &= ~0x1;  // clear PCI I/O space indicator bit
                ctrPort &= ~0x1;

                if ( cmdPort == SEC_CMD_ADDR &&
                     ctrPort == SEC_CTR_ADDR )
                {
                     cmdPort = ctrPort = 0;
                }
            }
            break;
    }

    if (cmdPort && ctrPort)
        return createChannelInfo( ataChannel, cmdPort, ctrPort,
                     fProvider->configRead8(kIOPCIConfigInterruptLine) );
    else
        return 0;
}

//---------------------------------------------------------------------------

OSDictionary *
AppleNForceATARoot::createLegacyModeChannelInfo( UInt32 ataChannel )
{
    UInt16  cmdPort = 0;
    UInt16  ctrPort = 0;
    UInt8   isaIrq  = 0;

    switch ( ataChannel )
    {
        case PRI_CHANNEL_ID:
            cmdPort = PRI_CMD_ADDR;
            ctrPort = PRI_CTR_ADDR;
            isaIrq  = PRI_ISA_IRQ;
            break;
        
        case SEC_CHANNEL_ID:
            cmdPort = SEC_CMD_ADDR;
            ctrPort = SEC_CTR_ADDR;
            isaIrq  = SEC_ISA_IRQ;
            break;
    }

    return createChannelInfo( ataChannel, cmdPort, ctrPort, isaIrq );
}

//---------------------------------------------------------------------------

OSDictionary *
AppleNForceATARoot::createChannelInfo( UInt32 ataChannel,
                                    UInt16 commandPort,
                                    UInt16 controlPort,
                                    UInt8 interruptVector )
{
    OSDictionary * dict = OSDictionary::withCapacity( 4 );
    OSNumber *     num;
//	OSObject *     prop;

    if ( dict == 0 || commandPort == 0 || controlPort == 0 || 
         interruptVector == 0 || interruptVector == 0xFF )
    {
        if ( dict ) dict->release();
        return 0;
    }

    num = OSNumber::withNumber( ataChannel, 32 );
    if (num)
    {
        dict->setObject( kChannelNumberKey, num );
        num->release();
    }
    
    num = OSNumber::withNumber( commandPort, 16 );
    if (num)
    {
        dict->setObject( kCommandBlockAddressKey, num );
        num->release();
    }

    num = OSNumber::withNumber( controlPort, 16 );
    if (num)
    {
        dict->setObject( kControlBlockAddressKey, num );
        num->release();
    }

    num = OSNumber::withNumber( interruptVector, 32 );
    if (num)
    {
        dict->setObject( kInterruptVectorKey, num );
        num->release();
    }
	
#if 0
/*
	prop = fProvider->copyProperty( "IOInterruptControllers" );
	if (prop)
	{
		dict->setObject( "IOInterruptControllers", prop );
		prop->release();
	}
	
	prop = fProvider->copyProperty( "IOInterruptSpecifiers" );
	if (prop)
	{
		dict->setObject( "IOInterruptSpecifiers", prop );
		prop->release();
	}
*/
#endif

    return dict;
}

//---------------------------------------------------------------------------
//
// Handle an open request from a client. Multiple channel nubs can hold
// an open on the root driver.
//

bool AppleNForceATARoot::handleOpen( IOService *  client,
                                  IOOptionBits options,
                                  void *       arg )
{
    bool ret = true;

    // Reject open request from unknown clients, or if the client
    // already holds an open.

    if ((fChannels->containsObject(client) == false) ||
        (fOpenChannels->containsObject(client) == true))
        return false;

    // First client open will trigger an open to our provider.

    if (fOpenChannels->getCount() == 0)
        ret = fProvider->open(this);

    if (ret == true)
    {
        fOpenChannels->setObject(client);

        // Return the PCI device to the client
        if ( arg ) *((IOService **) arg) = fProvider;
    }

    return ret;
}

//---------------------------------------------------------------------------
//
// Handle a close request from a client.
//

void AppleNForceATARoot::handleClose( IOService *  client,
                                   IOOptionBits options )
{
    // Reject close request from clients that do not hold an open.

    if (fOpenChannels->containsObject(client) == false) return;

    fOpenChannels->removeObject(client);

    // Last client close will trigger a close to our provider.

    if (fOpenChannels->getCount() == 0)
        fProvider->close(this);
}

//---------------------------------------------------------------------------
//
// Report if the specified client (or any client) has an open on us.
//

bool AppleNForceATARoot::handleIsOpen( const IOService * client ) const
{
    if (client)
        return fOpenChannels->containsObject(client);
    else
        return (fOpenChannels->getCount() != 0);
}

//---------------------------------------------------------------------------

UInt32 AppleNForceATARoot::getHardwareID( void ) const
{
	return fHardwareID;
}

const char* AppleNForceATARoot::getHardwareName( void ) const
{
	return (const char*)fHardwareName;
}

UInt32 AppleNForceATARoot::getHardwareType( void ) const
{
    return fHardwareType;
}

UInt32 AppleNForceATARoot::getHardwareFlags( void ) const
{
    return fHardwareFlags;
}

UInt32 AppleNForceATARoot::getUltraDMAModeMask( void ) const
{
    static const UInt8 HardwareTypeToUDMAModeMask[ PCI_HW_COUNT ] =
    {
        0x00,  /* no  UDMA   */
        0x07,  /* PCI ATA33  */
        0x1F,  /* PCI ATA66  */
        0x3F,  /* PCI ATA100 */
        0x7F,  /* PCI ATA133 */
        0x7F   /* PCI SATA   */
    };
    if (fHardwareType == PCI_HW_UDMA_100)
        return HardwareTypeToUDMAModeMask[ fHardwareType ];
    else
        return HardwareTypeToUDMAModeMask[ PCI_HW_UDMA_133 ];
}

//---------------------------------------------------------------------------

void AppleNForceATARoot::pciConfigWrite8( UInt8 offset, UInt8 data, UInt8 mask )
{
    UInt8 u8;

    IOLockLock( fPCILock );

    u8 = fProvider->configRead8( offset );
    u8 &= ~mask;
    u8 |= (mask & data);
    fProvider->configWrite8( offset, u8 );

    IOLockUnlock( fPCILock );
}

void AppleNForceATARoot::pciConfigWrite16( UInt8 offset, UInt16 data, UInt16 mask )
{
    UInt16 u16;

    IOLockLock( fPCILock );

    u16 = fProvider->configRead16( offset );
    u16 &= ~mask;
    u16 |= (mask & data);
    fProvider->configWrite16( offset, u16 );

    IOLockUnlock( fPCILock );
}


void AppleNForceATARoot::pciConfigWrite32( UInt8 offset, UInt32 data, UInt32 mask )
{
    UInt32 u32;

    IOLockLock( fPCILock );

    u32 = fProvider->configRead32( offset );
    u32 &= ~mask;
    u32 |= (mask & data);
    fProvider->configWrite32( offset, u32 );

    IOLockUnlock( fPCILock );
}

UInt8 AppleNForceATARoot::pciConfigRead8( UInt8 offset )
{
    return fProvider->configRead8( offset );
}

UInt16 AppleNForceATARoot::pciConfigRead16( UInt8 offset )
{
    return fProvider->configRead16( offset );
}

UInt32 AppleNForceATARoot::pciConfigRead32( UInt8 offset )
{
    return fProvider->configRead32( offset );
}
