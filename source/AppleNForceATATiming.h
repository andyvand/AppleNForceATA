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

#ifndef _APPLENFORCEATATIMING_H
#define _APPLENFORCEATATIMING_H

#include "AppleNForceATAHardware.h"

#define kPIOModeCount    7   /* PIO mode 0 to 6 */
#define kDMAModeCount    5   /* DMA mode 0 to 4 */
#define kUDMAModeCount   7   /* Ultra mode 0 to 6 */

typedef struct
{
    UInt16  cycle;      /* t0  min total cycle time */
    UInt16  setup;      /* t1  min address setup time */
    UInt16  active;     /* t2  min command active time */
    UInt16  recovery;   /* t2i min command recovery time */
} TimingParameter;

/*
 * Enumeration of virtual timing registers.
 */
enum TimingReg {
    kTimingRegCommandActive = 0,
    kTimingRegCommandRecovery,
    kTimingRegDataActive,
    kTimingRegDataRecovery,
    kTimingRegAddressSetup,
    kTimingRegUltra,
    kTimingRegCount
};

/*
 * This table returns the PCI config space offset of any virtual
 * timing register indexed by ATA channel number, and drive number.
 */
static const UInt8
TimingRegOffset[kTimingRegCount][kMaxChannelCount][kMaxDriveCount] = 
{
    {   /* kVIATimingRegCommandActive */
        { PCI_CMD_TIMING + 1, PCI_CMD_TIMING + 1 },     /* 0, 1 */ // 1, 1
        { PCI_CMD_TIMING + 0, PCI_CMD_TIMING + 0 }      /* 2, 3 */ // 0, 0
    },
    {   /* kVIATimingRegCommandRecovery */
        { PCI_CMD_TIMING + 1, PCI_CMD_TIMING + 1 },     /* 0, 1 */ // 1, 1
        { PCI_CMD_TIMING + 0, PCI_CMD_TIMING + 0 }      /* 2, 3 */ // 0, 0
    },
    {   /* kVIATimingRegDataActive */
        { PCI_DATA_TIMING + 3, PCI_DATA_TIMING + 2 },   /* 0, 1 */
        { PCI_DATA_TIMING + 1, PCI_DATA_TIMING + 0 }    /* 2, 3 */
    },
    {   /* kVIATimingRegDataRecovery */
        { PCI_DATA_TIMING + 3, PCI_DATA_TIMING + 2 },   /* 0, 1 */
        { PCI_DATA_TIMING + 1, PCI_DATA_TIMING + 0 }    /* 2, 3 */
    },
    {   /* kVIATimingRegAddressSetup */
        { PCI_ADDRESS_SETUP, PCI_ADDRESS_SETUP },       /* 0, 1 */
        { PCI_ADDRESS_SETUP, PCI_ADDRESS_SETUP }        /* 2, 3 */
    },
    {   /* kVIATimingRegUltra */
        { PCI_ULTRA_TIMING + 3, PCI_ULTRA_TIMING + 2 }, /* 0, 1 */
        { PCI_ULTRA_TIMING + 1, PCI_ULTRA_TIMING + 0 }  /* 2, 3 */
    }
};

/*
 * Properties about each virtual timing register.
 */
static const struct {
    UInt8  mask;
    UInt8  shift;
    UInt8  minValue;
    UInt8  maxValue;
} TimingRegInfo[ kTimingRegCount ] =
{
    { 0x0F, 4, 1, 16 },  /* kPCITimingRegCommandActive */
    { 0x0F, 0, 1, 16 },  /* kPCITimingRegCommandRecovery */
    { 0x0F, 4, 1, 16 },  /* kPCITimingRegDataActive */
    { 0x0F, 0, 1, 16 },  /* kPCITimingRegDataRecovery */
    { 0x03, 6, 1,  4 },  /* kPCITimingRegAddressSetup */
    { 0xFF, 0, 0,  0 }   /* kPCITimingRegUltra */
};

/*---------------------------------------------------------------------------
 *
 * PIO
 *
 ---------------------------------------------------------------------------*/

/*
 * Minimum cycle time for each PIO mode number.
 */
static const UInt16 PIOMinCycleTime[ kPIOModeCount ] =
{
    600,   /* Mode 0 */
    383,   /* Mode 1 */
    240,   /* Mode 2 */
    180,   /* Mode 3 */
    120,   /* Mode 4 */
    100,   /* Mode 5 */
     80    /* Mode 6 */
};

/*
 * PIO timing parameters.
 * The setup and recovery times are equal to or larger than the
 * minimal values defined by the ATA spec.
 */
#define kPIOTimingCount 7

static const TimingParameter PIOTimingTable[ kPIOTimingCount ]=
{
   /* Cycle  Setup  Act    Rec */
    { 600,    70,   290,   240 },
    { 383,    50,   290,    93 },
    { 240,    30,   290,    40 },
    { 180,    30,    80,    70 }, 
    { 120,    25,    70,    25 }, 
    { 100,    15,    65,    25 }, 
    {  80,    10,    55,    25 }
};

/*---------------------------------------------------------------------------
 *
 * Multi-word DMA
 *
 ---------------------------------------------------------------------------*/

/*
 * Minimum cycle time for each MW-DMA mode number.
 */
static const UInt16 DMAMinCycleTime[ kDMAModeCount ] =
{
    480,   /* Mode 0 */
    150,   /* Mode 1 */
    120,   /* Mode 2 */
    100,   /* Mode 3 */
     80    /* Mode 4 */
};

/*
 * DMA timing parameters.
 * The tD and tK parameters are equal to or larger than the
 * minimal values defined by the ATA spec.
 */
#define kDMATimingCount 5

static const TimingParameter DMATimingTable[ kDMATimingCount ]=
{
   /* Cycle  Setup   tD     tK */
    { 480,    60,   215,   215 },
    { 150,    45,    80,    50 },
    { 120,    25,    70,    25 },
    { 100,    25,    65,    25 },
    {  80,    25,    55,    20 }
};

/*---------------------------------------------------------------------------
 *
 * Ultra DMA
 *
 ---------------------------------------------------------------------------*/

static const UInt8 UltraTimingTable[kUDMAModeCount] =
{
/* UDMA  0     1     2     3     4     5     6 */
      0xc2, 0xc1, 0xc0, 0xc4, 0xc5, 0xc6, 0xc7
};

#endif /* !_APPLENFORCEATATIMING_H */
