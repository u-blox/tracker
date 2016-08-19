/*
 * Copyright (C) u-blox, author: Rob Meades (rob.meades@u-blox.com)
 * 
 * All rights reserved. 
 * 
 * Permission to use, copy, modify, and distribute this software for any 
 * purpose without fee is hereby granted, provided that this entire notice 
 * is included in all copies of any software which is or includes a copy 
 * or modification of this software and in all copies of the supporting 
 * documentation for such software. 
 * 
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED 
 * WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY 
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY 
 * OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE. 
 */

#include "application.h"

#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

class Accelerometer {
public:
    /// The types of events that can be
    // returned by the interrupt handler.
    typedef enum {
        EVENT_NONE,
        EVENT_ACTIVITY,
        EVENT_SINGLE_TAP,
        EVENT_DOUBLE_TAP,
        MAX_NUM_EVENTS
    } EventsBitmap_t;

    /// Constructor.
    Accelerometer(void);
    /// Destructor.
    ~Accelerometer(void);

    // Handle interrupt events
    EventsBitmap_t handleInterrupt(void);
    
    /// Start the accelerometer.
    bool begin(void);
    /// Configure the accelerometer.
    bool configure(void);
    /// Read the accelerometer.  All pointers may be NULL.
    // \param pX X axis data.
    // \param pY Y axis data.
    // \param pZ Z axis data.
    // \return true if the accelerometer was read successfully, otherwise false.
    bool read(int16_t *pX, int16_t *pY, int16_t *pZ);
    /// Set the activity threshold that should trigger an interrupt.
    // \param threshold  the threshold to set.  The units are 62.5 mg steps.
    // \return true if the threshold was set successfully, otherwise false.
    bool setActivityThreshold(uint8_t threshold);
    /// Check if interrupts are enabled.
    // \return true if interrupts are enabled, otherwise false.
    bool areInterruptsEnabled(void);
    /// Enable interrupts.
    // \return true if interrupts were switched on successfully, otherwise false.
    bool enableInterrupts(void);
    /// Disable interrupts.
    // \return true if interrupts were switched off successfully, otherwise false.
    bool disableInterrupts(void);
    
private:
    /// Debug
    void readDeviceRegisters(uint8_t address, uint32_t reg, uint8_t numValues);
};

#endif
