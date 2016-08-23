/*
 * Copyright (C) u-blox, author: Rob Meades (rob.meades@u-blox.com)
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
