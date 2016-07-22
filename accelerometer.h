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
