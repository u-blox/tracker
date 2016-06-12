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
    /// Read the accelerometer.  All pointers may be NULL.
    // \param pX X axis data.
    // \param pY Y axis data.
    // \param pZ Z axis data.
    // \return true if the accelerometer was read successfully, otherwise false.
    bool read(int16_t *pX, int16_t *pY, int16_t *pZ);
    
private:
    /// Debug
    void readDeviceRegisters(uint8_t address, uint32_t reg, uint8_t numValues);
};

#endif
