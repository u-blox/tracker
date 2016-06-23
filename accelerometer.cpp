#include "accelerometer.h"

#define ACCELEROMETER_ADDRESS   ((uint8_t) 0x53)

// ----------------------------------------------------------------
// INTERRUPT HANDLER
// ----------------------------------------------------------------

/// Interrupt handler for the accelerometer
Accelerometer::EventsBitmap_t Accelerometer::handleInterrupt(void)
{
    Accelerometer::EventsBitmap_t eventsBitmap = Accelerometer::EVENT_NONE;
    char eventReg;

    // Read what happened
    Wire.beginTransmission(ACCELEROMETER_ADDRESS);
    Wire.write(0x30);
    Wire.endTransmission();
    
    if (Wire.requestFrom(ACCELEROMETER_ADDRESS, (uint8_t) 1) == 1) {
        eventReg = Wire.read();

        // Activity
        if (eventReg & 0x10)
        {
            eventsBitmap = (Accelerometer::EventsBitmap_t) (eventsBitmap | Accelerometer::EVENT_ACTIVITY);
        }
    
        // Double tap
        if (eventReg & 0x20)
        {
            eventsBitmap = (Accelerometer::EventsBitmap_t) (eventsBitmap | Accelerometer::EVENT_DOUBLE_TAP);
        }
    
        // Single tap
        if (eventReg & 0x40)
        {
            eventsBitmap = (Accelerometer::EventsBitmap_t) (eventsBitmap | Accelerometer::EVENT_SINGLE_TAP);
        }
    }
    
    return eventsBitmap;
}

// ----------------------------------------------------------------
// GENERIC PRIVATE FUNCTIONS
// ----------------------------------------------------------------

/// Constructor
Accelerometer::Accelerometer(void) {
}

/// Destructor
Accelerometer::~Accelerometer(void) {
}

/// Debug
void Accelerometer::readDeviceRegisters(uint8_t address, uint32_t reg, uint8_t numValues)
{
    uint8_t numBytes;

    Serial.printf("Device 0x%02x: ", address);
    Wire.beginTransmission(address);
    numBytes = Wire.write(reg);
    Wire.endTransmission();
    
    if (numBytes == 1) {
        numBytes = Wire.requestFrom(address, numValues);
        if (numBytes != numValues) {
            Serial.printf("(read %d out of %d value(s) requested) ", numBytes, numValues);
        }
        
        for (uint8_t x = 0; x < numBytes; x++) {
            Serial.printf("%02x: %02x", (int) (reg + x), Wire.read());
            if ((x + 1) < numBytes)
            {
                Serial.printf(", ");
            }
        }
        Serial.printf(".\n");
    } else {
        Serial.printf("unable to write address of register 0x%02x.\n", reg);
    }

}

// ----------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------

/// Set up the accelerometer.
bool Accelerometer::begin(void)
{
    uint8_t bytesWritten;
    bool success = false;
    uint8_t data[2];

    // Start the I2C interface
    Wire.setSpeed(CLOCK_SPEED_100KHZ);
    Wire.begin();

    // Setup the ADI XL345 accelerometer
    // Reading register 0x00 should get us back 0xE5
    Wire.beginTransmission(ACCELEROMETER_ADDRESS);
    bytesWritten = Wire.write(0x00);
    Wire.endTransmission();
    if (bytesWritten == 1) {
        if (Wire.requestFrom(ACCELEROMETER_ADDRESS, (uint8_t) 1) == 1) {
            data[0] = Wire.read();
            if (data[0] == 0xE5) {
                success = true;
                
                Serial.printf("Accelerometer is connected at I2C address 0x%02x.\n", ACCELEROMETER_ADDRESS);
                
                // Set up the interrupts: activity only
                data[0] = 0x2E;
                data[1] = 0x00;  // Disable all interrupts for the moment
                Wire.beginTransmission(ACCELEROMETER_ADDRESS);
                bytesWritten = Wire.write(data, 2);
                Wire.endTransmission(true);
            
                if (bytesWritten != 2) {
                    success = false;
                    Serial.printf("Accelerometer: I2C address 0x%02x, unable to set interrupt register (0x%02x) to value 0x%02x.\n",
                                  ACCELEROMETER_ADDRESS, data[0], data[1]);
                }

                data[0] = 0x24;  // Activity threshold
                data[1] = 0x10;  // Low
                Wire.beginTransmission(ACCELEROMETER_ADDRESS);
                bytesWritten = Wire.write(data, 2);
                Wire.endTransmission(true);

                if (bytesWritten != 2) {
                    success = false;
                    Serial.printf("Accelerometer: I2C address 0x%02x, unable to set activity threshold register (0x%02x) to value 0x%02x.\n",
                                  ACCELEROMETER_ADDRESS, data[0], data[1]);
                }

                data[0] = 0x27;  // Activity/inactivity control
                data[1] = 0xF0;  // Compare changes, all axes participating
                Wire.beginTransmission(ACCELEROMETER_ADDRESS);
                bytesWritten = Wire.write(data, 2);
                Wire.endTransmission(true);
            
                if (bytesWritten != 2) {
                    success = false;
                    Serial.printf("Accelerometer: I2C address 0x%02x, unable to set activity/inactivity register (0x%02x) to value 0x%02x.\n",
                                  ACCELEROMETER_ADDRESS, data[0], data[1]);
                }

                data[0] = 0x2C; // Measurement rate
                data[1] = 0x07; // The lowest rate
                Wire.beginTransmission(ACCELEROMETER_ADDRESS);
                bytesWritten = Wire.write(data, 2);
                Wire.endTransmission(true);
            
                if (bytesWritten != 2) {
                    success = false;
                    Serial.printf("Accelerometer: I2C address 0x%02x, unable to set measurement rate register (0x%02x) to value 0x%02x.\n",
                                  ACCELEROMETER_ADDRESS, data[0], data[1]);
                }

                data[0] = 0x2E;
                data[1] = 0x10;   // Enable the activity interrupt
                Wire.beginTransmission(ACCELEROMETER_ADDRESS);
                bytesWritten = Wire.write(data, 2);
                Wire.endTransmission(true);
            
                if (bytesWritten != 2) {
                    success = false;
                    Serial.printf("Accelerometer: I2C address 0x%02x, unable to set interrupt enable register (0x%02x) to value 0x%02x.\n",
                                  ACCELEROMETER_ADDRESS, data[0], data[1]);
                }

                data[0] = 0x2D; // The power control register
                data[1] = 0x08; // Measurement mode
                Wire.beginTransmission(ACCELEROMETER_ADDRESS);
                bytesWritten = Wire.write(data, 2);
                Wire.endTransmission(true);
    
                if (bytesWritten != 2) {
                    success = false;
                    Serial.printf("Accelerometer: I2C address 0x%02x, unable to set power control register (0x%02x) to value 0x%02x.\n",
                                  ACCELEROMETER_ADDRESS, data[0], data[1]);
                }

            } else {
                Serial.printf("Accelerometer: I2C address 0x%02x, reading value of register 0x00 got 0x%02x but expected 0xE5.\n",
                              ACCELEROMETER_ADDRESS, data[0]);
            }
        } else {
            Serial.printf("Accelerometer: I2C address 0x%02x, timeout reading value of register 0x00.\n", ACCELEROMETER_ADDRESS);
        }
    } else {
        Serial.printf("Accelerometer: I2C address 0x%02x, unable to write the address of register 0x00.\n", ACCELEROMETER_ADDRESS);
    }

    // For debug purposes
    //readDeviceRegisters(ACCELEROMETER_ADDRESS, 0x1D, 30);

    return success;
}

/// Read the accelerometer.
bool Accelerometer::read(int16_t *pX, int16_t *pY, int16_t *pZ)
{
    uint8_t numBytes;
    bool success = true;
    uint8_t data[6];
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;

    data[0] = 0x2D; // The power control register
    data[1] = 0x08; // Measurement mode
    Wire.beginTransmission(ACCELEROMETER_ADDRESS);
    numBytes = Wire.write(data, 2);
    Wire.endTransmission(true);

    if (numBytes != 2) {
        success = false;
        Serial.printf("Accelerometer: I2C address 0x%02x, unable to set power control register (0x%02x) to value 0x%02x.\n", ACCELEROMETER_ADDRESS, data[0], data[1]);
    }

    delay(10);
    Wire.beginTransmission(ACCELEROMETER_ADDRESS);
    numBytes = Wire.write(0x32);
    Wire.endTransmission();
    
    if (numBytes != 1) {
        success = false;
        Serial.printf("Accelerometer: I2C address 0x%02x, unable to set read start position to value 0x32.\n", ACCELEROMETER_ADDRESS);
    }

     // Read all six at once#
    numBytes = Wire.requestFrom(ACCELEROMETER_ADDRESS, (uint8_t) 6);
    if (numBytes == 6) {
        for (uint8_t i = 0; i < sizeof(data); i++) {
            data[i] = Wire.read();
        }
    } else {
        success = false;
        Serial.printf("Accelerometer: I2C address 0x%02x, only able to read %d out of the six data registers.\n", ACCELEROMETER_ADDRESS, numBytes);
    }
    
    x = data[0] + ((int16_t) data[1] << 8);
    y = data[2] + ((int16_t) data[3] << 8);
    z = data[4] + ((int16_t) data[5] << 8);

    if (pX != NULL)
    {
        *pX = x;
    }
    if (pY != NULL)
    {
        *pY = y;
    }
    if (pZ != NULL)
    {
        *pZ = z;
    }

    Serial.printf("Accelerometer: x %d, y %d, z %d.\n", x, y, z);

    return success;
}

/// Set the activity threshold for an interrupt to be triggered
bool Accelerometer::setActivityThreshold(uint8_t threshold)
{
    uint8_t bytesWritten;
    bool success = true;
    uint8_t data[2];

    data[0] = 0x24;  // Activity threshold
    data[1] = threshold;
    Wire.beginTransmission(ACCELEROMETER_ADDRESS);
    bytesWritten = Wire.write(data, 2);
    Wire.endTransmission(true);

    if (bytesWritten != 2) {
        success = false;
        Serial.printf("Accelerometer: I2C address 0x%02x, unable to set activity threshold register (0x%02x) to value 0x%02x.\n",
                      ACCELEROMETER_ADDRESS, data[0], data[1]);
    }

    return success;
}

/// Enable interrupts from the accelerometer
bool Accelerometer::enableInterrupts(void)
{
    uint8_t bytesWritten;
    bool success = true;
    uint8_t data[2];

    // Set up the interrupts: activity only
    data[0] = 0x2E;
    data[1] = 0x00;  // Disable all interrupts for the moment
    Wire.beginTransmission(ACCELEROMETER_ADDRESS);
    bytesWritten = Wire.write(data, 2);
    Wire.endTransmission(true);

    if (bytesWritten != 2) {
        success = false;
        Serial.printf("Accelerometer: I2C address 0x%02x, unable to set interrupt register (0x%02x) to value 0x%02x.\n",
                      ACCELEROMETER_ADDRESS, data[0], data[1]);
    }

    data[0] = 0x24;  // Activity threshold
    data[1] = 0x10;  // Low
    Wire.beginTransmission(ACCELEROMETER_ADDRESS);
    bytesWritten = Wire.write(data, 2);
    Wire.endTransmission(true);

    if (bytesWritten != 2) {
        success = false;
        Serial.printf("Accelerometer: I2C address 0x%02x, unable to set activity threshold register (0x%02x) to value 0x%02x.\n",
                      ACCELEROMETER_ADDRESS, data[0], data[1]);
    }

    data[0] = 0x27;  // Activity/inactivity control
    data[1] = 0xF0;  // Compare changes, all axes participating
    Wire.beginTransmission(ACCELEROMETER_ADDRESS);
    bytesWritten = Wire.write(data, 2);
    Wire.endTransmission(true);

    if (bytesWritten != 2) {
        success = false;
        Serial.printf("Accelerometer: I2C address 0x%02x, unable to set activity/inactivity register (0x%02x) to value 0x%02x.\n",
                      ACCELEROMETER_ADDRESS, data[0], data[1]);
    }

    data[0] = 0x2C; // Measurement rate
    data[1] = 0x07; // The lowest rate
    Wire.beginTransmission(ACCELEROMETER_ADDRESS);
    bytesWritten = Wire.write(data, 2);
    Wire.endTransmission(true);

    if (bytesWritten != 2) {
        success = false;
        Serial.printf("Accelerometer: I2C address 0x%02x, unable to set measurement rate register (0x%02x) to value 0x%02x.\n",
                      ACCELEROMETER_ADDRESS, data[0], data[1]);
    }

    // Call this just to clear any interrupts
    handleInterrupt();

    data[0] = 0x2E;
    data[1] = 0x10;   // Enable the activity interrupt
    Wire.beginTransmission(ACCELEROMETER_ADDRESS);
    bytesWritten = Wire.write(data, 2);
    Wire.endTransmission(true);

    if (bytesWritten != 2) {
        success = false;
        Serial.printf("Accelerometer: I2C address 0x%02x, unable to set interrupt enable register (0x%02x) to value 0x%02x.\n",
                      ACCELEROMETER_ADDRESS, data[0], data[1]);
    }

    data[0] = 0x2D; // The power control register
    data[1] = 0x08; // Measurement mode
    Wire.beginTransmission(ACCELEROMETER_ADDRESS);
    bytesWritten = Wire.write(data, 2);
    Wire.endTransmission(true);

    if (bytesWritten != 2) {
        success = false;
        Serial.printf("Accelerometer: I2C address 0x%02x, unable to set power control register (0x%02x) to value 0x%02x.\n",
                      ACCELEROMETER_ADDRESS, data[0], data[1]);
    }

    return success;
}

/// Disable interrupts from the accelerometer
bool Accelerometer::disableInterrupts(void)
{
    uint8_t bytesWritten;
    bool success = true;
    uint8_t data[2];

    data[0] = 0x2E;
    data[1] = 0x00;  // Disable all interrupts
    Wire.beginTransmission(ACCELEROMETER_ADDRESS);
    bytesWritten = Wire.write(data, 2);
    Wire.endTransmission(true);

    if (bytesWritten != 2) {
        success = false;
        Serial.printf("Accelerometer: I2C address 0x%02x, unable to set interrupt register (0x%02x) to value 0x%02x.\n",
                      ACCELEROMETER_ADDRESS, data[0], data[1]);
    }

    return success;
}