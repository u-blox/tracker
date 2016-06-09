#include "accelerometer.h"

#define ACCELEROMETER_ADDRESS   ((uint8_t) 0x53)
#define ACCELEROMETER_INT_ENABLE_REG_VALUE 0x54   // Enable the activity, single-tap and free-fall interrupts

// ----------------------------------------------------------------
// INTERRUPT HANDLER
// ----------------------------------------------------------------

#if 0
// Handle for ourselves to allow interrupt function to work
static Accelerometer * gAccelerometer = NULL;

/// Interrupt handler for the accelerometer
void intAccelerometer(void)
{
    char data[2];

    gAccelerometer->pAccelerometerInt->disable_irq();

    // Don't ask me why but this interrupt only works reliably with the
    // ADXL345 chip when this LED flash (or, presumably, an equivalent
    // delay) is inserted.
    //doFlash(1);

    gEventsBitmap = (Accelerometer::EventsBitmap_t) (gEventsBitmap | SensorsHandler::EVENT_MOTION);

    // Read what happened
    data[0] = 0x30;
    data[1] = 0x00;
    gAccelerometer->pI2c->write(ACCELEROMETER_ADDRESS, &(data[0]), 1);
    gAccelerometer->pI2c->read(ACCELEROMETER_ADDRESS, &(data[1]), 1);

    // Activity
    if (data[1] & 0x10)
    {
        gEventsBitmap = (Accelerometer::EventsBitmap_t) (gEventsBitmap | SensorsHandler::EVENT_MOTION_NUDGE);
    }

    // Single tap
    if (data[1] & 0x40)
    {
        gEventsBitmap = (Accelerometer::EventsBitmap_t) (gEventsBitmap | SensorsHandler::EVENT_MOTION_SLAP);
    }

    // Free-fall
    if (data[1] & 0x04)
    {
        gEventsBitmap = (Accelerometer::EventsBitmap_t) (gEventsBitmap | SensorsHandler::EVENT_MOTION_DROPPED);
    }

    gAccelerometer->pAccelerometerInt->enable_irq();
}
#endif

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

/// Power up the accelerometer.
bool Accelerometer::begin(void)
{
    uint8_t bytesWritten;
    bool success = false;
    uint8_t data[2];

    // D2 high (connected to CS bar,
    // and going high puts the chip into
    // I2C mode)
    pinMode(D2, OUTPUT);
    digitalWrite(D2, HIGH);

    // D3 is an interrupt input (INT1)
    pinMode(D3, INPUT);

    // Start the I2C interface
    Wire.setSpeed(CLOCK_SPEED_100KHZ);
    Wire.begin();

    // TODO set up interrupt

    // Setup the ADI XL345 accelerometer
    // Reading register 0x00 should get us back 0xE5
    Wire.beginTransmission(ACCELEROMETER_ADDRESS);
    bytesWritten = Wire.write(0x00);
    Wire.endTransmission();
    if (bytesWritten == 1) {
        if (Wire.requestFrom(ACCELEROMETER_ADDRESS, (uint8_t) 1) == 1) {
            data[0] = Wire.read();
            if (data[0] == 0xE5)
            {
                success = true;
                
                // Set up the interrupts: activity, single-tap and
                // free-fall
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

                data[0] = 0x24;  // Activity threshold (== nudge)
                data[1] = 0x18;  // Quite low
                Wire.beginTransmission(ACCELEROMETER_ADDRESS);
                bytesWritten = Wire.write(data, 2);
                Wire.endTransmission(true);

                if (bytesWritten != 2) {
                    success = false;
                    Serial.printf("Accelerometer: I2C address 0x%02x, unable to set activity threshold register (0x%02x) to value 0x%02x.\n",
                                  ACCELEROMETER_ADDRESS, data[0], data[1]);
                }


                data[0] = 0x27;  // Activity/inactivity control
                data[1] = 0x70;  // Fixed comparison, all axes participating
                Wire.beginTransmission(ACCELEROMETER_ADDRESS);
                bytesWritten = Wire.write(data, 2);
                Wire.endTransmission(true);
            
                if (bytesWritten != 2) {
                    success = false;
                    Serial.printf("Accelerometer: I2C address 0x%02x, unable to set activity/inactivity register (0x%02x) to value 0x%02x.\n",
                                  ACCELEROMETER_ADDRESS, data[0], data[1]);
                }

                data[0] = 0x1D;  // Single-tap threshold (== slap)
                data[1] = 0xC0;  // Quite high
                Wire.beginTransmission(ACCELEROMETER_ADDRESS);
                bytesWritten = Wire.write(data, 2);
                Wire.endTransmission(true);

                if (bytesWritten != 2) {
                    success = false;
                    Serial.printf("Accelerometer: I2C address 0x%02x, unable to set single-tap threshold register (0x%02x) to value 0x%02x.\n",
                                  ACCELEROMETER_ADDRESS, data[0], data[1]);
                }

                data[0] = 0x21;  // Longest duration of single tap in 625 us units
                data[1] = 0xA0;  // 100 ms
                Wire.beginTransmission(ACCELEROMETER_ADDRESS);
                bytesWritten = Wire.write(data, 2);
                Wire.endTransmission(true);

                if (bytesWritten != 2) {
                    success = false;
                    Serial.printf("Accelerometer: I2C address 0x%02x, unable to set single tap duration register (0x%02x) to value 0x%02x.\n",
                                  ACCELEROMETER_ADDRESS, data[0], data[1]);
                }

                data[0] = 0x2A;  // Tap involvement setting
                data[1] = 0x07;  // All axes involved
                Wire.beginTransmission(ACCELEROMETER_ADDRESS);
                bytesWritten = Wire.write(data, 2);
                Wire.endTransmission(true);
            
                if (bytesWritten != 2) {
                    success = false;
                    Serial.printf("Accelerometer: I2C address 0x%02x, unable to set tap axes involvement register (0x%02x) to value 0x%02x.\n",
                                  ACCELEROMETER_ADDRESS, data[0], data[1]);
                }

                data[0] = 0x28;  // Free-fall threshold (== dropped)
                data[1] = 0x05;  // A low value
                Wire.beginTransmission(ACCELEROMETER_ADDRESS);
                bytesWritten = Wire.write(data, 2);
                Wire.endTransmission(true);

                if (bytesWritten != 2) {
                    success = false;
                    Serial.printf("Accelerometer: I2C address 0x%02x, unable to set free-fall threshold register (0x%02x) to value 0x%02x.\n",
                                  ACCELEROMETER_ADDRESS, data[0], data[1]);
                }

                data[0] = 0x29;  // Duration of free fall in 5 ms units
                data[1] = 0x14;  // A low value
                Wire.beginTransmission(ACCELEROMETER_ADDRESS);
                bytesWritten = Wire.write(data, 2);
                Wire.endTransmission(true);
            
                if (bytesWritten != 2) {
                    success = false;
                    Serial.printf("Accelerometer: I2C address 0x%02x, unable to set duration of free fall register (0x%02x) to value 0x%02x.\n",
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
                data[1] = ACCELEROMETER_INT_ENABLE_REG_VALUE;
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

                success = true;
            
            } else {
                Serial.printf("Accelerometer: I2C address 0x%02x, reading value of register 0x00 got 0x%02x but expected 0xE5.\n",
                              ACCELEROMETER_ADDRESS, data[0]);
            }
        } else {
            Serial.printf("Accelerometer: I2C address 0x%02x, timeout reading value of register 0x%02x.\n",
                          ACCELEROMETER_ADDRESS, data[0]);
        }
    } else {
        Serial.printf("Accelerometer: I2C address 0x%02x, unable to write address of register 0x%02x.\n",
                      ACCELEROMETER_ADDRESS, data[0]);
    }

    // For debug purposes
    readDeviceRegisters(ACCELEROMETER_ADDRESS, 0x1D, 30);

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
        Serial.printf("Accelerometer: I2C address 0x%02x, unable to set power control register (0x%02x) to value 0x%02x.\n",
                      ACCELEROMETER_ADDRESS, data[0], data[1]);
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
        Serial.printf("Accelerometer: I2C address 0x%02x, only able to read %d out of the six data registers.\n",
                      ACCELEROMETER_ADDRESS, numBytes);
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