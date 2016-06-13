#include "accelerometer.h"
#include "TinyGPS.h"

/* Tracker
 *
 * This code is for a tracker, borrowing from AssetTracker
 * but using a u-blox GPS module and all through-hole components
 * so that it can be hand assembled (or built using a breadboard.
 * It uses a Particle Electron board (though it is also
 * pin-compatible with a Partilce Photon board since the lower
 * pins are not used), a u-blox PAM7Q GPS module and an ADXL345
 * accelerometer break-out board to establish the GPS position
 * in a power-efficient way.  It reports data to the Particle
 * server and, from there, via webhooks, anyone who is interested.
 *
 * There are some other differences to AssetTracker:
 *
 * It is expected that GPS position is sent quite frequently and
 * so provision is made for use of multiple batteries.  NOTE:
 * BE VERY CAREFUL with this.  There are specific things you need
 * to do for this to be safe.
 *
 * Processor sleep (but with network idle) is used to save
 * power between GPS fixes.
 *
 * In order to use power most efficiently, this program wakes up
 * and shuts down (to deep sleep this time) the entire system
 * so that it only operates during the working day.
 *
 * The program is designed to begin operation on a specific date,
 * so that a device can be prepared in advance, placed into
 * position, and then will begin operation as intended on the given
 * day.
 *
 * MESSAGES
 *
 * The chosen (JSON) message formats are hilghly compressed to
 * save data and are of the following form:
 *
 * gps: 353816058851462;51.283645;-0.776569;1465283731
 *
 * ...where the first item is the 15-digit IMEI, the second one
 * the latitude in degrees as a float, the third one the longitude
 * in degrees as a float and the last one the timestamp in Unix
 * time (UTC).  All fields must be present. This is sent when
 * motion begins and periodically while moving.
 * 
 * telemetry: 353816058851462;80.65;-70;1465283731
 *
 * ...where the first item is the 15-digit IMEI, the second one
 * the battery left as a percentage, the third the signal strength
 * in dBm and the last one the timestamp in Unix time (UTC). All
 * fields must be present.  This is sent periodically.
 *
 * NOTE: it you want to use a different message format, maybe
 * using a more standard (though less efficient) JSON approach,
 * simply modify queueTelemeteryReport() and queueGpsReport()
 * to match.
 *
 * NOTE on webhooks: Particle webhooks are really neat but they
 * can be a bit confusing to code.  First of all, when you
 * use the "json" field, you must quote a key/value pair, so even
 * if you only want to send on what Particle received (using the
 * {{PARTICLE_EVENT_VALUE}}) you can't.
 *
 * Then, you will see in your Particle web log something like:
 *
 * hook-sent/telemetry   undefined   time-date
 *
 * The "undefined" bit looks bad but actually it's normal; hook-sent
 * is just an indication that your info was sent on, it contains
 * no data itself, hence the "undefined".
 *
 * Lastly, if you connect to a site that reflects back what it
 * receives, you will see a hook-response something like:
 *
 * {"data":"{\"telemetry\":\"the stuff you actually sent","ttl":"60","published_at":"2016-06-08T09:27:10.130Z","coreid":"particle-internal","name":"hook-response/telemetry/0"}
 *
 * What the other end is actually _receiving_ is the value part
 * of the "data" key, so in this example:
 *
 * "telemetry":"the stuff you actually sent"
 *
 * The rest is just stuff that Particle inserts for your info.
 * Note that any quotes inside the stuff you actually sent will
 * be converted to &quot; (URL-style) so you will need to make
 * sure that whoever is parsing your data can cope with that.
 */

/****************************************************************
 * MACROS
 ***************************************************************/

// The length of the IMEI, used as the device ID.
#define IMEI_LENGTH 15

// The maximum amount of time to hang around waiting for a
// connection to the Particle server.
#define WAIT_FOR_CONNECTION_SECONDS 60

// How long to wait for things to sort themselves out (e.g. the
// USB port) after waking up from sleep.
#define WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS 5

// How long to wait for the Particle server to respond to
// a time sync request
#define TIME_SYNC_WAIT_SECONDS 10

// If the system cannot establish time by talking to the 
// Particle server, wait this long and try again.
// NOTE: must be bigger than WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS.
#define TIME_SYNC_RETRY_PERIOD_SECONDS 30

// The minimum time between GPS fixes when moving.  This
// must always be the minimum period because it is also used
// as the sleep period.
// NOTE: must be bigger than WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS.
#define GPS_PERIOD_SECONDS 30

// The wake-up period in seconds.  This should be equal to the
// shortest wake-up period, which is usually the GPS one.
#define WAKEUP_PERIOD_SECONDS GPS_PERIOD_SECONDS

// The periodicity of telemetry reports.
#define TELEMETRY_PERIOD_SECONDS 600

// The report period in seconds .
#define REPORT_PERIOD_SECONDS 120

// The size of a JSON record.
#define LEN_RECORD 60

// The queue length at which to try sending a record.
#define QUEUE_SEND_LEN 4

// The time allowed for GPS to sync each time, once
// it has initially synchronised
#define GPS_FIX_TIME_SECONDS 10

// The minimum possible value of Time (in Unix, UTC).
#define MIN_TIME_UNIX_UTC 1451606400 // 1 Jan 2016 @ midnight

// The start time of daily operation (in Unix, UTC).
// Use http://www.unixtimestamp.com/ to work this out.
#define START_TIME_UNIX_UTC 1465718400 // 12 June 2016 @ 08:00

// Start of day in seconds after midnight.
#define START_OF_WORKING_DAY_SECONDS 28800 // 08:00

// Duration of a workingi day in seconds.
#define LENGTH_OF_WORKING_DAY_SECONDS 36000 // 10 hours, so day ends at 18:00

// Define this have the system run irrespective of the overall start time
// and working day start/stop times above.
#define DISABLE_WAKEUP_TIMES

// Define this to do GPS readings irrespective of the state of the
// accelerometer
//#define DISABLE_ACCELEROMETER

// Define this to record GPS values even if the values are not valid.
// The invalid value is TinyGPS::GPS_INVALID_F_ANGLE, which comes
// out as 1000 degrees.
//#define IGNORE_INVALID_GPS

/****************************************************************
 * TYPES
 ***************************************************************/

// Struct to hold an accelerometer reading
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} AccelerometerReading_t;

// The possible record types
typedef enum {
    RECORD_TYPE_NULL,
    RECORD_TYPE_TELEMETRY,
    RECORD_TYPE_GPS,
    MAX_NUM_RECORD_TYPES
} RecordType_t;

// A single record
typedef struct {
    bool isUsed;
    RecordType_t type;
    char contents[LEN_RECORD];
} Record_t;

// For debugging: LED flash types
typedef enum {
    DEBUG_IND_OFF,
    DEBUG_IND_TOGGLE,
    DEBUG_IND_GPS_FIX,
    DEBUG_IND_ACTIVITY,
    MAX_NUM_DEBUG_INDS
} DebugInd_t;

/****************************************************************
 * GLOBAL VARIABLES
 ***************************************************************/

// Record type strings
// NOTE: must match the RecordType enum above
const char * recordTypeString[] = {"null", "telemetry", "gps"};

// The IMEI of the module
char imei[IMEI_LENGTH] = "";

// The amount of time to sleep for
time_t sleepForSeconds;

// When we last tried to get a fix
time_t lastGpsSeconds = 0;

// Time Of the last ping message
time_t lastPingSeconds = 0;

// Time Of the last telemetry message
time_t lastTelemetrySeconds = 0;

// Time Of the last report
time_t lastReportSeconds = 0;

// The number of fixes after which to
// publish what we've got
uint32_t pubInterval = 4;

// The last lat/long readings
float lastLatitude = TinyGPS::GPS_INVALID_F_ANGLE;
float lastLongitude = TinyGPS::GPS_INVALID_F_ANGLE;

// The records accumulated
Record_t records[50];

// The current record
uint8_t currentRecord = 0;

// The next record to send
uint8_t nextPubRecord = currentRecord;

// The number of records queued
uint8_t numRecordsQueued = 0;

// Track if this is our first tine through
bool firstTime = true;

// Set this to force a send attempt
bool forceSend = false;

// The time we went to sleep
time_t sleepTime = 0;

// Interrupt flag
volatile bool intFlag = false;

// Boolean to flag that we're moving
bool inMotion = true;

// Count the number of times around the loop, for info
uint32_t numLoops = 0;

// Count the number of loops for which a GPS fix was attempted, for info
uint32_t numLoopsGpsRunning = 0;

// Count the number of loops for which a GPS fix was achieved, for info
uint32_t numLoopsGpsFix = 0;

// Count the number of seconds we've been asleep for
uint32_t totalSleepSeconds = 0;

// Count the number of seconds GPS has been on for
time_t gpsPowerOnTime = 0;
uint32_t totalGpsSeconds = 0;

// Hold the last accelerometer reading
AccelerometerReading_t accelerometerReading;

// Instantiate GPS
TinyGPS gps;

// Instantiate the accelerometer
Accelerometer accelerometer = Accelerometer();

// The pin that the accelerometer interrupt is attached to
const int intPin = WKP;

// Instantiate a fuel gauge
FuelGauge fuel;

// Only connect when it is required
SYSTEM_MODE(SEMI_AUTOMATIC);

/****************************************************************
 * FUNCTION PROTOTYPES
 ***************************************************************/

static void intPinOccurred();
static bool attachInterrupt(int pin);
static void handleInterrupt();
static void debugInd(DebugInd_t debugInd);
static int getImeiCallBack(int type, const char* pBuf, int len, char* imei);
static uint8_t incModRecords (uint8_t x);
static time_t gpsOn();
static void gpsOff();
static bool gpsUpdate(time_t onTimeSeconds, float *pLatitude, float *pLongitude);
static void updateTotalGpsSeconds();
static void connect();
static bool establishTime();
static char * getRecord(RecordType_t type);
static void queueTelemetryReport();
static void queueGpsReport(float latitude, float longitude);

/****************************************************************
 * STATIC FUNCTIONS
 ***************************************************************/

// Deal with an interrupt from intPin
static void intPinOccurred() {
    // No bandwidth to do anything here, just flag it
    intFlag = true;
    debugInd(DEBUG_IND_TOGGLE);
}

// Attach the intPinOccurred() interrupt function to the given pin
static bool attachInterrupt(int pin) {
    bool success = false;
    
    if (attachInterrupt(pin, intPinOccurred, RISING)) {
        success = true;
        Serial.printf ("Interrupt attached to pin %d.\n", pin);
    } else {
        Serial.println ("WARNING: couldn't attach interrupt to pin.");
    }
    
    return success;
}

// Handle the accelerometer interrupt, though do NOT call this from
// an interrupt context as it does too much
static void handleInterrupt() {
    Accelerometer::EventsBitmap_t eventsBitmap;
    
    // Disable interrupts
    noInterrupts();
    accelerometer.read (&accelerometerReading.x, &accelerometerReading.y, &accelerometerReading.z);
    eventsBitmap = accelerometer.handleInterrupt();
    // Re-enable interrupts
    interrupts();
    
    // Flag if we're in motion and switch GPS
    // on to get a fix
    if (eventsBitmap & Accelerometer::EVENT_ACTIVITY) {
        inMotion = true;
        debugInd(DEBUG_IND_ACTIVITY);
    }
}

// Parse the IMEI from the module
static int getImeiCallBack(int type, const char* pBuf, int len, char* imei) {
    
    if ((type == TYPE_UNKNOWN) && imei) {
        if (sscanf(pBuf, "\r\n%15s\r\n", imei) == 1) {
            Serial.printf("IMEI is: %.*s.\n", IMEI_LENGTH, imei);
        }
    }

    return WAIT;
}

// Use the LED on the board for debug indications.
// NOTE: this may be called from an interrupt context.
static void debugInd(DebugInd_t debugInd) {
    
    switch (debugInd) {
        case DEBUG_IND_OFF:
            digitalWrite(D7, LOW);
        break;
        case DEBUG_IND_TOGGLE:
            // This one is the lowest-cost debug option
            digitalWrite(D7, !(digitalRead(D7)));
        break;
        case DEBUG_IND_GPS_FIX:
            digitalWrite(D7, LOW);
            delay (25);
            digitalWrite(D7, HIGH);
            delay (25);
            digitalWrite(D7, LOW);
            delay (25);
        break;
        case DEBUG_IND_ACTIVITY:
            digitalWrite(D7, LOW);
            delay (25);
            digitalWrite(D7, HIGH);
            delay (100);
            digitalWrite(D7, LOW);
            delay (25);
        break;
        default:
        break;
    }
}

// Increment a number and return the incremented
// number modulo the number of records available
static uint8_t incModRecords (uint8_t x) {
    x++;
    if (x >= (sizeof (records) / sizeof (Record_t))) {
        x = 0;
    }
    
    return x;
}

// Switch GPS on, returning the time that
// GPS was switched on
static time_t gpsOn() {
    // If it's already on, total up the time
    if (!digitalRead(D2)) {
        gpsPowerOnTime = Time.now();
    }
    digitalWrite(D2, HIGH);
    
    return Time.now();
}

// Switch GPS off
static void gpsOff() {
    if (digitalRead(D2)) {
        digitalWrite(D2, LOW);
        updateTotalGpsSeconds();
    }
}

// Update the GPS, making sure it's been on for
// enough time to give us a fix. If onTimeSeconds
// is non-zero then it represents the time at which GPS
// was switched on.  pLatitude and pLongitude are filled in
// with fix values if there is one (and true is returned),
// otherwise they are left alone.
static bool gpsUpdate(time_t onTimeSeconds, float *pLatitude, float *pLongitude) {
    bool fixAchieved = false;
    uint32_t startTimeSeconds = Time.now();
    float latitude;
    float longitude;

    Serial.printf("Checking for GPS fix for up to %d second(s):\n", GPS_FIX_TIME_SECONDS);

    // Make sure GPS is switched on
    gpsOn();

    if (onTimeSeconds != 0) {
        startTimeSeconds = onTimeSeconds;
    }

    while (!fixAchieved && (Time.now() - startTimeSeconds < GPS_FIX_TIME_SECONDS)) {
        // Check GPS data is available
        while (!fixAchieved && Serial1.available()) {
            // parse GPS data
            char c = Serial1.read();
            Serial.printf("%c", c);
            if (gps.encode(c)) {
                gps.f_get_position(&latitude, &longitude);
                if ((latitude != TinyGPS::GPS_INVALID_F_ANGLE) && (longitude != TinyGPS::GPS_INVALID_F_ANGLE)) {
                    // Indicate that we have a fix
                    debugInd (DEBUG_IND_GPS_FIX);

                    fixAchieved = true;
                    
                    if (pLatitude) {
                        *pLatitude = latitude;
                    }
                    if (pLongitude) {
                        *pLongitude = longitude;
                    }
                }
            }
        }
    }
    
    Serial.print("\n");
    if ((gps.satellites() != 255) && (gps.satellites() > 0)) {
        Serial.printf("%d satellites visible.\n", gps.satellites());
    }
    if (fixAchieved){
        Serial.printf("Fix achieved in %d second(s): latitude: %.6f, longitude: %.6f.\n",
                      Time.now() - startTimeSeconds, latitude, longitude);
    } else {
        Serial.printf("No fix yet.\n");
    }
    
    // Update the stats
    updateTotalGpsSeconds();

#ifndef DISABLE_GPS_POWER_SAVING
    // If GPS has achieved a fix, switch it off
    if (fixAchieved) {
        gpsOff();
    }
#endif
    
    return fixAchieved;
}

// Keep track of some stats for info
static void updateTotalGpsSeconds() {
    uint32_t x = Time.now() - gpsPowerOnTime;
    
    // Ignore silly values which could result if
    // the timebase underneath us is update between
    // gpsPowerOnTime and now
    if (x < 31536000) {
        totalGpsSeconds += x;
    }
    
    gpsPowerOnTime = Time.now();
}

// Connect to the network
static void connect() {
    if (!Particle.connected()) {
        Serial.printf("Connecting to network (waiting for up to %d seconds)... ", WAIT_FOR_CONNECTION_SECONDS);
        Particle.connect();
        if (waitFor(Particle.connected, WAIT_FOR_CONNECTION_SECONDS)) {
            Serial.println("Connected.");
        } else {
            Serial.println("WARNING: connection failed.");
        }
    } else {
        Serial.println("Already connected to network.");
    }
}

// Make sure we have time sync
static bool establishTime() {
    bool success = false;
    
    if (Time.now() < MIN_TIME_UNIX_UTC) {
        Serial.printf("Syncing time (this will take %d second(s)).\n", TIME_SYNC_WAIT_SECONDS);
        connect();
        Particle.syncTime();
        delay(TIME_SYNC_WAIT_SECONDS * 1000);
    }
    
    if (Time.now() >= MIN_TIME_UNIX_UTC) {
        success = true;
    }

    return success;
}

// Get a record
static char * getRecord(RecordType_t type) {
    char * pContents;

    Serial.printf("Using record %d.\n", currentRecord);
    
    if (records[currentRecord].isUsed) {
        Serial.println("WARNING: records queue has wrapped, over-writing old data.");
    } else {
        numRecordsQueued++;
    }
    
    records[currentRecord].isUsed = true;
    records[currentRecord].type = type;
    pContents = records[currentRecord].contents;
    
    // Move the index on to the next record
    currentRecord = incModRecords(currentRecord);
    Serial.printf("Incremented currentRecord to %d.\n", currentRecord);
    
    return pContents;
}

// Queue a telemetry report
static void queueTelemetryReport() {
    char *pRecord;
    uint32_t contentsIndex = 0;
    
    Serial.println("Queuing a telemetry report.");
    
    // Start a new record
    pRecord = getRecord(RECORD_TYPE_TELEMETRY);

    // Add the device ID
    contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, "%.*s", IMEI_LENGTH, imei);  // -1 for terminator

    // Add battery status
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%.2f", fuel.getSoC());  // -1 for terminator
        Serial.printf("Battery level is %f.\n", fuel.getSoC());
    } else {
        Serial.println("WARNING: couldn't fit battery level into report.");
    }
    
    // Add signal strength
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        CellularSignal signal = Cellular.RSSI();
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%d", signal.rssi); // -1 for terminator
        Serial.printf("RSSI is %d.\n", signal.rssi);
    } else {
        Serial.println("WARNING: couldn't fit Signal Strength reading into report.");
    }
    
    // Add Unix time
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%u", (unsigned int) (uint32_t) Time.now());  // -1 for terminator
        Serial.printf("Time now is %s UTC.\n", Time.timeStr().c_str());
    } else {
        Serial.println("WARNING: couldn't fit timestamp into report.");
    }
    
    // Add the closing brace (and the terminator)
    Serial.printf ("%d bytes of record used (%d unused).\n", contentsIndex + 1, LEN_RECORD - (contentsIndex + 1)); // +1 to account for terminator
}

// Queue a GPS report
static void queueGpsReport(float latitude, float longitude) {
    char *pRecord;
    uint32_t contentsIndex = 0;

    Serial.println("Queuing a GPS report.");
    
    // Start a new record
    pRecord = getRecord(RECORD_TYPE_GPS);

    // Add the device ID
    contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, "%.*s", IMEI_LENGTH, imei);  // -1 for terminator

    // Add GPS
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%.6f;%.6f", latitude, longitude);  // -1 for terminator
    } else {
        Serial.println("WARNING: couldn't fit GPS reading into report.");
    }
        
    // Add Unix time
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%u", (unsigned int) (uint32_t) Time.now());  // -1 for terminator
        Serial.printf("Time now is %s UTC.\n", Time.timeStr().c_str());
    } else {
        Serial.println("WARNING: couldn't fit timestamp into report.");
    }
    
    Serial.printf ("%d byte(s) of record used (%d unused).\n", contentsIndex + 1, LEN_RECORD - (contentsIndex + 1)); // +1 to account for terminator
}

/****************************************************************
 * GLOBAL FUNCTIONS
 ***************************************************************/

void setup() {
    
    // Set time to zero when we start so that we know if time
    // is synchronised or not later on
    Time.setTime(0);
    
    // Opens up a Serial port so you can listen over USB
    Serial.begin(9600);
    
    // Set D3 to be an ouput and zero so that we can use it as a sort
    // of NULL pin paremeter to the sleep() function
    pinMode(D3, OUTPUT);
    digitalWrite (D3, LOW);
    
    // Set D7 to be an output (for the debug LED on the
    // Particle board) and switch it off
    pinMode(D7, OUTPUT);
    debugInd(DEBUG_IND_OFF);

    // Set D2 to be an output (driving power to the GPS module)
    // and set it to ON to allow GPS position to be established
    // while we do everything else
    pinMode(D2, OUTPUT);
    digitalWrite (D2, LOW);
    gpsOn();

    // After a reset of the Electron board it takes a Windows PC several seconds
    // to sort out what's happened to its USB interface, hence you need this
    // delay if you're going to capture the serial output from here.
    delay (5000);
    
    // Connect to the network so as to establish time
    connect();
    establishTime();

    // Set up all the necessary accelerometer bits
    memset (&accelerometerReading, 0, sizeof (accelerometerReading));
    accelerometer.begin();

    // Start the serial port for the GPS module
    Serial1.begin(9600);

    // Set all the records used to false
    memset (records, false, sizeof (records));

    // Get the IMEI of the module
    Serial.println("Getting IMEI...");
    Cellular.command(getImeiCallBack, imei, 10000, "AT+CGSN\r\n");
    Serial.println("Started.");
    
    // Attach an interrupt to the wake-up pin so that to handle
    // the accelerometer interrupt (while we're not sleeping)
    attachInterrupt(intPin);
}

void loop() {
    bool inTheWorkingDay = false;
    time_t gpsOnTime = 0;

    // This only for info
    numLoops++;
    
    // Deal with the interrupt here, nice and calm, out of interrupt context
    // NOTE: always read it, irrespective of the flag, as if we happened
    // to miss the interrupt for some reason we'd get stuck if we didn't
    // handle it (which is also the thing that clears it)
    handleInterrupt();

    if (!firstTime) {
#ifdef DISABLE_ACCELEROMETER
        inMotion = true;
#endif
        // Switch GPS on early so that it can get fixing.
        if (inMotion && (Time.now() - lastGpsSeconds >= GPS_PERIOD_SECONDS)) {
            gpsOnTime = gpsOn();
        }
        
        // It seems counterintuitive to add a delay here but, if you don't
        // then, on wake-up from sleep, one or other of the calls below
        // stalls for some time (maybe one of the serial functions) and so
        // it actually saves power to wait a little while here.
        delay (WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS * 1000);
    }

    // Having valid time is fundamental to this program and so, if time
    // has not been established (via a connection to the Particle server),
    // try to establish it again
    if (establishTime()) {
#ifdef DISABLE_WAKEUP_TIMES
        if (true) {
#else
        if (Time.now() >= START_TIME_UNIX_UTC) {
#endif
            uint32_t secondsSinceMidnight = Time.hour() * 3600 + Time.minute() * 60 + Time.second();
#ifdef DISABLE_WAKEUP_TIMES
            if (true) {
#else
            if ((secondsSinceMidnight >= START_OF_WORKING_DAY_SECONDS) && (secondsSinceMidnight <= START_OF_WORKING_DAY_SECONDS + LENGTH_OF_WORKING_DAY_SECONDS)) {
#endif
                inTheWorkingDay = true;

                if (firstTime || (Time.now() >= sleepTime + WAKEUP_PERIOD_SECONDS)) {
                    Serial.printf("\nAwake: time now is %s UTC, slept since %s UTC (%d second(s) ago).\n",
                                  Time.timeStr().c_str(), Time.timeStr(sleepTime).c_str(), Time.now() - sleepTime);
                                  
                    sleepForSeconds = WAKEUP_PERIOD_SECONDS - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS;
                    
                    // I have seen IMEI retrieval fail so check again here
                    if (imei[0] < '0') {
                        Serial.println("WARNING: trying to get IMEI again...");
                        Cellular.command(getImeiCallBack, imei, 10000, "AT+CGSN\r\n");
                    }
                
                    // Queue a telemetry report, if required
                    if (Time.now() - lastTelemetrySeconds >= TELEMETRY_PERIOD_SECONDS) {
                        lastTelemetrySeconds = Time.now();
                        queueTelemetryReport();
                        Serial.println("Forcing a send.");
                        forceSend = true;
                    }
                    
                    // Queue a GPS report, if required
                    if ((Time.now() - lastGpsSeconds >= GPS_PERIOD_SECONDS)) {
                        // If we've moved, take a new reading
                        if (inMotion) {
                            numLoopsGpsRunning++;
                            Serial.println ("*** Motion was detected, getting latest GPS reading.");
                            // Get the latest output from GPS
                            gpsUpdate(gpsOnTime, &lastLatitude, &lastLongitude);
                            // Reset the flag
                            inMotion = false;
                        }
                    
                        // Only report if we have a fix
#ifdef IGNORE_INVALID_GPS
                        if (true) {
#endif
                        if ((lastLatitude != TinyGPS::GPS_INVALID_F_ANGLE) && (lastLongitude != TinyGPS::GPS_INVALID_F_ANGLE)) {
                            numLoopsGpsFix++;
                            lastGpsSeconds = Time.now();
                            queueGpsReport(lastLatitude, lastLongitude);
                        }
                    }
                    
                    // Check if it's time to publish
                    if (forceSend || ((Time.now() - lastReportSeconds >= REPORT_PERIOD_SECONDS) && (numRecordsQueued >= QUEUE_SEND_LEN))) {
                        uint8_t sentCount = 0;
                        uint8_t failedCount = 0;
                        uint8_t x;
                
                        if (forceSend) {
                            Serial.println ("Force Send was set.");
                        }
                        forceSend = false;
                        
                        // Publish any records that are marked as used
                        x = nextPubRecord;
                        Serial.printf("Sending reports (numRecordsQueued %d, nextPubRecord %d, currentRecord %d).\n", numRecordsQueued, x, currentRecord);
                        connect();
                
                        do {
                            Serial.printf("Report %d: ", x);
                            if (records[x].isUsed) {
                                if (Particle.connected() && Particle.publish(recordTypeString[records[x].type], records[x].contents, 60, PRIVATE)) {
                                    Serial.printf("sent %s.\n", records[x].contents);
                                    records[x].isUsed = false;
                                    if (numRecordsQueued > 0) {
                                        numRecordsQueued--;
                                    }
                                    sentCount++;
                                    // Only four publish operations can be performed per second, so
                                    // put in a 1 second delay every four
                                    if (sentCount % 4 == 0) {
                                        delay (1000);
                                    }
                                } else {
                                    failedCount++;
                                    Serial.println("WARNING: send failed.");
                                }
                                
                                // If there has been no publish failure, increment the place to start next time
                                if (failedCount == 0) {
                                    nextPubRecord = incModRecords(nextPubRecord);
                                    Serial.printf("Incremented nextPubRecord to %d.\n", nextPubRecord);
                                }
                            } else {
                                Serial.println("unused.");
                            }
                            
                            // Increment x
                            x = incModRecords(x);
                            
                        } while (x != incModRecords(currentRecord));
                        
                        Serial.printf("%d report(s) sent, %d failed to send.\n", sentCount, failedCount);
                        // If there's been a publish failure and nextPubRecord is
                        // equal to currentRecord then we must have wrapped.  In this
                        // case, increment nextPubRecord to keep things in time order.
                        if ((failedCount > 0) && (nextPubRecord == currentRecord)) {
                            nextPubRecord = incModRecords(nextPubRecord);
                        }
                        
                        lastReportSeconds = Time.now();
                    }
                
                    // Back to sleep now
                    sleepTime = Time.now();
                }
            } else {
                // We're awake outside the working day, calculate the new wake-up time
                if (sleepForSeconds < START_OF_WORKING_DAY_SECONDS) {
                    sleepForSeconds = Time.now() + START_OF_WORKING_DAY_SECONDS - secondsSinceMidnight - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS;
                } else {
                    // Must be after the end of the day, so wake-up next tomorrow morning
                    sleepForSeconds = Time.now() + START_OF_WORKING_DAY_SECONDS + (3600 * 24) - secondsSinceMidnight - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS;
                }
                if (sleepForSeconds > 0) {
                    Serial.printf("Awake outside the working day (time now %02d:%02d:%02d UTC, working day is %02d:%02d:%02d to %02d:%02d:%02d), going back to sleep for %d seconds in order to wake up at %s.\n",
                                  Time.hour(), Time.minute(), Time.second(),
                                  Time.hour(START_OF_WORKING_DAY_SECONDS), Time.minute(START_OF_WORKING_DAY_SECONDS), Time.second(START_OF_WORKING_DAY_SECONDS),
                                  Time.hour(START_OF_WORKING_DAY_SECONDS + LENGTH_OF_WORKING_DAY_SECONDS), Time.minute(START_OF_WORKING_DAY_SECONDS + LENGTH_OF_WORKING_DAY_SECONDS),
                                  Time.second(START_OF_WORKING_DAY_SECONDS + LENGTH_OF_WORKING_DAY_SECONDS),
                                  sleepForSeconds + WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS, Time.timeStr(Time.now() + sleepForSeconds + WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS).c_str());
                } else {
                    sleepForSeconds = 0;
                }
            }
        } else {
            // We're awake when we shouldn't have started operation yet, calculate new wake-up time
            sleepForSeconds = START_TIME_UNIX_UTC - Time.now() - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS;
            if (sleepForSeconds > 0) {
                Serial.printf("Awake too early (time now %s UTC, expected start time %s UTC), going back to sleep for %d second(s) in order to wake up at %s.\n",
                              Time.timeStr().c_str(), Time.timeStr(START_TIME_UNIX_UTC).c_str(), sleepForSeconds + WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS,
                              Time.timeStr(Time.now() + sleepForSeconds + WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS).c_str());
            } else {
                sleepForSeconds = 0;
            }
        }
    } else {
        Serial.printf("WARNING: unable to establish time (time now is %d).\n", Time.now());
        sleepForSeconds = TIME_SYNC_RETRY_PERIOD_SECONDS - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS;
    }

    // Print some stats
    uint32_t x = millis() / 1000 + totalSleepSeconds;
    if (x == 0) {
        x = 1; // avoid div by 0
    }
    Serial.printf("Stats: up-time %d %d:%02d:%02d (since %s), of which %d%% was in power-save mode.\n",
                  x / 86400, (x / 3600) % 24, (x / 60) % 60,  x % 60, Time.timeStr(Time.now() - x).c_str(), totalSleepSeconds * 100 / x);
    Serial.printf("       GPS has been on for ~%d%% of the up-time.\n", totalGpsSeconds * 100 / x);
    Serial.printf("       looped %d time(s), detected motion on %d loop(s) and obtained a GPS fix on %d (%d%%) of those loop(s).\n",
                  numLoops, numLoopsGpsRunning, numLoopsGpsFix, (numLoopsGpsFix != 0) ? (numLoopsGpsRunning * 100 / numLoopsGpsFix) : 0);
    Serial.printf("       last accelerometer reading: x = %d, y = %d, z = %d.\n", accelerometerReading.x, accelerometerReading.y, accelerometerReading.z);
    Serial.printf("Loop %d: now sleeping for %d second(s) (will awake at %s UTC).\n",
                  numLoops, sleepForSeconds + WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS,
                  Time.timeStr(Time.now() + sleepForSeconds + WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS).c_str());

    // Make sure the debug LED is off to save power
    debugInd(DEBUG_IND_OFF);
    
    // Leave a little time for serial prints to leave the building before sleepy-byes
    delay (500);
#ifdef DISABLE_WAKEUP_TIMES
    inTheWorkingDay = true;
#endif
    time_t downTime = Time.now();
    // Now go to sleep for the allotted time.  If the accelerometer interrupts goes
    // off it will be flagged and dealt with when we're good and ready, hence we sleep
    // listening to a pin that is an output and always low.
    // NOTE: I originally had the "inTheWorkingDay" code waking up on intPin. However,
    // if you do that it replaces the interrupt you've attached earlier and, even though
    // you can reattach your interrupt afterwards there's always the chance that the
    // interrupt goes off betweeen detachment and reattachment in which case we'll
    // miss it
    if (inTheWorkingDay) {
        // If we're in the working day, sleep with the network connection up so
        // that we can send reports.
        System.sleep(D3, RISING, sleepForSeconds, SLEEP_NETWORK_STANDBY);
   } else {
        // Nice deep sleep otherwise.
        System.sleep(D3, RISING, sleepForSeconds);
    }
    totalSleepSeconds += Time.now() - downTime;

    // It is no longer our first time
    firstTime = false;
}