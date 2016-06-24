#include "accelerometer.h"
#include "TinyGPS.h"

/* Tracker
 *
 * This code is for a tracker, borrowing from AssetTracker
 * but using a u-blox GPS module and all through-hole components
 * so that it can be hand assembled (or built using a breadboard).
 * It uses a Particle Electron board (though it is also
 * pin-compatible with a Partilce Photon board since the lower
 * pins are not used), a u-blox PAM7Q GPS module and an ADXL345
 * accelerometer break-out board to establish the GPS position
 * in a power-efficient way.  It reports data to the Particle
 * server and, from there, via webhooks, to anyone who is
 * interested.
 *
 * There are some other differences to AssetTracker:
 *
 * - It is expected that GPS position is sent quite frequently
 *   and so provision is made for use of multiple batteries.
 *   NOTE: BE VERY CAREFUL with this.  There are specific
 *   things you need to do for this to be safe, see the note
 *   in bom.xls.
 * - Processor sleep (sometimes with the modem on, sometimes
 *   with it off) is used to save* power between GPS fixes.
 *
 * In order to use power most efficiently, sleep is used in
 * combination with timed operation in the following way:
 *
 * - If, after establishing network time, the time is found to be
 *   less than START_TIME_UNIX_UTC then, the device returns to
 *   deep sleep (modem off, processor clocks and RAM off,
 *   ~0.1 mA consumed) until START_TIME_UNIX_UTC is reached.
 * - If the time is greater than or equal to START_TIME_UNIX_UTC then
 *   the device checks if the working day has begun, i.e. is the time
 *   greater than START_OF_WORKING_DAY_SECONDS and less than
 *   START_OF_WORKING_DAY_SECONDS + LENGTH_OF_WORKING_DAY_SECONDS.
 * - If the time is within the work day then one of two things happens:
 *   - If the time is less than START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC
 *     then the device wakes up only for long enough to get a GPS reading, 
 *     "slow operation" (timing out at SLOW_OPERATION_MAX_TIME_TO_GPS_FIX_SECONDS)
 *     and repeats this SLOW_OPERATION_NUM_WAKEUPS_PER_WORKING_DAY times,
 *     evenly spread throughout the working day.  Between wake-ups the
 *     device returns to deep sleep with the modem off.
 *   - If the time is greater than or equal to
 *     START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC then the device remains
 *     awake for all of the working day, putting the processor to sleep but keeping
 *     the modem registered with the network, so that we can collect and report
 *     position for the full working day.
 * - If the time is not within the working day then the device returns to
 *   deep sleep, with modem off, until the start of the next working day.
 *
 * The default timings are set so that the device wakes-up in slow operation and
 * then goes into full working day operation a few days later. This was for a show
 * where the trackers were installed a few days before the start of the show and
 * we wanted to monitor that they were OK but not consume a lot of battery power.
 * By judicious choice of:
 *
 *  - START_TIME_UNIX_UTC
 *  - START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC
 *  - START_OF_WORKING_DAY_SECONDS
 *  - LENGTH_OF_WORKING_DAY_SECONDS
 *  - SLOW_OPERATION_NUM_WAKEUPS_PER_WORKING_DAY
 *
 * ... you should be able to achieve the tracker behaviour you want.  To always
 * get slow operation, set START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC to a
 * time far in the future.  To always get full monitoring throughout the working
 * day, set START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC to START_TIME_UNIX_UTC.
 * To set the working day to be 24 hours, set START_OF_WORKING_DAY_SECONDS to 0 and
 * LENGTH_OF_WORKING_DAY_SECONDS to 3600.
 *
 * MESSAGES
 *
 * The chosen message formats are hilghly compressed to
 * save data and are of the following form:
 *
 * gps: 353816058851462;51.283645;-0.776569;1465283731;1;5.15
 *
 * ...where the first item is the 15-digit IMEI of the device, the
 * second one the latitude in degrees as a float, the third one the
 * longitude in degrees as a float, then there's the timestamp
 * in Unix time (UTC) and finally the last two, inserted at the end
 * to preserve backwards-compatibility with previous implementations of this
 * messaging system; these are a 1 if the reading was triggered due to the
 * accelerometer indicating motion (0 otherwise) and then the horizontal
 * dilution of position (i.e. accuracy) as a float.  All fields must be present
 * aside from the HDOP.
 * This is sent every GPS_PERIOD_SECONDS while moving, or at the wake-up time
 * in slow operation.
 * 
 * telemetry: 353816058851462;80.65;-70;1465283731
 *
 * ...where the first item is the 15-digit IMEI, the second one
 * the battery left as a percentage, the third the signal strength
 * in dBm and the last one the timestamp in Unix time (UTC). All
 * fields must be present.  This is sent periodically when the device
 * wakes up from deep sleep and every TELEMETRY_PERIOD_SECONDS seconds
 * thereafter.
 *
 * NOTE: in addition to the above there is a "stats" message with
 * a similar format. This is used to send statistics purely as far
 * as the Particle web server to assist with debugging this code.
 * See queueStatsReport() for more details.
 *
 * NOTE: it you want to use a different message format, maybe
 * using a more standard (though less efficient) JSON approach,
 * simply modify queueTelemeteryReport(), queueGpsReport()
 * (and, if required, queueStatsReport()) to match.
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
 *
 * NOTE on waking up from System.sleep(): the HW for this
 * project was designed with the accelerometer interrupt output wired
 * to the WKP pin of the Electron board, on the assumption that
 * we could use that feature or disable it by using a "null"
 * (i.e. set to an output and to low) pin in the System.sleep() call.
 * D3 is used for this purpose.  However, in revision 0.5.1 of
 * the Particle software (fixed in 0.6.0 apparently), the Electron
 * board will *always* wakeup with an edge on the WKP pin
 * *whatever* pin is specified in the System.sleep() call.  Even
 * once this is fixed, the WKP pin will always wake up the system
 * from deep sleep, whatever else you do.  So, having decided not
 * to use the accelerometer interrupt, it was necessary to disable
 * interrupts from the accelerometer board itself during sleep.
 * I could, of course, disable them always and diff the X/Y/Z
 * readings but that wouldn't capture motion that occurred while
 * the Electron board was asleep. If I were having the board made
 * again I would route INT1 from the accelerometer to D4, in case
 * I needed it again, and connect the Electron's WKP pin firmly to
 * ground.
 */

/****************************************************************
 * CONFIGURATION MACROS
 ***************************************************************/

/// The maximum amount of time to hang around waiting for a
// connection to the Particle server.
#define WAIT_FOR_CONNECTION_SECONDS 60

/// How long to wait for things to sort themselves out (e.g. the
// USB port) after waking up from sleep.
#define WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS 5

/// How long to wait for the Particle server to respond to
// a time sync request
#define TIME_SYNC_WAIT_SECONDS 10

/// If the system cannot establish time by talking to the 
// Particle server, wait this long and try again.
// NOTE: must be bigger than WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS.
#define TIME_SYNC_RETRY_PERIOD_SECONDS 30

/// The time between GPS records  when moving.  This must always
// be the minimum period because it is also used as the sleep
// period.
// NOTE: must be bigger than WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS.
#define GPS_PERIOD_SECONDS 30

/// The wake-up period in seconds.  This should be equal to the
// shortest wake-up period, which is usually the GPS one.
#define WAKEUP_PERIOD_SECONDS GPS_PERIOD_SECONDS

/// The periodicity of telemetry reports.
#define TELEMETRY_PERIOD_SECONDS 600

/// The periodicity of stats reports.
#define STATS_PERIOD_SECONDS WAKEUP_PERIOD_SECONDS

/// The report period in seconds.  At this interval the
// queued-up records are sent.
#define REPORT_PERIOD_SECONDS 120

/// The size of a record.
#define LEN_RECORD 120

/// The queue length at which to try sending a record.
#define QUEUE_SEND_LEN 4

/// The time allowed for GPS to sync each time, once
// it has initially synchronised.
#define GPS_FIX_TIME_SECONDS 10

/// The minimum possible value of Time (in Unix, UTC), used
// to check the sanity of our RTC time.
#define MIN_TIME_UNIX_UTC 1451606400 // 1 Jan 2016 @ midnight

/// The start time for the device (in Unix, UTC).
// If the device wakes up before this time it will return
// to deep sleep.
// Use http://www.onlineconversion.com/unix_time.htm to work this out.
//#define START_TIME_UNIX_UTC 1467615600 // 4th July 2016 @ 07:00 UTC
#define START_TIME_UNIX_UTC 1466586000 // 22 June 2016 @ 09:00 UTC

/// The number of times to wake-up during the working day when in slow operation.
#define SLOW_OPERATION_NUM_WAKEUPS_PER_WORKING_DAY 1

/// The maximum amount of time to wait for a GPS fix to be established while we are in
// slow operation.  After this time, or as soon as a GPS fix has been established
// and transmitted, we can go to deep sleep.
#define SLOW_OPERATION_MAX_TIME_TO_GPS_FIX_SECONDS (60 * 5)

/// The start time for full working day operation (in Unix, UTC).
// After this time the device will be awake for the whole working day and send reports
// as necessary.
// This time must be later than or equal to START_TIME_UNIX_UTC.
// Use http://www.onlineconversion.com/unix_time.htm to work this out.
//#define START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC 1468134000 // 10 July 2016 @ 07:00 UTC
#define START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC 1466586000 // 22 June 2016 @ 09:00 UTC

/// Start of day in seconds after midnight UTC.
#define START_OF_WORKING_DAY_SECONDS (3600 * 7) // 07:00 UTC, so 08:00 BST

/// Duration of a working day in seconds.
#define LENGTH_OF_WORKING_DAY_SECONDS (3600 * 10) // 10 hours, so day ends at 17:00 UTC (18:00 BST)

/// The accelerometer activity threshold (in units of 62.5 mg).
#define ACCELEROMETER_ACTIVITY_THRESHOLD 10

/****************************************************************
 * CONDITIONAL COMPILATION OPTIONS
 ***************************************************************/

/// Enable stats reporting.
#define ENABLE_STATS_REPORTING

/// Define this to do GPS readings irrespective of the state of the
// accelerometer.
//#define DISABLE_ACCELEROMETER

/// Define this to record GPS values even if the values are not valid.
// The invalid value is TinyGPS::GPS_INVALID_F_ANGLE, which comes
// out as 1000 degrees.
//#define IGNORE_INVALID_GPS


/****************************************************************
 * OTHER MACROS
 ***************************************************************/

/// A magic string to indicate that retained RAM has been initialised.
#define RETAINED_INITIALISED "RetInit"

/// The length of the IMEI, used as the device ID.
#define IMEI_LENGTH 15

/// Work out the number of seconds between each wake-up in slow mode.
#define SLOW_MODE_INTERVAL_SECONDS (LENGTH_OF_WORKING_DAY_SECONDS / (SLOW_OPERATION_NUM_WAKEUPS_PER_WORKING_DAY + 1))

/****************************************************************
 * TYPES
 ***************************************************************/

/// The types of fatal error that can occur.
typedef enum {
    FATAL_TYPE_NULL,
    FATAL_RECORDS_OVERRUN_1,
    FATAL_RECORDS_OVERRUN_2,
    FATAL_RECORDS_OVERRUN_3,
    MAX_NUM_FATAL_TYPES
} FatalType_t;

/// Struct to hold an accelerometer reading.
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} AccelerometerReading_t;

/// The possible record types.
typedef enum {
    RECORD_TYPE_NULL,
    RECORD_TYPE_TELEMETRY,
    RECORD_TYPE_GPS,
    RECORD_TYPE_STATS,
    MAX_NUM_RECORD_TYPES
} RecordType_t;

/// A single record.
typedef struct {
    bool isUsed;
    RecordType_t type;
    char contents[LEN_RECORD];
} Record_t;

/// For debugging: LED flash types.
typedef enum {
    DEBUG_IND_OFF,
    DEBUG_IND_TOGGLE,
    DEBUG_IND_GPS_FIX,
    DEBUG_IND_ACTIVITY,
    DEBUG_IND_RETAINED_RESET,
    MAX_NUM_DEBUG_INDS
} DebugInd_t;

/// The stuff that is stored in retained RAM.
// NOTE: the variables that follow are retained so that
// we can go to deep sleep and still keep a record of
// what happened from initial power-on.  They are kept
// in one structure so that they can all be reset with
// a memset().
typedef struct {
    // Something to use as a key so that we know whether 
    // retained memory has been initialised or not
    char key[sizeof(RETAINED_INITIALISED)];
    // This array was previously used and is kept here
    // in order that retained memory on existing devices
    // in the field is not corrupted by its removal
    char unused1[9];
    // The number of times setup() has run in a working day
    uint32_t numSetupsCompletedToday;
    // The time we went to sleep
    time_t sleepTime;
    // The time we actually went down to lost power state
    time_t powerSaveTime;
    // Count the number of times around the loop, for info
    uint32_t numLoops;
    // Count the number of loops on which motion was detected, for info
    uint32_t numLoopsMotionDetected;
    // Count the number of loops where position  was needed, for info
    uint32_t numLoopsLocationNeeded;
    // Count the number of loops for which a GPS fix was attempted, for info
    uint32_t numLoopsGpsOn;
    // Count the number of loops for which a GPS fix was achieved, for info
    uint32_t numLoopsGpsFix;
    // Count the number of loops for which we reported a valid location
    // (which is different to numLoopsGpsFix as we may not need to get a fix
    // if we haven't moved)
    uint32_t numLoopsLocationValid;
    // Count the number of seconds we've been in power saving state for
    uint32_t totalPowerSaveSeconds;
    // Count the number of seconds GPS has been on for
    time_t gpsPowerOnTime;
    uint32_t totalGpsSeconds;
    // Count the number of publish attempts
    uint32_t numPublishAttempts;
    // Count the number of publishes that failed
    uint32_t numPublishFailed;
    // Count the number of connect attempts
    uint32_t numConnectAttempts;
    // Count the number of connect failures
    uint32_t numConnectFailed;
    // Count the number of entries into Setup()
    uint32_t numStarts;
    // Hold the last accelerometer reading
    AccelerometerReading_t accelerometerReading;
    // Storage for fatals
    uint32_t numFatals;
    FatalType_t fatalList[20];
    // Like unused1[] only different
    char unused2[9];
} Retained_t;

/****************************************************************
 * GLOBAL VARIABLES
 ***************************************************************/

/// Record type strings.
// NOTE: must match the RecordType_t enum above.
const char * recordTypeString[] = {"null", "telemetry", "gps", "stats"};

/// The IMEI of the module.
char imei[IMEI_LENGTH] = "";

/// Whether we have an accelerometer or not
bool accelerometerConnected = false;

/// When we last tried to get a fix.
time_t lastGpsSeconds = 0;

/// Time Of the last ping message.
time_t lastPingSeconds = 0;

/// Time Of the last telemetry message.
time_t lastTelemetrySeconds = 0;

/// Time of the last stats report.
time_t lastStatsSeconds = 0;

/// Time Of the last report.
time_t lastReportSeconds = 0;

/// The time at which setup was finished.
time_t setupCompleteSeconds = 0;

/// The last lat/long reading and accuracy.
float lastLatitude = TinyGPS::GPS_INVALID_F_ANGLE;
float lastLongitude = TinyGPS::GPS_INVALID_F_ANGLE;
uint32_t lastHdop = TinyGPS::GPS_INVALID_HDOP;

/// The records accumulated.
Record_t records[100];

/// The current record.
uint32_t currentRecord = 0;

/// The next record to send.
uint32_t nextPubRecord = currentRecord;

/// The number of records queued.
uint32_t numRecordsQueued = 0;

/// Track if this is our first time through.
bool firstTime = true;

/// All the retained variables.
retained Retained_t r;

/// Instantiate GPS.
TinyGPS gps;

/// Instantiate the accelerometer.
Accelerometer accelerometer = Accelerometer();

/// Instantiate a fuel gauge.
FuelGauge fuel;

/// Only connect when it is required.
SYSTEM_MODE(SEMI_AUTOMATIC);

/// Enable retained memory.
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

/****************************************************************
 * DEBUG
 ***************************************************************/

/// Macros to invoke asserts/fatals.
#define FATAL(fatalType) {if (r.numFatals < (sizeof (r.fatalList) / sizeof (r.fatalList[0]))) {r.fatalList[r.numFatals] = fatalType;} r.numFatals++; System.reset();}
#define ASSERT(condition, fatalType) {if (!(condition)) {FATAL(fatalType)}}

/****************************************************************
 * FUNCTION PROTOTYPES
 ***************************************************************/

static void resetRetained();
static bool handleInterrupt();
static void debugInd(DebugInd_t debugInd);
static int getImeiCallBack(int type, const char* pBuf, int len, char* imei);
static time_t gpsOn();
static void gpsOff();
static bool gpsUpdate(time_t onTimeSeconds, float *pLatitude, float *pLongitude, uint32_t *pHdop);
static void updateTotalGpsSeconds();
static bool connect();
static bool establishTime();
static char * getRecord(RecordType_t type);
static void freeRecord(Record_t *pRecord);
static uint32_t incModRecords (uint32_t x);
static void queueTelemetryReport();
static void queueGpsReport(float latitude, float longitude, bool motion, uint32_t hdop);
static void queueStatsReport();
static bool sendQueuedReports();
static uint32_t secondsToWorkingDayStart(uint32_t secondsToday);

/****************************************************************
 * STATIC FUNCTIONS
 ***************************************************************/

/// Reset the retained variables.
static void resetRetained() {
    memset (&r, 0, sizeof (r));
    strcpy (r.key, RETAINED_INITIALISED);
    debugInd(DEBUG_IND_RETAINED_RESET);
}

/// Handle the accelerometer interrupt, returning true if activity
// was detected, otherwise false.
// NOTE: though we set-up interrupts and read the interrupt registers
// on the ADXL345 chip, and the ADXL345 INT1 line is wired to the
// WKP pin on the Electron board, we don't use the interrupt to wake
// us up.  Instead we wake-up at a regular interval and then check
// with this function whether the interrupt went off or not.  This
// stops us waking up all the time and allows us to set a nice
// low threshold for activity, so that we can be sure we wake-up, but
// still send GPS readings at sensible intervals.
static bool handleInterrupt() {
    bool activityDetected = false;
    
    if (accelerometerConnected) {
        Accelerometer::EventsBitmap_t eventsBitmap;
        // Disable interrupts
        noInterrupts();
        accelerometer.read (&r.accelerometerReading.x, &r.accelerometerReading.y, &r.accelerometerReading.z);
        eventsBitmap = accelerometer.handleInterrupt();
        // Re-enable interrupts
        interrupts();
        
        // Flag if we're in motion
        if (eventsBitmap & Accelerometer::EVENT_ACTIVITY) {
            activityDetected = true;
            debugInd(DEBUG_IND_ACTIVITY);
        }
    }
    
    return activityDetected;
}

/// Parse the IMEI from the module.
static int getImeiCallBack(int type, const char* pBuf, int len, char* imei) {
    
    if ((type == TYPE_UNKNOWN) && imei) {
        if (sscanf(pBuf, "\r\n%15s\r\n", imei) == 1) {
            Serial.printf("IMEI is: %.*s.\n", IMEI_LENGTH, imei);
        }
    }

    return WAIT;
}

/// Use the LED on the board (on D7) for debug indications.
static void debugInd(DebugInd_t debugInd) {
    
    switch (debugInd) {
        case DEBUG_IND_OFF:
            digitalWrite(D7, LOW);
        break;
        case DEBUG_IND_RETAINED_RESET:
            // One long flash
            digitalWrite(D7, LOW);
            delay (25);
            digitalWrite(D7, HIGH);
            delay (500);
            digitalWrite(D7, LOW);
            delay (25);
        break;
        case DEBUG_IND_TOGGLE:
            // This one is the lowest-cost debug option
            digitalWrite(D7, !(digitalRead(D7)));
        break;
        case DEBUG_IND_GPS_FIX:
            // Two flashes
            digitalWrite(D7, LOW);
            delay (25);
            digitalWrite(D7, HIGH);
            delay (25);
            digitalWrite(D7, LOW);
            delay (25);
            digitalWrite(D7, HIGH);
            delay (25);
            digitalWrite(D7, LOW);
            delay (25);
        break;
        case DEBUG_IND_ACTIVITY:
            // One flash
            digitalWrite(D7, LOW);
            delay (25);
            digitalWrite(D7, HIGH);
            delay (25);
            digitalWrite(D7, LOW);
            delay (25);
        break;
        default:
        break;
    }
}

/// Switch GPS on, returning the time that
// GPS was switched on.
static time_t gpsOn() {
    if (digitalRead(D2)) {
        // Log the time that GPS was swithed on
        r.gpsPowerOnTime = Time.now();
    }
    digitalWrite(D2, LOW);

    return Time.now();
}

/// Switch GPS off.
static void gpsOff() {
    if (!digitalRead(D2)) {
        // Record the duration that GPS was on for
        updateTotalGpsSeconds();
    }
    digitalWrite(D2, HIGH);
}

/// Update the GPS, making sure it's been on for
// enough time to give us a fix. If onTimeSeconds
// is non-zero then it represents the time at which GPS
// was already switched on.  pLatitude, pLongitude and pHdop are
// filled in with fix values and their accuracy if a fix is
// achieved (and true is returned), otherwise they are left alone.
static bool gpsUpdate(time_t onTimeSeconds, float *pLatitude, float *pLongitude, uint32_t *pHdop) {
    bool fixAchieved = false;
    uint32_t startTimeSeconds = Time.now();
    float latitude;
    float longitude;
    uint32_t hdopValue;

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
                    hdopValue =  gps.hdop();
                    if (pHdop) {
                        *pHdop = hdopValue;
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
        Serial.printf("Fix achieved in %d second(s): latitude: %.6f, longitude: %.6f",
                      Time.now() - startTimeSeconds, latitude, longitude);
        if (hdopValue != TinyGPS::GPS_INVALID_HDOP) {
            Serial.printf(", HDOP: %.2f.\n", ((float) hdopValue) / 100);
        } else {
            Serial.printf(", no HDOP.\n");
        }
    } else {
        Serial.printf("No fix this time.\n");
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

/// Keep track of GPS up-time for info.
static void updateTotalGpsSeconds() {
    uint32_t x = Time.now() - r.gpsPowerOnTime;
    
    // Ignore silly values which could occur if
    // the timebase underneath us is updated between
    // gpsPowerOnTime and now
    if (x < 31536000) { // 1 year
        r.totalGpsSeconds += x;
    }

    r.gpsPowerOnTime = Time.now();
}

/// Connect to the network, returning true if successful.
static bool connect() {
    bool success = false;
    
    if (Particle.connected()) {
        success = true;
    } else {
        r.numConnectAttempts++;
        Serial.printf("Connecting to network (waiting for up to %d second(s))... ", WAIT_FOR_CONNECTION_SECONDS);
        Particle.connect();
        if (waitFor(Particle.connected, WAIT_FOR_CONNECTION_SECONDS)) {
            success = true;
            Serial.println("Connected.");
        } else {
            r.numConnectFailed++;
            Serial.println("WARNING: connection failed.");
        }
    }
    
    return success;
}

/// Make sure we have time sync.
static bool establishTime() {
    bool success = false;

    if (Time.now() < MIN_TIME_UNIX_UTC) {
        Serial.printf("Syncing time (this will take %d second(s)).\n", TIME_SYNC_WAIT_SECONDS);
        connect();
        Particle.syncTime();
        // The above is an asynchronous call and so we've no alternative, if we want
        // to be sure that time is correct, then to hang around for a bit and see
        delay(TIME_SYNC_WAIT_SECONDS * 1000);
    }
    
    if (Time.now() >= MIN_TIME_UNIX_UTC) {
        success = true;
        if (setupCompleteSeconds < MIN_TIME_UNIX_UTC) {
            // If we didn't already have time established when we left setup(),
            // reset it to a more correct time here.
            setupCompleteSeconds = Time.now();
        }
    } else {
        Serial.printf("WARNING: unable to establish time (time now is %d).\n", Time.now());
    }

    return success;
}

/// Get a new record from the list.
static char * getRecord(RecordType_t type) {
    char * pContents;

    Serial.printf("Using record %d.\n", currentRecord);
    
    ASSERT (currentRecord < (sizeof (records) / sizeof (records[0])), FATAL_RECORDS_OVERRUN_1);
    
    if (records[currentRecord].isUsed) {
        Serial.println("WARNING: records queue has wrapped, over-writing old data.");
    } else {
        numRecordsQueued++;
    }
    
    ASSERT (numRecordsQueued < (sizeof (records) / sizeof (records[0])), FATAL_RECORDS_OVERRUN_2);
    
    records[currentRecord].isUsed = true;
    records[currentRecord].type = type;
    pContents = records[currentRecord].contents;
    
    // Move the index on to the next record
    currentRecord = incModRecords(currentRecord);
    Serial.printf("Incremented currentRecord to %d.\n", currentRecord);
    
    return pContents;
}

/// Free a record.
static void freeRecord(Record_t *pRecord) {
    if (pRecord) {
        pRecord->isUsed = false;
        if (numRecordsQueued > 0) {
            numRecordsQueued--;
        }
    }
}

/// Increment a number and return the incremented
// number modulo the number of records available.
static uint32_t incModRecords (uint32_t x) {
    x++;
    if (x >= (sizeof (records) / sizeof (records[0]))) {
        x = 0;
    }
    
    return x;
}

/// Queue a telemetry report.
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
    
    Serial.printf ("%d byte(s) of record used (%d byte(s) unused).\n", contentsIndex + 1, LEN_RECORD - (contentsIndex + 1)); // +1 to account for terminator
}

/// Queue a GPS report.
static void queueGpsReport(float latitude, float longitude, bool motion, uint32_t hdop) {
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
    
    // Add motion to the end, to preserve backwards-compatibility
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%d", motion);  // -1 for terminator
    } else {
        Serial.println("WARNING: couldn't fit motion indication into report.");
    }
        
    // Add HDOP to the end, to preserve backwards-compatibility
    if (hdop != TinyGPS::GPS_INVALID_HDOP) {
        if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
            contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%.2f", ((float) hdop) / 100);  // -1 for terminator
        } else {
            Serial.println("WARNING: couldn't fit HDOP into report.");
        }
    }

    Serial.printf ("%d byte(s) of record used (%d byte(s) unused).\n", contentsIndex + 1, LEN_RECORD - (contentsIndex + 1)); // +1 to account for terminator
}

/// Queue a stats report.
static void queueStatsReport() {
    char *pRecord;
    uint32_t contentsIndex = 0;
    uint32_t upTimeSeconds = millis() / 1000 + r.totalPowerSaveSeconds;
    
    if (upTimeSeconds == 0) {
        upTimeSeconds = 1; // avoid div by 0
    }

    Serial.println("Queuing a stats report.");
    
    // Start a new record
    pRecord = getRecord(RECORD_TYPE_STATS);

    // Add the device ID
    contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, "%.*s", IMEI_LENGTH, imei);  // -1 for terminator

    // Add fatal count and types
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";F%ld", r.numFatals); // -1 for terminator
        for (uint8_t x = 0; (x < r.numFatals) && (x < (sizeof (r.fatalList) / sizeof (r.fatalList[0]))); x++) {
            contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ".%02d", r.fatalList[x]); // -1 for terminator
        }
    }
    if ((contentsIndex < 0) || (contentsIndex >= LEN_RECORD)) {
        Serial.println("WARNING: couldn't fit fatal count and types into report.");
    }
    
    // Add up-time
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%ld.%ld:%02ld:%02ld",
                                   upTimeSeconds / 86400, (upTimeSeconds / 3600) % 24, (upTimeSeconds / 60) % 60,  upTimeSeconds % 60);  // -1 for terminator
    } else {
        Serial.println("WARNING: couldn't fit up-time into report.");
    }
    
    // Add %age power save time
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%ld%%", r.totalPowerSaveSeconds * 100 / upTimeSeconds);  // -1 for terminator
    } else {
        Serial.println("WARNING: couldn't fit percentage power saving time into report.");
    }
    
    // Add %age GPS on time
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";~%ld%%", r.totalGpsSeconds * 100 / upTimeSeconds);  // -1 for terminator
    } else {
        Serial.println("WARNING: couldn't fit percentage GPS on time into report.");
    }
    
    // Add loop counts and position percentage
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";L%ldM%ldG%ldP%ld%%",
                                   r.numLoops, r.numLoopsMotionDetected, r.numLoopsGpsOn, (r.numLoopsLocationNeeded != 0) ? (r.numLoopsLocationValid * 100 / r.numLoopsLocationNeeded) : 0); // -1 for terminator
    } else {
        Serial.println("WARNING: couldn't fit loop counts into report.");
    }
    
    // Add connect counts
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";C%ld-%ld", r.numConnectAttempts, r.numConnectFailed); // -1 for terminator
    } else {
        Serial.println("WARNING: couldn't fit connect counts into report.");
    }
    
    // Add publish (== send) counts
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";S%ld-%ld", r.numPublishAttempts, r.numPublishFailed); // -1 for terminator
    } else {
        Serial.println("WARNING: couldn't fit publish counts into report.");
    }

    // Add last accelerometer reading
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";X%dY%dZ%d",
                                   r.accelerometerReading.x, r.accelerometerReading.y, r.accelerometerReading.z); // -1 for terminator
    } else {
        Serial.println("WARNING: couldn't fit last accelerometer reading into report.");
    }
    
    // Add Unix time
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%u", (unsigned int) (uint32_t) Time.now());  // -1 for terminator
        Serial.printf("Time now is %s UTC.\n", Time.timeStr().c_str());
    } else {
        Serial.println("WARNING: couldn't fit timestamp into report.");
    }
    
    Serial.printf ("%d byte(s) of record used (%d byte(s) unused).\n", contentsIndex + 1, LEN_RECORD - (contentsIndex + 1)); // +1 to account for terminator
}

/// Send the queued reports, returning true if at least one
// GPS report was sent, otherwise false
static bool sendQueuedReports() {
    bool atLeastOneGpsReportSent = false;
    uint32_t sentCount = 0;
    uint32_t failedCount = 0;
    uint32_t x;
    bool sendStatsReport = false;

#ifdef ENABLE_STATS_REPORTING
    sendStatsReport = true;
#endif

    // Publish any records that are marked as used
    x = nextPubRecord;
    Serial.printf("Sending report(s) (numRecordsQueued %d, nextPubRecord %d, currentRecord %d).\n", numRecordsQueued, x, currentRecord);

    do {
        ASSERT (x < (sizeof (records) / sizeof (records[0])), FATAL_RECORDS_OVERRUN_3);
        Serial.printf("Report %d: ", x);
        if (records[x].isUsed) {
            // If this is a stats record and we're not reporting stats over the air
            // then just print this record and then mark it as unused
            if ((records[x].type == RECORD_TYPE_STATS) && !sendStatsReport) {
                Serial.printf("[Stats: %s]\n", records[x].contents);
                freeRecord(&(records[x]));
            } else {
                // This is something we want to publish, so first connect
                if (connect()) {
                    r.numPublishAttempts++;
                    if (Particle.publish(recordTypeString[records[x].type], records[x].contents, 60, PRIVATE)) {
                        Serial.printf("sent %s.\n", records[x].contents);
                        // Keep track if we've sent a GPS report as, if we're in pre-operation mode,
                        // we can then go to sleep for longer.
                        if (records[x].type == RECORD_TYPE_GPS) {
                            atLeastOneGpsReportSent = true;
                        }
                        freeRecord((&records[x]));
                        sentCount++;
                        // In the Particle code only four publish operations can be performed per second,
                        // so put in a 1 second delay every four
                        if (sentCount % 4 == 0) {
                            delay (1000);
                        }
                    } else {
                        r.numPublishFailed++;
                        failedCount++;
                        Serial.println("WARNING: send failed.");
                    }
                } else {
                    failedCount++;
                    Serial.println("WARNING: send failed due to not being connected.");
                }
            }
            // If there has not yet been a publish failure, increment the place to start next time
            if (failedCount == 0) {
                nextPubRecord = incModRecords(nextPubRecord);
                Serial.printf("Incremented nextPubRecord to %d.\n", nextPubRecord);
            }
        } else {
            Serial.println("unused.");
        }

        // Increment x
        x = incModRecords(x);

    } while (x != incModRecords(currentRecord));  // Loop until we've gone beyond the current record

    Serial.printf("%ld report(s) sent, %ld failed to send.\n", sentCount, failedCount);
    // If there's been a publish failure and nextPubRecord is
    // equal to currentRecord then we must have wrapped.  In this
    // case, increment nextPubRecord to keep things in time order.
    if ((failedCount > 0) && (nextPubRecord == currentRecord)) {
        nextPubRecord = incModRecords(nextPubRecord);
    }

    return atLeastOneGpsReportSent;
}

/// Calculate the number of seconds to the start of the working day.
static uint32_t secondsToWorkingDayStart(uint32_t secondsToday) {
    uint32_t seconds = 0;
    
    // We're awake outside the working day, calculate the new wake-up time
    if (secondsToday < START_OF_WORKING_DAY_SECONDS) {
        seconds = START_OF_WORKING_DAY_SECONDS - secondsToday - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS;
    } else {
        if (secondsToday > START_OF_WORKING_DAY_SECONDS + LENGTH_OF_WORKING_DAY_SECONDS) {
            // After the end of the day, so wake up next tomorrow morning
            seconds = START_OF_WORKING_DAY_SECONDS + (3600 * 24) - secondsToday - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS;
        }
    }
    
    if (seconds > 0) {
        Serial.printf("Awake outside the working day (time now %02d:%02d:%02d UTC, working day is %02d:%02d:%02d to %02d:%02d:%02d), going back to sleep for %d second(s) in order to wake up at %s.\n",
                      Time.hour(), Time.minute(), Time.second(),
                      Time.hour(START_OF_WORKING_DAY_SECONDS), Time.minute(START_OF_WORKING_DAY_SECONDS), Time.second(START_OF_WORKING_DAY_SECONDS),
                      Time.hour(START_OF_WORKING_DAY_SECONDS + LENGTH_OF_WORKING_DAY_SECONDS), Time.minute(START_OF_WORKING_DAY_SECONDS + LENGTH_OF_WORKING_DAY_SECONDS),
                      Time.second(START_OF_WORKING_DAY_SECONDS + LENGTH_OF_WORKING_DAY_SECONDS),
                      seconds + WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS, Time.timeStr(Time.now() + seconds + WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS).c_str());
        // If we will still be in slow mode when we awake, no need to wake up until the first slow-operation
        // wake-up time
        if (Time.now() + seconds < START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS) {
            seconds += SLOW_MODE_INTERVAL_SECONDS;
            Serial.printf("Adding %d second(s) to this as we're in slow operation mode, so will actually wake up at %s.\n",
                           SLOW_MODE_INTERVAL_SECONDS, Time.timeStr(Time.now() + seconds + WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS).c_str());
        }
    
    } else {
        seconds = 0;
    }
    
    return seconds;
}

/****************************************************************
 * GLOBAL FUNCTIONS
 ***************************************************************/

void setup() {
    // Set up retained memory, if required
    if (strcmp (r.key, RETAINED_INITIALISED) != 0) {
        resetRetained();
    }
    
    // Starting again
    r.numStarts++;
    
    // Set time to zero when we start so that we know if time
    // is synchronised or not later on
    Time.setTime(0);
    
    // Open up a Serial port to listen over USB
    Serial.begin(9600);
    
    // Set D7 to be an output (for the debug LED on the
    // Particle board) and switch it off
    pinMode(D7, OUTPUT);
    debugInd(DEBUG_IND_OFF);

    // Set D2 to be an output (driving power to the GPS module)
    // and set it to ON to allow GPS position to be established
    // while we do everything else
    pinMode(D2, OUTPUT);
    digitalWrite (D2, HIGH);
    gpsOn();

    // Use D3 as a "null" pin that will not generate any activity at all
    pinMode(D3, OUTPUT);
    digitalWrite (D3, LOW);
    
    // After a reset of the Electron board it takes a Windows PC several seconds
    // to sort out what's happened to its USB interface, hence you need this
    // delay if you're going to capture the serial output completely
    delay (5000);
    
    // Connect to the network so as to establish time
    connect();
    establishTime();

#ifdef DISABLE_ACCELEROMETER
    accelerometerConnected = false;
#else
    // Set up all the necessary accelerometer bits
    accelerometerConnected = accelerometer.begin();
    if (accelerometerConnected) {
        accelerometer.setActivityThreshold(ACCELEROMETER_ACTIVITY_THRESHOLD);
        accelerometer.enableInterrupts();
    }
#endif

    // Start the serial port for the GPS module
    Serial1.begin(9600);

    // Set all the records used to false
    memset (records, false, sizeof (records));

    // Get the IMEI of the module
    Serial.println("Getting IMEI...");
    Cellular.command(getImeiCallBack, imei, 10000, "AT+CGSN\r\n");
    Serial.println("Started.");

    // Setup is now complete
    setupCompleteSeconds = Time.now();
}

void loop() {
    time_t sleepForSeconds = 0;
    bool inMotion;
    bool forceSend = false;
    bool modemStaysAwake = false;
    bool atLeastOneGpsReportSent = false;
    time_t gpsOnTime = 0;

   // This only for info
    r.numLoops++;
    
    // Check here if the accelerometer interrupt occurred
    inMotion = handleInterrupt();
    
    if (!firstTime) {
        // Switch GPS on early so that it can get fixing.
        if (inMotion && (Time.now() - lastGpsSeconds >= GPS_PERIOD_SECONDS)) {
            gpsOnTime = gpsOn();
        }

        // It seems counterintuitive to add a delay here but, if you don't
        // then, on wake-up from System.sleep(), one or other of the calls below
        // stalls for some time (maybe one of the serial functions) and so
        // it actually saves power to wait a little while here.
        // Also, Serial prints before this point in loop() likely won't appear.
        delay (WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS * 1000);
    }

    // Having valid time is fundamental to this program and so, if time
    // has not been established (via a connection to the Particle server),
    // try to establish it again
    if (establishTime()) {

        // Add up the time we were in power save mode (for info)
        if (r.powerSaveTime != 0) {
            r.totalPowerSaveSeconds += Time.now() - r.powerSaveTime;
            r.powerSaveTime = 0;
        }

        // Check if we should be awake at all
        if (Time.now() >= START_TIME_UNIX_UTC - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS) {
            uint32_t secondsSinceMidnight = Time.hour() * 3600 + Time.minute() * 60 + Time.second();

            // IMEI retrieval can occasionally fail so check again here
            if (imei[0] < '0') {
                Serial.println("WARNING: trying to get IMEI again...");
                Cellular.command(getImeiCallBack, imei, 10000, "AT+CGSN\r\n");
            }

            // Check if we are inside the working day
            if ((secondsSinceMidnight >= START_OF_WORKING_DAY_SECONDS) &&
                (secondsSinceMidnight <= START_OF_WORKING_DAY_SECONDS + LENGTH_OF_WORKING_DAY_SECONDS)) {

                modemStaysAwake = true;

                // Is it time to do something?
                if (firstTime || (Time.now() >= r.sleepTime + WAKEUP_PERIOD_SECONDS)) {
                    Serial.printf("\nAwake: time now is %s UTC, slept since %s UTC (%d second(s) ago).\n",
                                  Time.timeStr().c_str(), Time.timeStr(r.sleepTime).c_str(), Time.now() - r.sleepTime);

                    // Set the default sleep time after we've done stuff
                    sleepForSeconds = WAKEUP_PERIOD_SECONDS - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS;

                    // Queue a telemetry report, if required
                    if (Time.now() - lastTelemetrySeconds >= TELEMETRY_PERIOD_SECONDS) {
                        lastTelemetrySeconds = Time.now();
                        queueTelemetryReport();
                        Serial.println("Forcing a send.");
                        forceSend = true;
                    }

                    // Queue a GPS report, if required
                    if (Time.now() - lastGpsSeconds >= GPS_PERIOD_SECONDS) {
                        bool getFix = false;
                        r.numLoopsLocationNeeded++;
                        
                        // If we've moved, or don't have a valid reading, take a new reading
                        if (!accelerometerConnected) {
                            getFix = true;
                            Serial.println ("No accelerometer, getting GPS reading every time.");
                        }
                        if (inMotion) {
                            getFix = true;
                            r.numLoopsMotionDetected++;
                            Serial.println ("*** Motion was detected, getting latest GPS reading.");
                        }
                        if ((lastLatitude == TinyGPS::GPS_INVALID_F_ANGLE) || (lastLongitude == TinyGPS::GPS_INVALID_F_ANGLE)) {
                            getFix = true;
                            Serial.println ("No fix yet, getting GPS reading.");
                        }
                        
                        if (getFix) {
                            r.numLoopsGpsOn++;
                            
                            // Get the latest output from GPS
                            if (gpsUpdate(gpsOnTime, &lastLatitude, &lastLongitude, &lastHdop)) {
                                r.numLoopsGpsFix++;
                            } else {
                                lastLatitude = TinyGPS::GPS_INVALID_F_ANGLE;
                                lastLongitude = TinyGPS::GPS_INVALID_F_ANGLE;
                                lastHdop = TinyGPS::GPS_INVALID_HDOP;
                            }
                        }

                        // If we have a valid reading (either a new one if we're moving, or the
                        // previous one if we haven't moved) then queue a report
#ifdef IGNORE_INVALID_GPS
                        if (true) {
#else
                        if ((lastLatitude != TinyGPS::GPS_INVALID_F_ANGLE) && (lastLongitude != TinyGPS::GPS_INVALID_F_ANGLE)) {
#endif                            
                            r.numLoopsLocationValid++;
                            lastGpsSeconds = Time.now();
                            queueGpsReport(lastLatitude, lastLongitude, inMotion, lastHdop);
                        }
                    }

                    // Queue a stats report, if required
                    if (Time.now() - lastStatsSeconds >= STATS_PERIOD_SECONDS) {
                        lastStatsSeconds = Time.now();
                        queueStatsReport();
                    }

                    // Check if it's time to publish the queued reports
                    if (forceSend || ((Time.now() - lastReportSeconds >= REPORT_PERIOD_SECONDS) && (numRecordsQueued >= QUEUE_SEND_LEN))) {
                        if (forceSend) {
                            Serial.println ("Force Send was set.");
                        }
                        forceSend = false;

                        atLeastOneGpsReportSent = sendQueuedReports();
                        lastReportSeconds = Time.now(); // Do this at the end as transmission could, potentially, take some time
                    }

                    // If were still in slow operation and at least one GPS report has been sent, or we've
                    // timed out on getting a GPS fix during this slow operation wake-up, then we can go to
                    // deep sleep until the interval expires
                    if (Time.now() < START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC) {
                        if (atLeastOneGpsReportSent || (Time.now() - setupCompleteSeconds > SLOW_OPERATION_MAX_TIME_TO_GPS_FIX_SECONDS)) {
                            modemStaysAwake = false;
                            uint32_t numIntervalsPassed = (secondsSinceMidnight - START_OF_WORKING_DAY_SECONDS) / SLOW_MODE_INTERVAL_SECONDS;
                            sleepForSeconds = START_OF_WORKING_DAY_SECONDS + (numIntervalsPassed + 1) * SLOW_MODE_INTERVAL_SECONDS -
                                              secondsSinceMidnight - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS;
                            if (sleepForSeconds < 0) {
                                sleepForSeconds = 0;
                            }
                            // If we would be waking up after the end of the working day, have a proper sleep instead
                            if (secondsSinceMidnight + sleepForSeconds >
                                START_OF_WORKING_DAY_SECONDS + LENGTH_OF_WORKING_DAY_SECONDS - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS) {
                                sleepForSeconds = secondsToWorkingDayStart(secondsSinceMidnight);
                            }
                        }
                    }

                    // Back to sleep now
                    r.sleepTime = Time.now();
                } // if() we need to do something
            } else {
                // We're awake outside of the working day, calculate the time to the start of the working day
                sleepForSeconds = secondsToWorkingDayStart(secondsSinceMidnight);
            } // else condition of if() it's time to do something
        } else {
            // We're awake when we shouldn't have started operation at all yet, calculate new wake-up time
            sleepForSeconds = START_TIME_UNIX_UTC - Time.now() - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS;

            // If we will still be in slow mode when we awake, no need to wake up until the first interval of the working day expires
            if (Time.now() + sleepForSeconds < START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS) {
                sleepForSeconds += SLOW_MODE_INTERVAL_SECONDS;
            }

            if (sleepForSeconds > 0) {
                Serial.printf("Awake too early (time now %s UTC, expected start time %s UTC), going back to sleep for %d second(s) in order to wake up at %s.\n",
                              Time.timeStr().c_str(), Time.timeStr(START_TIME_UNIX_UTC).c_str(), sleepForSeconds + WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS,
                              Time.timeStr(Time.now() + sleepForSeconds + WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS).c_str());
            } else {
                sleepForSeconds = 0;
            }

            // Also, clear the retained variables as we'd like a fresh start when we awake
            resetRetained();
        } // else condition of if() time >= START_TIME_UNIX_UTC

        // Record the time we went into power saving state (only if we have established time though)
        r.powerSaveTime = Time.now();

    } else {
        sleepForSeconds = TIME_SYNC_RETRY_PERIOD_SECONDS - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS;
        // Keep the modem awake in this case as we really want to establish network time
        modemStaysAwake = true;
    } // else condition of if() network time has been established

    Serial.printf("Loop %ld: now sleeping for %ld second(s) (will awake at %s UTC), modem will be ",
                  r.numLoops, sleepForSeconds + WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS,
                  Time.timeStr(Time.now() + sleepForSeconds + WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS).c_str());
    if (modemStaysAwake) {
        Serial.printf("ON while sleeping.\n");
    } else {
        Serial.printf("OFF while sleeping.\n");
    }

    // Make sure the debug LED is off to save power
    debugInd(DEBUG_IND_OFF);
    
    // Leave a little time for serial prints to leave the building before sleepy-byes
    delay (500);
    
    // Now go to sleep for the allotted time.  If the accelerometer interrupt goes
    // off it will be flagged and dealt with when we're good and ready, hence we sleep
    // listening to a pin that is an output and always low.
    // NOTE: I originally had the "inTheWorkingDay" code waking up on intPin. However,
    // the System.sleep() call replaces the interrupt you've attached earlier with its
    // own interrupt handler and, even though you can reattach your interrupt afterwards,
    // there's always the chance that the interrupt goes off betweeen detachment and
    // reattachment, in which case we'll miss it.  Also, waking up all the time on intPin
    // means you would have to tune the accelerometer to go off at a sensible yet not
    // stupid level (otherwise it would waste power) and go back to sleep again if
    // some back-off interval had not been met.  Too complicated, better to keep things
    // simple; our GPS fix interval is short enough at 30 seconds in any case.
    if (sleepForSeconds > 0) {
        if (modemStaysAwake) {
            // If we're in normal working-day operation, sleep with the network connection up so
            // that we can send reports
            // NOTE: in revision 0.5.1 of the Particle software, when it says that the pin
            // specified in the System.sleep() call is the "specific interrupt" that will
            // wake up from sleep, that is incorrect.  In fact the system will also ALWAYS
            // wake-up if the WKP pin is toggled.  Hence we have to force ourselves back to
            // bed if we've woken up early.   This will be fixed in revision 0.6.0 I'm told.
            time_t t = Time.now();
            while (Time.now() < t + sleepForSeconds) {
                System.sleep(D3, RISING, sleepForSeconds, SLEEP_NETWORK_STANDBY);
            }
        } else {
            // Nice deep sleep otherwise, making sure that GPS is off so that we
            // don't use power and the accelerometer can't interrupt us.
            // NOTE: we will come back from reset after this, only the
            // retained variables will be kept
            if (accelerometerConnected) {
                accelerometer.disableInterrupts();
            }
            gpsOff();
            System.sleep(SLEEP_MODE_DEEP, sleepForSeconds);
        }
    }

    // It is no longer our first time
    firstTime = false;
}