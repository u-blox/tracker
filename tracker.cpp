#include "accelerometer.h"

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
 * If, on the other hand, you want a tracker for something that moves relatively
 * rarely, so there is no "working day", but it is important that you get a GPS report
 * the moment it moves, then you should modify the deep sleep portion of this code
 * so that instead of disabling the accelerometer interrupt during deep sleep it
 * leaves it on.  This would wake-up the system at any time, have it register with
 * the network and send a GPS fix. You would need to ensure that it doesn't
 * re-trigger often as that would waste more battery than it saves.
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
 * NOTE: if you want to use a different message format, maybe
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
 */

/****************************************************************
 * CONDITIONAL COMPILATION OPTIONS
 ***************************************************************/

/// Define this to force a development build, which sends
// stats and initiates "working day" operation straight away
#define DEV_BUILD

#ifdef DEV_BUILD
/// Enable stats reporting.
# define ENABLE_STATS_REPORTING
#endif

/// Define this to do GPS readings irrespective of the state of the
// accelerometer.
//#define DISABLE_ACCELEROMETER

/// Define this to record GPS values even if the values are not valid.
// The invalid value is TinyGPS::GPS_INVALID_F_ANGLE, which comes
// out as 1000 degrees.
//#define IGNORE_INVALID_GPS

/// Define this if using the USB port for debugging
//#define USB_DEBUG

/// Define this if a 2D GPS fix is sufficient
#define GPS_FIX_2D

/****************************************************************
 * CONFIGURATION MACROS
 ***************************************************************/

/// The maximum amount of time to hang around waiting for a
// connection to the Particle server.
#define WAIT_FOR_CONNECTION_SECONDS 60

/// How long to wait for things to sort themselves out on the
// USB port after waking up from sleep.
#ifdef USB_DEBUG
// NOTE: be careful if you change this, search below for
// #defines whose value is related
# define WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS 5
#else
# define WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS 0
#endif

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
// NOTE: must be bigger than WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS
// when in USB_DEBUG mode.
#define GPS_PERIOD_SECONDS 20

/// The time we wait for GPS to get a fix every GPS_PERIOD_SECONDS.
#define GPS_FIX_TIME_SECONDS 20

/// The interval at which we check for a fix, within
// GPS_FIX_TIME_SECONDS
#define GPS_CHECK_INTERVAL_SECONDS 5

/// The minimum interval between wake-ups, to allow us to
// sleep if the accelerometer is being triggered a lot
// NOTE: must be bigger than WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS
// when in USB_DEBUG mode.
#define MIN_SLEEP_PERIOD_SECONDS 6

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

/// The minimum possible value of Time (in Unix, UTC), used
// to check the sanity of our RTC time.
#define MIN_TIME_UNIX_UTC 1451606400 // 1 Jan 2016 @ midnight

/// The maximum number of consecutive connection failures
// before we take further action
#define MAX_NUM_CONSECUTIVE_CONNECT_FAILURES 5

/// The start time for the device (in Unix, UTC).
// If the device wakes up before this time it will return
// to deep sleep.
// Use http://www.onlineconversion.com/unix_time.htm to work this out.
#ifdef DEV_BUILD
# define START_TIME_UNIX_UTC 1466586000 // 22 June 2016 @ 09:00 UTC
#else
# define START_TIME_UNIX_UTC 1467615600 // 4th July 2016 @ 07:00 UTC
#endif

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
#ifdef DEV_BUILD
# define START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC 1466586000 // 22 June 2016 @ 09:00 UTC
#else
# define START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC 1468134000 // 10 July 2016 @ 07:00 UTC
#endif

/// Start of day in seconds after midnight UTC.
#define START_OF_WORKING_DAY_SECONDS (3600 * 7) // 07:00 UTC, so 08:00 BST

/// Duration of a working day in seconds.
#ifdef DEV_BUILD
# define LENGTH_OF_WORKING_DAY_SECONDS (3600 * 16) // 16 hours, so day ends at 23:00 UTC (00:00 BST)
#else
# define LENGTH_OF_WORKING_DAY_SECONDS (3600 * 10) // 10 hours, so day ends at 17:00 UTC (18:00 BST)
#endif

/// The accelerometer activity threshold (in units of 62.5 mg).
// NOTE: this is extremely sensitive, likely to over-trigger.  A more relaxed value would be
// 10 or 16.
#define ACCELEROMETER_ACTIVITY_THRESHOLD 3

/// GPS module power on delay
#define GPS_POWER_ON_DELAY_MILLISECONDS 500

/// How long to wait for responses from the GPS module after sending commands
#define GPS_DELAY_MILLISECONDS 100

/// How long to wait for an Ack message back from the GPS module
#define GPS_WAIT_FOR_ACK_MILLISECONDS 3000

/// How long to wait for non-ack responses from the GPS module
#define GPS_WAIT_FOR_RESPONSE_MILLISECONDS 2000

/// Allow a gap between characters read from GPS when in command/response mode
#define GPS_INTER_CHARACTER_DELAY_MILLISECONDS 50

/// The offset at the start of a UBX protocol message
#define GPS_UBX_PROTOCOL_HEADER_SIZE 6

/// The minimum number of satellites for which we want to have ephemeris data
#define GPS_MIN_NUM_EPHEMERIS_DATA 5

/// Something to use as an invalid angle
#define GPS_INVALID_ANGLE 999999999

// Something to use as an invalid HDOP
#define GPS_INVALID_HDOP 999999999

/***************************************************************
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
    DEBUG_IND_BOOT_COMPLETE,
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

/// True if we've once achieved a fix
bool gotInitialFix = false;

/// The records accumulated.
Record_t records[100];

/// The current record.
uint32_t currentRecord = 0;

/// The next record to send.
uint32_t nextPubRecord = currentRecord;

/// The number of records queued.
uint32_t numRecordsQueued = 0;

/// Track the number of consecutive connection
// failures
uint32_t numConsecutiveConnectFailures = 0;

/// All the retained variables.
retained Retained_t r;

/// Record if we're currently in motion.
bool inMotion = false;

/// A general purpose buffer, used for sending
// and receiving UBX commands to/from the GPS module
uint8_t msgBuffer[1024];

/// For hex printing
static const char hexTable[] =
{ '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };

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

#ifdef USB_DEBUG
#define LOG_MSG(...) Serial.printf(__VA_ARGS__)
#else
#define LOG_MSG(...)
#endif

/****************************************************************
 * FUNCTION PROTOTYPES
 ***************************************************************/

static uint32_t littleEndianUint32(uint8_t *pByte);
static bool handleInterrupt();
static void debugInd(DebugInd_t debugInd);
static int getImeiCallBack(int type, const char* pBuf, int len, char * pImei);
static void printHex(const char *pBytes, uint32_t lenBytes);
int32_t sendUbx(uint8_t msgClass, uint8_t msgId, const void * pBuf, uint32_t msgLen, bool checkAck);
static uint32_t readGpsMsg (uint8_t *pBuffer, uint32_t bufferLen, uint32_t waitMilliseconds);
static bool configureGps();
static bool gotGpsFix(float *pLatitude, float *pLongitude, float *pElevation, float *pHdop);
static bool canGpsPowerSave();
static void resetRetained();
static time_t gpsOn();
static bool gpsOff();
static bool gpsUpdate(time_t onTimeSeconds, float *pLatitude, float *pLongitude, float *pElevation, float *pHdop);
static void updateTotalGpsSeconds();
static bool connect();
static bool establishTime();
static char * getRecord(RecordType_t type);
static void freeRecord(Record_t *pRecord);
static uint32_t incModRecords (uint32_t x);
static void queueTelemetryReport();
static void queueGpsReport(float latitude, float longitude, bool motion, float hdop);
static void queueStatsReport();
static bool sendQueuedReports();
static uint32_t secondsToWorkingDayStart(uint32_t secondsToday);

/****************************************************************
 * STATIC FUNCTIONS
 ***************************************************************/

/// Return a uint32_t from a pointer to a little-endian uint32_t
// in memory
static uint32_t littleEndianUint32(uint8_t *pByte) {
    uint32_t retValue;
    
    retValue = *pByte;
    retValue += ((uint32_t) *(pByte + 1)) << 8;
    retValue += ((uint32_t) *(pByte + 2)) << 16;
    retValue += ((uint32_t) *(pByte + 3)) << 24;
    
    return  retValue;
}


/// Reset the retained variables.
static void resetRetained() {
    memset (&r, 0, sizeof (r));
    strcpy (r.key, RETAINED_INITIALISED);
    debugInd(DEBUG_IND_RETAINED_RESET);
}

/// Handle the accelerometer interrupt, returning true if activity
// was detected, otherwise false.
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
static int getImeiCallBack(int type, const char* pBuf, int len, char* pImei) {
    
    if ((type == TYPE_UNKNOWN) && (pImei != NULL)) {
        if (sscanf(pBuf, "\r\n%15s\r\n", pImei) == 1) {
            LOG_MSG("IMEI is: %.*s.\n", IMEI_LENGTH, pImei);
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
        case DEBUG_IND_BOOT_COMPLETE:
            // Three long flashes
            digitalWrite(D7, LOW);
            delay (250);
            digitalWrite(D7, HIGH);
            delay (500);
            digitalWrite(D7, LOW);
            delay (250);
            digitalWrite(D7, HIGH);
            delay (500);
            digitalWrite(D7, LOW);
            delay (250);
            digitalWrite(D7, HIGH);
            delay (500);
            digitalWrite(D7, LOW);
            delay (250);
        break;
        default:
        break;
    }
}

/// Print an array of bytes as hex
static void printHex(const uint8_t *pBytes, uint32_t lenBytes)
{
    char hexChar[2];

    for (uint32_t x = 0; x < lenBytes; x++)
    {
        hexChar[0] = hexTable[(*pBytes >> 4) & 0x0f]; // upper nibble
        hexChar[1] = hexTable[*pBytes & 0x0f]; // lower nibble
        pBytes++;
        LOG_MSG("%.2s-", hexChar);
    }
}

/// Send a u-blox format message to the GPS module
int32_t sendUbx(uint8_t msgClass, uint8_t msgId, const void * pBuf, uint32_t msgLen, bool checkAck)
{
    uint8_t head[6] = {0xB5, 0x62, msgClass, msgId, (uint8_t) (msgLen >> 0), (uint8_t) (msgLen >> 8)};
    uint8_t crc[2];
    uint32_t i;
    uint32_t ca = 0;
    uint32_t cb = 0;

    for (i = 2; i < 6; i ++)
    {
        ca += head[i];
        cb += ca;
    }
    
    for (i = 0; i < msgLen; i ++)
    {
        ca += *((uint8_t *) pBuf + i);
        cb += ca; 
    }
    
    LOG_MSG("Sending %d bytes: ", msgLen + sizeof (head) + sizeof (crc));
    i = Serial1.write(head, sizeof(head));
    printHex(head, sizeof(head));
    if (pBuf != NULL) {
        i += Serial1.write((uint8_t *) pBuf, msgLen);
        printHex((uint8_t *) pBuf, msgLen);
    }
    
    crc[0] = ca & 0xFF;
    crc[1] = cb & 0xFF;
    i += Serial1.write(crc, sizeof(crc));
    printHex(crc, sizeof(crc));
    
    LOG_MSG("\n");
    
    if (i < msgLen + sizeof (head) + sizeof (crc)) {
        LOG_MSG("WARNING: not all bytes were sent.\n");
    }
    
    // Check the ack if requested
    // See ublox7-V14_ReceiverDescrProtSpec section 33 (ACK)
    if (checkAck) {
        uint8_t buffer[32];
        bool gotAckOrNack = false;
        uint32_t t;
        
        t = millis();
        LOG_MSG("  > ");
        
        // Wait around for a message that is an ack
        while (!gotAckOrNack && (millis() - t < GPS_WAIT_FOR_ACK_MILLISECONDS)) {
            if (readGpsMsg(buffer, sizeof (buffer), GPS_WAIT_FOR_RESPONSE_MILLISECONDS) == 10) { // 10 is the Ack message size
                // Ack is  0xb5-62-05-00-02-00-msgclass-msgid-crcA-crcB
                // Nack is 0xb5-62-05-01-02-00-msgclass-msgid-crcA-crcB
                if ((buffer[4] == 2) && (buffer[5] == 0) && (buffer[6] == msgClass) && (buffer[7] == msgId) && (buffer[2] == 0x05)) {
                    gotAckOrNack = true;
                    if (buffer[3] != 0x01) {
                        i = -1;
                        LOG_MSG("!!> [Nack]\n");
                    } else {
                        LOG_MSG("  > [Ack]\n");
                    }
                }
            }
        }
        
        if (!gotAckOrNack) {
            i = -2;
            LOG_MSG("\n!!> [No Ack]\n");
        }
    }
    
    return i;
}

/// Read a UBX format GPS message into a buffer.  If waitMilliseconds
// is zero then don't hang around except for the intercharacter delay,
// otherwise wait up to that number of milliseconds for the message
static uint32_t readGpsMsg (uint8_t *pBuffer, uint32_t bufferLen, uint32_t waitMilliseconds) {
    uint32_t x = 0;
    uint16_t msgLen = 0;
    uint32_t ca = 0;
    uint32_t cb = 0;
    bool save;
    bool checksum;
    uint8_t checksumState = 0;
    uint32_t t = millis();
        
    do {
        while (Serial1.available() && (checksumState != 2) && (x < bufferLen)) {
            save = false;
            checksum = false;
            uint8_t c = Serial1.read();
            if ((x == 0) && (c == 0xb5)) {
                // First header byte
                save = true;
            } else if ((x == 1) && (c == 0x62)) {
                // Second header byte
                save = true;
            } else if (x == 2) {
                // Class byte
                save = true;
                checksum = true;
            } else if (x == 3) {
                // ID byte
                checksum = true;
                save = true;
            } else if (x == 4) {
                // First length byte (lower value byte)
                msgLen = c;
                checksum = true;
                save = true;
            } else if (x == 5) {
                // Second length byte (higher value byte)
                msgLen += ((uint16_t) c) << 8;
                checksum = true;
                save = true;
            } else if ((x > 5) && (x < bufferLen) && (x < msgLen + GPS_UBX_PROTOCOL_HEADER_SIZE)) {
                checksum = true;
                save = true;
            } else if (x == msgLen + GPS_UBX_PROTOCOL_HEADER_SIZE) {
                // First checksum byte
                save = true;
                if (c == (uint8_t) ca) {
                    checksumState++;
                }
            } else if (x == msgLen + GPS_UBX_PROTOCOL_HEADER_SIZE + 1) {
                // Second checksum byte
                save = true;
                if (c == (uint8_t) cb) {
                    checksumState++;
                }
            }
            
            if (checksum) {
                ca += c;
                cb += ca; 
            }
            
            if (save) {
                *(pBuffer + x) = c;
                x++;
            }
            
            if (!Serial1.available())
            {
                delay (GPS_INTER_CHARACTER_DELAY_MILLISECONDS);
            }
        }
    } while ((checksumState != 2) && (millis() - t < waitMilliseconds));
    
    if (checksumState == 2) {
        LOG_MSG("Read %d byte(s): ", x);
        printHex(pBuffer, x);
        LOG_MSG("\n");
    } else {
        x = 0;
    }
    
    if (x >= bufferLen) {
        LOG_MSG("WARNING: hit end of buffer (%d bytes).\n", x);
    }
    
    return x;
}

/// Configure the GPS module.
// NOTE: it is up to the caller to make
// sure that the module is powered up
static bool configureGps() {
    bool success = true;

    LOG_MSG("Configuring GPS...\n");
    
    // See ublox7-V14_ReceiverDescrProtSpec section 35.9 (CFG-NAV5)
    LOG_MSG("Setting automotive mode (CFG-NAV5)...\n");
    memset (msgBuffer, 0, sizeof (msgBuffer));
    msgBuffer[0] = 0x00; // Set dynamic config only
    msgBuffer[1] = 0x01;
    msgBuffer[2] = 0x04; // Automotive
    success = (sendUbx(0x06, 0x24, msgBuffer, 36, true) >= 0);

    // See ublox7-V14_ReceiverDescrProtSpec section 35.2 (CFG-CFG)
    LOG_MSG("Storing settings in battery-backed RAM (CFG-CFG)...\n");
    memset (msgBuffer, 0, sizeof (msgBuffer));
     // Set all items in all bitmaps so that we clear, save and re-load
    msgBuffer[0] = 0x00;
    msgBuffer[1] = 0x00;
    msgBuffer[2] = 0x06;
    msgBuffer[3] = 0x1F;
    msgBuffer[4] = 0x00;
    msgBuffer[5] = 0x00;
    msgBuffer[6] = 0x06;
    msgBuffer[7] = 0x1F;
    msgBuffer[8] = 0x00;
    msgBuffer[9] = 0x00;
    msgBuffer[10] = 0x06;
    msgBuffer[11] = 0x1F;
    msgBuffer[12] = 0x01;  // Save in BBR
    success = (sendUbx(0x06, 0x09, msgBuffer, 13, true) >= 0);

    return success;
}

/// Return true if we have a GPS fix and fill in any given parameters (any of which may be NULL)
static bool gotGpsFix(float *pLatitude, float *pLongitude, float *pElevation, float *pHdop) {
    bool gotFix = false;
    float longitude;
    float latitude;
    float elevation = 0;
    float hdop = GPS_INVALID_HDOP;
    
    // See ublox7-V14_ReceiverDescrProtSpec section 39.7 (NAV-PVT)
    LOG_MSG("Checking fix (NAV-PVT)...\n");
    if (sendUbx(0x01, 0x07, NULL, 0, false) >= 0) {
        if (readGpsMsg(msgBuffer, sizeof(msgBuffer), GPS_WAIT_FOR_RESPONSE_MILLISECONDS) > 0) {
#ifdef GPS_FIX_2D
            // Have we got at least a 2D fix?
            if ((msgBuffer[20 + GPS_UBX_PROTOCOL_HEADER_SIZE] == 0x03) || (msgBuffer[20 + GPS_UBX_PROTOCOL_HEADER_SIZE] == 0x02)) {
#else
            // Have we got at least a 3D fix?
            if (msgBuffer[20 + GPS_UBX_PROTOCOL_HEADER_SIZE] == 0x03) {
#endif
                LOG_MSG("%dD fix achieved.\n", msgBuffer[20 + GPS_UBX_PROTOCOL_HEADER_SIZE]);
                if ((msgBuffer[21 + GPS_UBX_PROTOCOL_HEADER_SIZE] & 0x01) == 0x01) {
                    LOG_MSG("gnssFixOK flag is set.\n");
                    gotFix = true;
                    longitude = ((float) (int32_t) littleEndianUint32(&(msgBuffer[24 + GPS_UBX_PROTOCOL_HEADER_SIZE]))) / 10000000;
                    latitude = ((float) (int32_t) littleEndianUint32(&(msgBuffer[28 + GPS_UBX_PROTOCOL_HEADER_SIZE]))) / 10000000;
                    elevation = ((float) (int32_t) littleEndianUint32(&(msgBuffer[36 + GPS_UBX_PROTOCOL_HEADER_SIZE]))) / 1000;
                    LOG_MSG("  > %d satellites used.\n", msgBuffer[23 + GPS_UBX_PROTOCOL_HEADER_SIZE]);
                    LOG_MSG("  > Latitude %.6f.\n", latitude);
                    LOG_MSG("  > Longitude %.6f.\n", longitude);
                    if (msgBuffer[20 + GPS_UBX_PROTOCOL_HEADER_SIZE] == 0x03) {
                        LOG_MSG("  > Elevation %.2f.\n", elevation);
                    } else {
                        LOG_MSG("  > Elevation ---.\n");
                    }
                    
                    // Now get HDOP
                    // See ublox7-V14_ReceiverDescrProtSpec section 39.4 (NAV-DOP)
                    LOG_MSG("Getting HDOP (NAV-DOP)...\n");
                    if (sendUbx(0x01, 0x04, NULL, 0, false) >= 0) {
                        if (readGpsMsg(msgBuffer, sizeof(msgBuffer), GPS_WAIT_FOR_RESPONSE_MILLISECONDS) > 0) {
                            hdop = ((float) ((uint32_t) (msgBuffer[12 + GPS_UBX_PROTOCOL_HEADER_SIZE]) + ((uint32_t) (msgBuffer[13 + GPS_UBX_PROTOCOL_HEADER_SIZE]) << 8))) / 100;
                            LOG_MSG("  > HDOP %.2f.\n", hdop);
                        } else {
                            LOG_MSG("No response.\n");
                        }
                    }

                    if (pLatitude != NULL) {
                        *pLatitude = latitude;
                    }
                    if (pLongitude != NULL) {
                        *pLongitude = longitude;
                    }
                    if (pElevation != NULL) {
                        *pElevation = elevation;
                    }
                    if (pHdop != NULL) {
                        *pHdop = hdop;
                    }
                } else {
                    LOG_MSG("gnssFixOK flag is NOT set (flags are 0x%02x).\n", msgBuffer[21 + GPS_UBX_PROTOCOL_HEADER_SIZE]);
                }
            } else {
                LOG_MSG("No fix (fix is %d).\n", msgBuffer[20 + GPS_UBX_PROTOCOL_HEADER_SIZE]);
            }
        } else {
            LOG_MSG("No response.\n");
        }
    }
    
    return gotFix;
}

/// Determine whether GPS has the necessary data to be
// put into power save state.  This is to have downloaded
// ephemeris data from sufficient satellites and to have
// calibrated its RTC.
// NOTE: it is up to the calleer to check separately if we
// have the required accuracy of fix.
static bool canGpsPowerSave() {
    uint32_t powerSaveState = 0;
    uint32_t numEntries = 0;
    uint32_t numEphemerisData = 0;

    LOG_MSG("Checking if GPS can power save...\n");
    
    // See ublox7-V14_ReceiverDescrProtSpec section 38.2 (MON-HW)
    LOG_MSG("Checking if RTC is calibrated (MON-HW)...\n");
    if (sendUbx(0x0A, 0x09, NULL, 0, false) >= 0) {
        if (readGpsMsg(msgBuffer, sizeof(msgBuffer), GPS_WAIT_FOR_RESPONSE_MILLISECONDS) > 0) {
            if ((msgBuffer[22 + GPS_UBX_PROTOCOL_HEADER_SIZE] & 0x01) == 0x01) {
                // If the rtcCalib bit is set, we're doing good...
                powerSaveState++;
                LOG_MSG("RTC is calibrated.\n");
            } else {
                LOG_MSG("RTC is NOT calibrated.\n");
            }
        } else {
            LOG_MSG("No response.\n");
        }
    }
    
    // See ublox7-V14_ReceiverDescrProtSpec section 39.11 (NAV-SVINFO)
    LOG_MSG("Checking ephemeris data (NAV-SVINFO)...\n");
    if (sendUbx(0x01, 0x30, NULL, 0, false) >= 0) {
        if (readGpsMsg(msgBuffer, sizeof(msgBuffer), GPS_WAIT_FOR_RESPONSE_MILLISECONDS) > 0) {
            // Check how many satellites we have ephemeris data for
            numEntries = msgBuffer[4 + GPS_UBX_PROTOCOL_HEADER_SIZE];
            LOG_MSG("  > %d entry/entries in the list", numEntries);
            
            if (numEntries > 0) {
                LOG_MSG(": \n");
                uint32_t i = GPS_UBX_PROTOCOL_HEADER_SIZE  + 8; // 8 is offset to start of satellites array
                // Now need to check that enough are used for navigation
                for (uint32_t x = 0; x < numEntries; x++) {
                    // Print out the info
                    LOG_MSG("  > chn %3d", msgBuffer[i]);
                    LOG_MSG(", svid %3d", msgBuffer[i + 1]);
                    LOG_MSG(", flags 0x%02x", msgBuffer[i + 2]);
                    LOG_MSG(", quality 0x%02x", msgBuffer[i + 3]);
                    LOG_MSG(", C/N (dBHz) %3d", msgBuffer[i + 4]);
                    LOG_MSG(", elev %3d", msgBuffer[i + 5]);
                    LOG_MSG(", azim %5d", ((uint16_t) msgBuffer[i + 6]) + ((uint16_t) (msgBuffer[i + 7] << 8)));
                    LOG_MSG(", prRes %10d", littleEndianUint32 (&(msgBuffer[i + 8])));
                    if ((msgBuffer[i + 2] & 0x01) == 0x01) { // 2 is offset to flags field in a satellite block
                        numEphemerisData++;
                        LOG_MSG(", used for navigation.\n");
                    } else {
                        LOG_MSG(", NOT usable.\n");
                    }
                    i += 12; // 12 is the size of a satellite block
                }
                LOG_MSG ("  > %d satellite(s) used for navigation with %d required.\n", numEphemerisData, GPS_MIN_NUM_EPHEMERIS_DATA);
                if (numEphemerisData >= GPS_MIN_NUM_EPHEMERIS_DATA) {
                    // Doing even better
                    powerSaveState++;
                }
            } else {
                LOG_MSG(".\n");
            }
        } else {
            LOG_MSG("No response.\n");
        }
    }
    
    if (powerSaveState != 2) {
        LOG_MSG("GPS NOT yet ready to power save.\n");
    } else {
        LOG_MSG("GPS can now power save.\n");
    }
    
    return (powerSaveState == 2);
}

/// Switch GPS on, returning the time that
// GPS was switched on.
static time_t gpsOn() {

    if (digitalRead(D2)) {
        // Log the time that GPS was switched on
        r.gpsPowerOnTime = Time.now();
        
        digitalWrite(D2, LOW);
        LOG_MSG("VCC applied to GPS.\n");
        delay (GPS_POWER_ON_DELAY_MILLISECONDS);
    }

    return r.gpsPowerOnTime;
}

/// Switch GPS off.
static bool gpsOff() {
    bool success = true;

    if (!digitalRead(D2)) {
        // Record the duration that GPS was on for
        updateTotalGpsSeconds();
        
        digitalWrite(D2, HIGH);
        LOG_MSG("VCC removed from GPS.\n");
        
        if (!success) {
            LOG_MSG("WARNING: couldn't send GPS 'off' commands to module before power-off.\n");
        }
    }
    
    return success;
}

/// Update the GPS, making sure it's been on for
// enough time to give us a fix. If onTimeSeconds
// is non-zero then it represents the time at which GPS
// was already switched on.  pLatitude, pLongitude, pElevation,
// pHdop and are filled in with fix values and their accuracy
// if a fix is achieved (and true is returned), otherwise
// they are left alone.
static bool gpsUpdate(time_t onTimeSeconds, float *pLatitude, float *pLongitude, float *pElevation, float *pHdop) {
    bool fixAchieved = false;
    uint32_t startTimeSeconds = Time.now();

    LOG_MSG("Checking for GPS fix for up to %d second(s):\n", GPS_FIX_TIME_SECONDS);

    // Make sure GPS is switched on
    gpsOn();

    if (onTimeSeconds != 0) {
        startTimeSeconds = onTimeSeconds;
    }

    while (!fixAchieved && (Time.now() - startTimeSeconds < GPS_FIX_TIME_SECONDS)) {
        // Check for a fix
        fixAchieved = gotGpsFix(pLatitude, pLongitude, pElevation, pHdop);
        // Sleep while we're waiting
        if (!fixAchieved) {
#ifdef USB_DEBUG
            // If we need the USB to be up there is no time to go to sleep
            delay (GPS_CHECK_INTERVAL_SECONDS * 1000);
#else
            System.sleep(WKP, RISING, GPS_CHECK_INTERVAL_SECONDS, SLEEP_NETWORK_STANDBY);
#endif
        }
    }

    LOG_MSG("\n");
    if (fixAchieved){
        LOG_MSG("Fix achieved in %d second(s): latitude: %.6f, longitude: %.6f, elevation: %.3f m",
                Time.now() - startTimeSeconds, *pLatitude, *pLongitude, *pElevation);
        if (*pHdop != 0) {
            LOG_MSG(", HDOP: %.2f.\n", *pHdop);
        } else {
            LOG_MSG(", no HDOP.\n");
        }
        // See ublox7-V14_ReceiverDescrProtSpec section 41.3 (TIM-VRFY)
        LOG_MSG("\nChecking RTC innaccuracy, for info, (TIM-VRFY)...");
        if (sendUbx(0x0d, 0x06, NULL, 0, false) >= 0) {
            if (readGpsMsg(msgBuffer, sizeof(msgBuffer), GPS_WAIT_FOR_RESPONSE_MILLISECONDS) > 0) {
                LOG_MSG ("  > tow (ms): %d.%0d\n", littleEndianUint32(&(msgBuffer[GPS_UBX_PROTOCOL_HEADER_SIZE])), \
                                                   littleEndianUint32(&(msgBuffer[GPS_UBX_PROTOCOL_HEADER_SIZE + 4])));
                LOG_MSG ("  > delta (ms): ");
                bool negative = false;
                int32_t iDelta = (int32_t) littleEndianUint32(&(msgBuffer[GPS_UBX_PROTOCOL_HEADER_SIZE + 8]));
                int32_t fDelta = (int32_t) littleEndianUint32(&(msgBuffer[GPS_UBX_PROTOCOL_HEADER_SIZE + 12]));
                if (iDelta < 0) {
                    negative = true;
                    iDelta = -iDelta;
                }
                if (fDelta < 0) {
                    negative = true;
                    fDelta = -fDelta;
                }
                if (negative) {
                    LOG_MSG ("-");
                }
                LOG_MSG ("%d.%0d\n", iDelta, fDelta);
                LOG_MSG ("  > week %d\n", (uint32_t) (msgBuffer[GPS_UBX_PROTOCOL_HEADER_SIZE + 16]) + (((uint32_t) msgBuffer[GPS_UBX_PROTOCOL_HEADER_SIZE + 17]) << 8));
                LOG_MSG ("  > flags 0x%02x", msgBuffer[GPS_UBX_PROTOCOL_HEADER_SIZE + 18]);
            } else {
                LOG_MSG("No response.");
            }
        }
    } else {
        LOG_MSG("No fix this time.\n");
    }
    
    // Update the stats
    updateTotalGpsSeconds();

#ifndef DISABLE_GPS_POWER_SAVING
    // If GPS has achieved a fix and met the power-save criteria, switch it off
    if (canGpsPowerSave() && fixAchieved) {
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
        numConsecutiveConnectFailures = 0;
    } else {
        r.numConnectAttempts++;
        LOG_MSG("Connecting to network (waiting for up to %d second(s))... ", WAIT_FOR_CONNECTION_SECONDS);
        Particle.connect();
        if (waitFor(Particle.connected, WAIT_FOR_CONNECTION_SECONDS)) {
            success = true;
            LOG_MSG("Connected.\n");
        } else {
            numConsecutiveConnectFailures++;
            r.numConnectFailed++;
            LOG_MSG("WARNING: connection failed.\n");
        }
    }
    
    return success;
}

/// Make sure we have time sync.
static bool establishTime() {
    bool success = false;

    if (Time.now() < MIN_TIME_UNIX_UTC) {
        LOG_MSG("Syncing time (this will take %d second(s)).\n", TIME_SYNC_WAIT_SECONDS);
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
        LOG_MSG("WARNING: unable to establish time (time now is %d).\n", Time.now());
    }

    return success;
}

/// Get a new record from the list.
static char * getRecord(RecordType_t type) {
    char * pContents;

    LOG_MSG("Using record %d.\n", currentRecord);
    
    ASSERT (currentRecord < (sizeof (records) / sizeof (records[0])), FATAL_RECORDS_OVERRUN_1);
    
    if (records[currentRecord].isUsed) {
        LOG_MSG("WARNING: records queue has wrapped, over-writing old data.\n");
    } else {
        numRecordsQueued++;
    }
    
    ASSERT (numRecordsQueued < (sizeof (records) / sizeof (records[0])), FATAL_RECORDS_OVERRUN_2);
    
    records[currentRecord].isUsed = true;
    records[currentRecord].type = type;
    pContents = records[currentRecord].contents;
    
    // Move the index on to the next record
    currentRecord = incModRecords(currentRecord);
    LOG_MSG("Incremented currentRecord to %d.\n", currentRecord);
    
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
    
   LOG_MSG("Queuing a telemetry report.\n");
    
    // Start a new record
    pRecord = getRecord(RECORD_TYPE_TELEMETRY);

    // Add the device ID
    contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, "%.*s", IMEI_LENGTH, imei);  // -1 for terminator

    // Add battery status
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%.2f", fuel.getSoC());  // -1 for terminator
        LOG_MSG("Battery level is %f.\n", fuel.getSoC());
    } else {
        LOG_MSG("WARNING: couldn't fit battery level into report.\n");
    }
    
    // Add signal strength
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        CellularSignal signal = Cellular.RSSI();
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%d", signal.rssi); // -1 for terminator
        LOG_MSG("RSSI is %d.\n", signal.rssi);
    } else {
        LOG_MSG("WARNING: couldn't fit Signal Strength reading into report.\n");
    }
    
    // Add Unix time
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%u", (unsigned int) Time.now());  // -1 for terminator
        LOG_MSG("Time now is %s UTC.\n", Time.timeStr().c_str());
    } else {
        LOG_MSG("WARNING: couldn't fit timestamp into report.\n");
    }
    
    LOG_MSG("%d byte(s) of record used (%d byte(s) unused).\n", contentsIndex + 1, LEN_RECORD - (contentsIndex + 1)); // +1 to account for terminator
}

/// Queue a GPS report.
static void queueGpsReport(float latitude, float longitude, bool motion, float hdop) {
    char *pRecord;
    uint32_t contentsIndex = 0;

    LOG_MSG("Queuing a GPS report.\n");
    
    // Start a new record
    pRecord = getRecord(RECORD_TYPE_GPS);

    // Add the device ID
    contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, "%.*s", IMEI_LENGTH, imei);  // -1 for terminator

    // Add GPS
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%.6f;%.6f", latitude, longitude);  // -1 for terminator
    } else {
        LOG_MSG("WARNING: couldn't fit GPS reading into report.\n");
    }
        
    // Add Unix time
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%u", (unsigned int) Time.now());  // -1 for terminator
        LOG_MSG("Time now is %s UTC.\n", Time.timeStr().c_str());
    } else {
        LOG_MSG("WARNING: couldn't fit timestamp into report.\n");
    }
    
    // Add motion to the end, to preserve backwards-compatibility
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%d", motion);  // -1 for terminator
    } else {
        LOG_MSG("WARNING: couldn't fit motion indication into report.\n");
    }
        
    // Add HDOP to the end, to preserve backwards-compatibility
    if (hdop != GPS_INVALID_HDOP) {
        if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
            contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%.2f", hdop);  // -1 for terminator
        } else {
            LOG_MSG("WARNING: couldn't fit PDOP into report.\n");
        }
    }

    LOG_MSG("%d byte(s) of record used (%d byte(s) unused).\n", contentsIndex + 1, LEN_RECORD - (contentsIndex + 1)); // +1 to account for terminator
}

/// Queue a stats report.
static void queueStatsReport() {
    char *pRecord;
    uint32_t contentsIndex = 0;
    uint32_t upTimeSeconds = millis() / 1000 + r.totalPowerSaveSeconds;
    
    if (upTimeSeconds == 0) {
        upTimeSeconds = 1; // avoid div by 0
    }

    LOG_MSG("Queuing a stats report.\n");
    
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
        LOG_MSG("WARNING: couldn't fit fatal count and types into report.\n");
    }
    
    // Add up-time
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%ld.%ld:%02ld:%02ld",
                                   upTimeSeconds / 86400, (upTimeSeconds / 3600) % 24, (upTimeSeconds / 60) % 60,  upTimeSeconds % 60);  // -1 for terminator
    } else {
        LOG_MSG("WARNING: couldn't fit up-time into report.\n");
    }
    
    // Add %age power save time
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%ld%%", r.totalPowerSaveSeconds * 100 / upTimeSeconds);  // -1 for terminator
    } else {
        LOG_MSG("WARNING: couldn't fit percentage power saving time into report.\n");
    }
    
    // Add %age GPS on time
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";~%ld%%", r.totalGpsSeconds * 100 / upTimeSeconds);  // -1 for terminator
    } else {
        LOG_MSG("WARNING: couldn't fit percentage GPS on time into report.\n");
    }
    
    // Add loop counts and position percentage
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";L%ldM%ldG%ldP%ld%%",
                                   r.numLoops, r.numLoopsMotionDetected, r.numLoopsGpsOn, (r.numLoopsLocationNeeded != 0) ? (r.numLoopsLocationValid * 100 / r.numLoopsLocationNeeded) : 0); // -1 for terminator
    } else {
        LOG_MSG("WARNING: couldn't fit loop counts into report.\n");
    }
    
    // Add connect counts
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";C%ld-%ld", r.numConnectAttempts, r.numConnectFailed); // -1 for terminator
    } else {
        LOG_MSG("WARNING: couldn't fit connect counts into report.\n");
    }
    
    // Add publish (== send) counts
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";S%ld-%ld", r.numPublishAttempts, r.numPublishFailed); // -1 for terminator
    } else {
        LOG_MSG("WARNING: couldn't fit publish counts into report.\n");
    }

    // Add last accelerometer reading
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";X%dY%dZ%d",
                                   r.accelerometerReading.x, r.accelerometerReading.y, r.accelerometerReading.z); // -1 for terminator
    } else {
        LOG_MSG("WARNING: couldn't fit last accelerometer reading into report.\n");
    }
    
    // Add Unix time
    if ((contentsIndex > 0) && (contentsIndex < LEN_RECORD)) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%u", (unsigned int) Time.now());  // -1 for terminator
        Serial.printf("Time now is %s UTC.\n", Time.timeStr().c_str());
    } else {
        LOG_MSG("WARNING: couldn't fit timestamp into report.\n");
    }
    
    LOG_MSG("%d byte(s) of record used (%d byte(s) unused).\n", contentsIndex + 1, LEN_RECORD - (contentsIndex + 1)); // +1 to account for terminator
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
    LOG_MSG("Sending report(s) (numRecordsQueued %d, nextPubRecord %d, currentRecord %d).\n", numRecordsQueued, x, currentRecord);

    do {
        ASSERT (x < (sizeof (records) / sizeof (records[0])), FATAL_RECORDS_OVERRUN_3);
        LOG_MSG("Report %d: ", x);
        if (records[x].isUsed) {
            // If this is a stats record and we're not reporting stats over the air
            // then just print this record and then mark it as unused
            if ((records[x].type == RECORD_TYPE_STATS) && !sendStatsReport) {
                LOG_MSG("[Stats: %s]\n", records[x].contents);
                freeRecord(&(records[x]));
            } else {
                // This is something we want to publish, so first connect
                if (connect()) {
                    r.numPublishAttempts++;
                    if (Particle.publish(recordTypeString[records[x].type], records[x].contents, 60, PRIVATE)) {
                        LOG_MSG("sent %s.\n", records[x].contents);
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
                        LOG_MSG("WARNING: send failed.\n");
                    }
                } else {
                    failedCount++;
                    LOG_MSG("WARNING: send failed due to not being connected.\n");
                }
            }
            // If there has not yet been a publish failure, increment the place to start next time
            if (failedCount == 0) {
                nextPubRecord = incModRecords(nextPubRecord);
                LOG_MSG("Incremented nextPubRecord to %d.\n", nextPubRecord);
            }
        } else {
            LOG_MSG("unused.\n");
        }

        // Increment x
        x = incModRecords(x);

    } while (x != incModRecords(currentRecord));  // Loop until we've gone beyond the current record

    LOG_MSG("%ld report(s) sent, %ld failed to send.\n", sentCount, failedCount);
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
        LOG_MSG("Awake outside the working day (time now %02d:%02d:%02d UTC, working day is %02d:%02d:%02d to %02d:%02d:%02d), going back to sleep for %d second(s) in order to wake up at %s.\n",
                Time.hour(), Time.minute(), Time.second(),
                Time.hour(START_OF_WORKING_DAY_SECONDS), Time.minute(START_OF_WORKING_DAY_SECONDS), Time.second(START_OF_WORKING_DAY_SECONDS),
                Time.hour(START_OF_WORKING_DAY_SECONDS + LENGTH_OF_WORKING_DAY_SECONDS), Time.minute(START_OF_WORKING_DAY_SECONDS + LENGTH_OF_WORKING_DAY_SECONDS),
                Time.second(START_OF_WORKING_DAY_SECONDS + LENGTH_OF_WORKING_DAY_SECONDS),
                seconds + WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS, Time.timeStr(Time.now() + seconds + WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS).c_str());
        // If we will still be in slow mode when we awake, no need to wake up until the first slow-operation
        // wake-up time
        if (Time.now() + seconds < START_TIME_FULL_WORKING_DAY_OPERATION_UNIX_UTC - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS) {
            seconds += SLOW_MODE_INTERVAL_SECONDS;
            LOG_MSG("Adding %d second(s) to this as we're in slow operation mode, so will actually wake up at %s.\n",
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
    bool gpsConfigured = false;
    
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

    // After a reset of the Electron board it takes a Windows PC several seconds
    // to sort out what's happened to its USB interface, hence you need this
    // delay if you're going to capture the serial output completely
    delay (WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS * 1000);
    
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
    Serial1.blockOnOverrun(true);
    
    // Set D2 to be an output (driving power to the GPS module)
    pinMode(D2, OUTPUT);
    LOG_MSG("VCC applied to GPS module during setup.\n");

    // Do the very initial configuration of GPS and then
    // switch it off again.  This puts settings into battery
    // backed RAM that can be restored later.
    r.gpsPowerOnTime = Time.now();
    digitalWrite(D2, LOW);
    delay(GPS_POWER_ON_DELAY_MILLISECONDS);
    gpsConfigured = configureGps();
    if (!gpsConfigured) {
        LOG_MSG("WARNING: couldn't configure GPS but continuing anyway.\n");
    }
    digitalWrite(D2, HIGH);
    LOG_MSG("VCC removed from GPS module at end of setup.\n");
    delay(GPS_POWER_ON_DELAY_MILLISECONDS);
    
    // Now properly power-up and configure GPS
    gpsOn();

    // Set all the records used to false
    memset (records, false, sizeof (records));

    // Get the IMEI of the module
    LOG_MSG("Getting IMEI...\n");
    Cellular.command(getImeiCallBack, imei, 10000, "AT+CGSN\r\n");

    // Flash the LED to say that all is good
#ifdef DISABLE_ACCELEROMETER
    if (gpsConfigured) {
#else
    if (accelerometerConnected && gpsConfigured) {
#endif
        debugInd(DEBUG_IND_BOOT_COMPLETE);
    }
    
    // Setup is now complete
    setupCompleteSeconds = Time.now();
    LOG_MSG("Started.\n");
}

void loop() {
    time_t sleepForSeconds = 0;
    bool forceSend = false;
    bool modemStaysAwake = false;
    bool atLeastOneGpsReportSent = false;
    time_t gpsOnTime = 0;
    float latitude = GPS_INVALID_ANGLE;
    float longitude = GPS_INVALID_ANGLE;
    float elevation = 0;
    float hdop = GPS_INVALID_HDOP;

    // See if we've moved
    if (handleInterrupt()) {
        inMotion = true;
    }

    // This only for info
    r.numLoops++;
    
    // Wait for USB to sort itself out
    delay (WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS * 1000);

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
                LOG_MSG("WARNING: trying to get IMEI again...\n");
                Cellular.command(getImeiCallBack, imei, 10000, "AT+CGSN\r\n");
            }

            // Check if we are inside the working day
            if ((secondsSinceMidnight >= START_OF_WORKING_DAY_SECONDS) &&
                (secondsSinceMidnight <= START_OF_WORKING_DAY_SECONDS + LENGTH_OF_WORKING_DAY_SECONDS)) {

                modemStaysAwake = true;

                // Is it time to do something?
                if (Time.now() >= r.sleepTime + WAKEUP_PERIOD_SECONDS) {
                    LOG_MSG("\nAwake: time now is %s UTC, slept since %s UTC (%d second(s) ago).\n",
                            Time.timeStr().c_str(), Time.timeStr(r.sleepTime).c_str(), Time.now() - r.sleepTime);

                    // Set the default sleep time after we've done stuff
                    sleepForSeconds = WAKEUP_PERIOD_SECONDS - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS;

                    // Queue a telemetry report, if required
                    if (Time.now() - lastTelemetrySeconds >= TELEMETRY_PERIOD_SECONDS) {
                        lastTelemetrySeconds = Time.now();
                        queueTelemetryReport();
                        LOG_MSG("Forcing a send.\n");
                        forceSend = true;
                    }

                    // Queue a GPS report, if required
                    if (Time.now() - lastGpsSeconds >= GPS_PERIOD_SECONDS) {
                        bool getFix = forceSend;  // Do this so that we always send a GPS report at the telemetry interval
                        r.numLoopsLocationNeeded++;
                        
                        // If we've moved, or don't have a valid reading, take a new reading
                        if (!accelerometerConnected) {
                            getFix = true;
                            LOG_MSG("No accelerometer, getting GPS reading every time.\n");
                        }
                        if (inMotion) {
                            getFix = true;
                            r.numLoopsMotionDetected++;
                            LOG_MSG("*** Motion was detected, getting latest GPS reading.\n");
                        }
                        if (!gotInitialFix) {
                            getFix = true;
                            LOG_MSG("No initial fix yet, trying again.\n");
                        }

                        if (getFix) {
                            r.numLoopsGpsOn++;
                            // Get the latest output from GPS
                            if (gpsUpdate(gpsOnTime, &latitude, &longitude, &elevation, &hdop)) {
                                gotInitialFix = true;
                                r.numLoopsGpsFix++;
                                r.numLoopsLocationValid++;
                                lastGpsSeconds = Time.now();
                                queueGpsReport(latitude, longitude, inMotion, hdop);
#ifdef IGNORE_INVALID_GPS
                            } else {
                                queueGpsReport(latitude, longitude, inMotion, hdop);
#endif
                            }
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
                            LOG_MSG("Force Send was set.\n");
                        }
                        forceSend = false;

                        //atLeastOneGpsReportSent = sendQueuedReports();
                        
                        // Sending reports will have triggered connections.  If the number of connections
                        // is very bad, it might be a network or modem issue, so try putting everything to
                        // sleep to see if that recovers things
                        if (numConsecutiveConnectFailures > MAX_NUM_CONSECUTIVE_CONNECT_FAILURES) {
                            modemStaysAwake = false;
                            LOG_MSG("Failed to connect %d times, power-cycling modem and resetting on next sleep.\n", numConsecutiveConnectFailures);
                        }

                        lastReportSeconds = Time.now(); // Do this at the end as transmission could, potentially, take some time
                    }

                    // If we are still in slow operation and at least one GPS report has been sent, or we've
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

                    // Reset the inMotion flag now that we've acted upon it
                    inMotion = false;
                    
                    // Back to sleep now
                    r.sleepTime = Time.now();
                } else { // else condition of if() we need to do something
                    // Sleep until we do have to do something 
                    sleepForSeconds = r.sleepTime + WAKEUP_PERIOD_SECONDS - Time.now() - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS;
                }
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
                LOG_MSG("Awake too early (time now %s UTC, expected start time %s UTC), going back to sleep for %d second(s) in order to wake up at %s.\n",
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

    LOG_MSG("Loop %ld: now sleeping for %ld second(s) (will awake at %s UTC), modem will be ",
        r.numLoops, sleepForSeconds + WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS,
        Time.timeStr(Time.now() + sleepForSeconds + WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS).c_str());
    if (modemStaysAwake) {
        LOG_MSG("ON while sleeping.\n");
    } else {
        LOG_MSG("OFF while sleeping.\n");
    }

    // Make sure the debug LED is off to save power
    debugInd(DEBUG_IND_OFF);
    
#ifdef USB_DEBUG
    // Leave a little time for serial prints to leave the building before sleepy-byes
    delay (500);
#endif
    
    // Make sure that we always sleep a little
    if (sleepForSeconds < MIN_SLEEP_PERIOD_SECONDS - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS) {
        sleepForSeconds = MIN_SLEEP_PERIOD_SECONDS - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS;
    }
    
    // Now go to sleep for the allotted time.  If the accelerometer interrupt goes
    // off it will be flagged and dealt with when we awake.
    if (modemStaysAwake) {
        // Sleep with the network connection up so that we can send reports
        // without having to re-register
        System.sleep(WKP, RISING, sleepForSeconds, SLEEP_NETWORK_STANDBY);
    } else {
        // Nice deep sleep otherwise, making sure that GPS is off so that we
        // don't use power and also switch off the accelerometer interrupt.
        // NOTE: we will come back from reset after this, only the
        // retained variables will be kept
        if (accelerometerConnected) {
            accelerometer.disableInterrupts();
        }
        gpsOff();
        System.sleep(SLEEP_MODE_DEEP, sleepForSeconds);
    }
}