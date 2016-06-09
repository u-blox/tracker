#include "accelerometer.h"
#include "TinyGPS.h"

/* Golf Buggy Tracker
 * This is the Golf Buggy tracker.  It uses a Particle Electron
 * board, a u-blox PAM7Q GPS module and a ADXL345 Accelerometer
 * to establish the GPS position of itself in a power-efficient
 * way and reports this back to the Particle server and, from 
 * there, via Webhooks, anyone who is interested.
 *
 * The (JSON) message formats are hilghly compressed to
 * save data and are as follows:
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

// The length of the IMEI, used as the device ID
#define IMEI_LENGTH 15

// The maximum amount of time to hang around waiting for a
// connection to the Particle server
#define WAIT_FOR_CONNECTION_SECONDS 60

// How long to wait for things to sort themselves out (e.g. the
// USB port) after waking up from sleep
#define WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS 5

// The minimum time between GPS fixes when moving.  This
// must always be the minimum period because it is also used
// as the sleep period.
// NOTE: must be bigger than WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS
#define GPS_PERIOD_SECONDS 30

// The periodicity of telemetry reports
#define TELEMETRY_PERIOD_SECONDS 600

// The report period in seconds 
#define REPORT_PERIOD_SECONDS 120

// The size of a JSON record
#define LEN_RECORD 200

// The queue length at which to try sending a record
#define QUEUE_SEND_LEN 4

// Define this to allow the system to go to lowest power state
//#define SLEEP_ENABLED

// Define this to always send GPS, even if the values are not valid
//#define SEND_GPS_ALWAYS

/****************************************************************
 * TYPES
 ***************************************************************/

// The possible record types
typedef enum {
    RECORD_TYPE_NULL,
    RECORD_TYPE_TELEMETRY,
    RECORD_TYPE_GPS,
    MAX_NUM_RECORD_TYPES
} RecordType;

// A single record
typedef struct {
    bool isUsed;
    RecordType type;
    char contents[LEN_RECORD];
} Record;

/****************************************************************
 * GLOBAL VARIABLES
 ***************************************************************/

// Record type strings
// NOTE: must match the RecordType enum above
const char * recordTypeString[] = {"null", "telemetry", "gps"};

// Variables to hold X, Y and Z booleans
bool accel[] = {false, false, false};

// The IMEI of the module
char imei[IMEI_LENGTH] = "";

// When we last tried to get a fix
uint32_t lastGpsSeconds = 0;

// Time Of the last ping message
uint32_t lastPingSeconds = 0;

// Time Of the last telemetry message
uint32_t lastTelemetrySeconds = 0;

// Time Of the last report
uint32_t lastReportSeconds = 0;

// The number of fixes after which to
// publish what we've got
uint32_t pubInterval = 4;

// The records accumulated
Record records[20];

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
uint32_t sleepTime = 0;

// Instantiate GPS
TinyGPS gps;

// Instantiate the accelerometer
Accelerometer accelerometer = Accelerometer();

// Instantiate a fuel gauge
FuelGauge fuel;

// Only connect when it is required
SYSTEM_MODE(SEMI_AUTOMATIC);

/****************************************************************
 * FUNCTION PROTOTYPES
 ***************************************************************/

static int getImeiCallBack(int type, const char* pBuf, int len, char* imei);
static uint8_t incModRecords (uint8_t x);
static bool gpsUpdate();
static void connect();
static char * getRecord(RecordType type);
static void queueTelemetryReport();
static void queueGpsReport(float latitude, float longitude);

/****************************************************************
 * STATIC FUNCTIONS
 ***************************************************************/

// Parse the IMEI from the module
static int getImeiCallBack(int type, const char* pBuf, int len, char* imei) {
    
    if ((type == TYPE_UNKNOWN) && imei) {
        if (sscanf(pBuf, "\r\n%15s\r\n", imei) == 1) {
            Serial.printf("IMEI is: %*s.\n", IMEI_LENGTH, imei);
        }
    }

    return WAIT;
}

// Increment a number and return the incremented
// number modulo the number of records available
static uint8_t incModRecords (uint8_t x) {
    x++;
    if (x >= (sizeof (records) / sizeof (Record))) {
        x = 0;
    }
    
    return x;
}

// Update the GPS
static bool gpsUpdate() {
    bool isValidGps = false;
    
    uint32_t start = millis();
    while (millis() - start < 1000) {
        // Check GPS data is available
        while (Serial1.available()) {
            // parse GPS data
            if (gps.encode(Serial1.read())) {
                isValidGps = true;
            }
        }
    }
    
    return isValidGps;
}

// Connect to the network
static void connect() {
    if (!Particle.connected()) {
        Serial.printf("Connecting to network (waiting for up to %d seconds)...", WAIT_FOR_CONNECTION_SECONDS);
        Particle.connect();
        waitFor(Particle.connected, WAIT_FOR_CONNECTION_SECONDS);
        if (Particle.connected()) {
            Serial.println("Connected.");
        } else {
            Serial.println("WARNING: connection failed.");
        }
    } else {
        Serial.println("Already connected to network.");
    }
}

// Get a record
static char * getRecord(RecordType type) {
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
    
    // Move the index  on to the next record
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
    if (contentsIndex < LEN_RECORD) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, "%*s", IMEI_LENGTH, imei);  // -1 for terminator
    } else {
        Serial.println("WARNING: couldn't fit device ID into report.");
    }
    
    // Add battery status
    if (contentsIndex < LEN_RECORD) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%.2f", fuel.getSoC());  // -1 for terminator
        Serial.printf("Battery level is %f.\n", fuel.getSoC());
    } else {
        Serial.println("WARNING: couldn't fit battery level into report.");
    }
    
    // Add signal strength
    if (contentsIndex < LEN_RECORD) {
        CellularSignal signal = Cellular.RSSI();
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%d", signal.rssi); // -1 for terminator
        Serial.printf("RSSI is %d.\n", signal.rssi);
    } else {
        Serial.println("WARNING: couldn't fit Signal Strength reading into report.");
    }
    
    // Add Unix time
    if (contentsIndex < LEN_RECORD) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%u", (unsigned int) (uint32_t) Time.now());  // -1 for terminator
        Serial.println("Time now is " + Time.timeStr());
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
    if (contentsIndex < LEN_RECORD) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, "%*s", IMEI_LENGTH, imei);  // -1 for terminator
    } else {
        Serial.println("WARNING: couldn't fit device ID into report.");
    }
    
    // Add GPS
    if (contentsIndex < LEN_RECORD) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%.6f;%.6f", latitude, longitude);  // -1 for terminator
        Serial.printf("Latitude: %.6f, Longitude: %.6f.\n", latitude, longitude);
    } else {
        Serial.println("WARNING: couldn't fit GPS reading into report.");
    }
        
    // Add Unix time
    if (contentsIndex < LEN_RECORD) {
        contentsIndex += snprintf (pRecord + contentsIndex, LEN_RECORD - contentsIndex - 1, ";%u", (unsigned int) (uint32_t) Time.now());  // -1 for terminator
        Serial.println("Time now is " + Time.timeStr());
    } else {
        Serial.println("WARNING: couldn't fit timestamp into report.");
    }
    
    Serial.printf ("%d bytes of record used (%d unused).\n", contentsIndex + 1, LEN_RECORD - (contentsIndex + 1)); // +1 to account for terminator
}

/****************************************************************
 * GLOBAL FUNCTIONS
 ***************************************************************/

// setup() and loop() are both required. setup() runs once when the device starts
// and is used for registering functions and variables and initializing things
void setup() {
    // Opens up a Serial port so you can listen over USB
    Serial.begin(9600);
    
    // After a reset of the Photon board it takes a Windows PC several seconds
    // to sort out what's happened to its USB interface, hence you need this
    // delay if you're going to capture the serial output from here.
    delay (5000);
    
    // Connect to the network so as to establish time
    connect();

   // Set up all the necessary accelerometer bits
    accelerometer.begin();

    // Start the serial port for the GPS module
    Serial1.begin(9600);

    // Enable the GPS module. Defaults to off to save power. 
    // Takes 1.5s or so because of delays.
    //tracker.gpsOn();
    
    // Set all the records used to false
    memset (records, false, sizeof (records));

    // Get the IMEI of the module
    Serial.println("Getting IMEI...");
    Cellular.command(getImeiCallBack, imei, 10000, "AT+CGSN\r\n");
    Serial.println("Started.");
}

// loop() runs continuously
void loop() {
    bool gpsValid;

#ifdef SLEEP_ENABLED
    if (!firstTime) {
        // It seems counterintuitive to add a delay here but, if you don't
        // then, on wake-up from sleep, one or other of the calls below
        // stalls for some time (maybe one of the serial functions) and so
        // it actually saves power to wait a little while here.
        delay (WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS * 1000);
    }
#endif    

    // Update the GPS
    gpsValid = gpsUpdate();
    
    accelerometer.read(NULL, NULL, NULL);
    
    if (firstTime || (Time.now() >= sleepTime + GPS_PERIOD_SECONDS)) {
        Serial.printf("Time now %d, been asleep since %d (%d seconds ago).\n", Time.now(), sleepTime,  Time.now() - sleepTime);
        
        // I have seen IMEI retrieval fail so check again here
        if (imei[0] < '0') {
            Serial.println("Trying to get IMEI again...");
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
        if (Time.now() - lastGpsSeconds >= GPS_PERIOD_SECONDS) {
            float latitude;
            float longitude;
            uint32_t age;
            // Only go on if there is a valid GPS sentence
#ifdef SEND_GPS_ALWAYS
            gpsValid = true;
#endif
            if (gpsValid) {
                gps.f_get_position(&latitude, &longitude, &age);
                // Only report if we have a fix
#ifdef SEND_GPS_ALWAYS
                if (true) {
#else
                if ((latitude != TinyGPS::GPS_INVALID_F_ANGLE) && (longitude != TinyGPS::GPS_INVALID_F_ANGLE)) {
#endif
                    lastGpsSeconds = Time.now();
                    queueGpsReport(latitude, longitude);
                } else {
                    Serial.println("No GPS fix yet.");
                }
            } else {
                Serial.println("No valid GPS data yet.");
            }
        }
        
        // Check if it's time to do the pub
        if (forceSend || ((Time.now() - lastReportSeconds >= REPORT_PERIOD_SECONDS) && (numRecordsQueued >= QUEUE_SEND_LEN))) {
            uint8_t sentCount = 0;
            uint8_t failedCount = 0;
            uint8_t x;
    
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
                        Serial.println("WARNING send failed.");
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
            
            Serial.printf("%d reports sent, %d failed to send.\n", sentCount, failedCount);
            // If there's been a publish failure and nextPubRecord is
            // equal to currentRecord then we must have wrapped.  In this
            // case, increment nextPubRecord to keep things in time order.
            if ((failedCount > 0) && (nextPubRecord == currentRecord)) {
                nextPubRecord = incModRecords(nextPubRecord);
            }
            
            lastReportSeconds = Time.now();
        }
        
        // It is no longer our first time
        firstTime = false;
        
        // Let the serial prints leave the building before we go to sleepy-byes
        delay (500);
        
        // Sleep for the given number of seconds
        Serial.printf("Sleeping for %d seconds.\n", GPS_PERIOD_SECONDS);
        sleepTime = Time.now();
#ifdef SLEEP_ENABLED        
        System.sleep(D3, RISING, GPS_PERIOD_SECONDS - WAIT_FOR_WAKEUP_TO_SETTLE_SECONDS, SLEEP_NETWORK_STANDBY);
#endif
    }
}