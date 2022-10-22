#include <Arduino.h>
#include <SPI.h>
#include <NTPClient.h>
#include <TimeLib.h>

#define LED_PIN                 D4
#define PWR_FOR_ANALOG_READ_PIN D5
#define PWR_FOR_WIFI_PIN        D6
#define LM75A_I2C_ADDRESS       0x48

#include <Adafruit_HTU21DF.h>
#include <Temperature_LM75_Derived.h>

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266Ping.h>
#include <PubSubClient.h>

// required for OTA update
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>

#include "credentials.h"

#define SERIAL_DEBUG
#ifndef SERIAL_DEBUG
// disable Serial output
#define Serial SomeOtherwiseUnusedName
static class {
public:
    void begin(...) {}
    void print(...) {}
    void println(...) {}
    void printf(...) {}
} Serial;
#endif

// public variables
struct {
    uint32_t crc32;

    uint32_t dummydata;
    uint32_t hour; // stores the hour of last wakeup
    uint32_t numStarts; // counts the number of starts
    uint32_t lastUpdate; // contains the start number when the last update was sent
    int32_t temperature1; // contains the temperature that was sent at the last update
    int32_t temperature2; // contains the temperature that was sent at the last update
    int32_t analogPinValue; // contains the analog pin value that was sent at the last update
    byte data[512 - 8*4]; // nothing yet
} rtcData;

int32_t temperature1;
int32_t temperature2;
int32_t analogPinValue;

const IPAddress google_host(8, 8, 8, 8); // for testing online status
const float rtcCorrectionFactor = 1.05;
unsigned int updateInterval = 3600 * rtcCorrectionFactor; // in seconds (1 hour)
const int dailyUpdateAtXoclock = 6;
const float minTemperatureDifference = 2; // only update server if newTemp > oldTemp +- diff
const int32_t minAnalogPinValueDifference = 20;
const int waitForApStartupTime = 80; // seconds
const int pingTimeout = 10; // seconds
uint8_t mac[6];

typedef enum {
    TEMPERATURE1 = 1,
    TEMPERATURE2,
    ANALOG_PIN_VALUE
} datatype;

// Initialize the WiFi and MQTT client object
WiFiClient espClient;
PubSubClient mqttclient(espClient);

// required for OTA update
const char* otaHost = "webupdate";
const char* otaSsid = OTA_SSID;
const char* otaPassword = OTA_PASSWORD;

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

bool isOtaUpdate = false;
int otaTimeout = 600; // * 100ms

Adafruit_HTU21DF htu21;
Generic_LM75 lm75a;


#define NTP_SERVER "de.pool.ntp.org"
#define GMT_TIME_ZONE +1
WiFiUDP ntpUDP;
// You can specify the time server pool and the offset, (in seconds)
// additionaly you can specify the update interval (in milliseconds).
NTPClient timeClient(ntpUDP, NTP_SERVER, GMT_TIME_ZONE * 3600 , 60000);


// function prototypes
bool readRtcMemory();
bool writeRtcMemory();
bool getTemperature();
bool sendDataToServer(datatype which);
bool initWiFi();
bool pingHost(IPAddress host);
bool connectToMqttServer();
int roundTo5(int in);
void shortBlink();
void blinkSuccessAndSleep();
void blinkFailedAndSleep();
void blinkNoUpdateAndSleep();

void setup()
{
    bool updateTemperature1 = false;
    bool updateTemperature2 = false;
    bool updateAnalogPinValue = false;

    WiFi.disconnect(true);

    digitalWrite(LED_PIN, HIGH); // active low. disable LED
    digitalWrite(PWR_FOR_ANALOG_READ_PIN, LOW); // disable analog measurement. only enable power during measurement
    digitalWrite(PWR_FOR_WIFI_PIN, HIGH); // there is an external pulldown resistor on this pin
    pinMode(LED_PIN, OUTPUT); // LED
    pinMode(PWR_FOR_ANALOG_READ_PIN, OUTPUT); // enable output
    pinMode(PWR_FOR_WIFI_PIN, INPUT); // it is configured as input by default. it is pulled down by external resistor
    pinMode(D0, WAKEUP_PULLUP); // reset after deep sleep
    pinMode(D7, INPUT_PULLUP); // if low during boot -> ota update

    Serial.begin(115200);
    delay(100);
    Serial.println();

    // start OTA update modus if D3 is low
    if (digitalRead(D7) == LOW)
    {
        memset(&rtcData, 0xff, sizeof(rtcData));
        ESP.rtcUserMemoryWrite(0, (uint32_t*) &rtcData, sizeof(rtcData));

        WiFi.softAP(otaSsid, otaPassword);

        MDNS.begin(otaHost);

        httpUpdater.setup(&httpServer);
        httpServer.begin();

        MDNS.addService("http", "tcp", 80);
        Serial.println("HTTPUpdateServer ready!");
        Serial.printf("Connect to Wifi %s, password %s and\n", otaSsid, otaPassword);
        Serial.printf("open http://%s.local/update in your browser\n", otaHost);

        isOtaUpdate = true;

        return;
    }

    if (readRtcMemory()) // CRC of rtcData is valid
    {
        rtcData.numStarts++;
        if (rtcData.hour == 26) // not set via internet yet
        {}
        else if (rtcData.hour < 23)
        {
            rtcData.hour++;
        }
        else
        {
            rtcData.hour = 0;
        }

        Serial.print("Started ");
        Serial.print(rtcData.numStarts);
        Serial.println(" times");

        Serial.print("It is something around ");
        Serial.print(rtcData.hour);
        Serial.println(":00 o'clock");
    }
    else // CRC of rtcData is invalid
    {
        rtcData.numStarts = 1;
        rtcData.lastUpdate = 1;
        rtcData.hour = 26; // initialize to illegal value to be able to check if was set via internet at all
        Serial.println("First start");

        updateTemperature1 = true;
        updateTemperature2 = true;
        updateAnalogPinValue = true;
    }

    if (rtcData.hour == dailyUpdateAtXoclock)
    {
        Serial.print("forcing daily update");

        updateTemperature1 = true;
        updateTemperature2 = true;
        updateAnalogPinValue = true;
    }

    Wire.begin(D1, D2);

    getTemperature();

    digitalWrite(PWR_FOR_ANALOG_READ_PIN, HIGH);
    delay(500);
    analogPinValue = roundTo5(analogRead(A0));
    delay(200);
    digitalWrite(PWR_FOR_ANALOG_READ_PIN, LOW);

    if ((temperature1 <= (rtcData.temperature1 - minTemperatureDifference)) || \
        (temperature1 >= (rtcData.temperature1 + minTemperatureDifference)))
    {
        Serial.print("Temperature 1 changed from ");
        Serial.print(rtcData.temperature1);
        Serial.print("°C to ");
        Serial.print(temperature1);
        Serial.println("°C");
        updateTemperature1 = true;
    }

    if ((temperature2 <= (rtcData.temperature2 - minTemperatureDifference)) || \
        (temperature2 >= (rtcData.temperature2 + minTemperatureDifference)))
    {
        Serial.print("Temperature 2 changed from ");
        Serial.print(rtcData.temperature2);
        Serial.print("°C to ");
        Serial.print(temperature2);
        Serial.println("°C");
        updateTemperature2 = true;
    }

    if ((analogPinValue <= (rtcData.analogPinValue - minAnalogPinValueDifference)) || \
        (analogPinValue >= (rtcData.analogPinValue + minAnalogPinValueDifference)))
    {
        Serial.print("analog pin value changed from ");
        Serial.print(rtcData.analogPinValue);
        Serial.print(" to ");
        Serial.println(analogPinValue);

        updateAnalogPinValue = true;
    }

    if (updateTemperature1 || updateTemperature2 || updateAnalogPinValue)
    {
        if (! initWiFi())
        {
            writeRtcMemory();
            blinkFailedAndSleep();
        }

        if (! pingHost(google_host))
        {
            writeRtcMemory();
            blinkFailedAndSleep();
        }

        // connecting to wifi takes some time. substract this from sleep time
        updateInterval = updateInterval - waitForApStartupTime;

        /* calibrate time with ntp-time once per day and at first startup */
        if (rtcData.hour == 26 || rtcData.hour == dailyUpdateAtXoclock)
        {
            timeClient.begin();
            if (! timeClient.update())
                goto ntpNotUpdated;
            timeClient.end();

            Serial.print("Internet Time: ");
            Serial.println(timeClient.getFormattedTime());

            setTime(timeClient.getEpochTime());

            // simple adjustment for daylight saving time
            if ((month() > 3) && (month() < 11))
            {
                adjustTime(+3600);
            }

            TimeElements tm;

            tm.Year = year() - 1970;
            tm.Month = month();
            tm.Day = day();
            tm.Minute = 0;
            tm.Second = 0;

            tm.Hour = hour() + 1;
            rtcData.hour = hour();

            // if we woke up at 5:5x, next check is at 7:00
            if ((hour() == dailyUpdateAtXoclock-1) && (minute() > 50))
            {
                tm.Hour += 1;
                rtcData.hour += 1;
            }

            if (tm.Hour == 24)
            {
                tm.Hour = 0;
                tm.Day += 1;
            }

            // sleep until next full hour
            updateInterval = ((makeTime(tm) - now()) * rtcCorrectionFactor) + 180;
            rtcData.hour = hour();
        }

ntpNotUpdated:
        if (!connectToMqttServer())
        {
            writeRtcMemory();
            blinkFailedAndSleep();
        }

        bool result = true;
        if (updateTemperature1) {
            if (sendDataToServer(TEMPERATURE1)) {
                rtcData.temperature1 = temperature1;
            }
            else {
                result = false;
            }
        }

        if (updateTemperature2) {
            if (updateTemperature1) {
                delay(500); // do not publish one after each other too fast
            }
            if (sendDataToServer(TEMPERATURE2)) {
                rtcData.temperature2 = temperature2;
             }
            else {
                result = false;
            }
        }

        if (updateAnalogPinValue) {
            if (updateTemperature1 || updateTemperature2) {
                delay(500); // do not publish one after each other too fast
            }
            if (sendDataToServer(ANALOG_PIN_VALUE)) {
                rtcData.analogPinValue = analogPinValue;
            }
            else {
                result = false;
            }
        }

        if (result) {
            rtcData.lastUpdate = rtcData.numStarts;

            writeRtcMemory();
            blinkSuccessAndSleep();
        }
        else
        {
            writeRtcMemory();
            blinkFailedAndSleep();
        }
    }
    else
    {
        writeRtcMemory();
        blinkNoUpdateAndSleep();
    }

    // do not put anything here, cpu will sleep and never reach this point
}

void loop()
{
    if (isOtaUpdate) {
        httpServer.handleClient();
        MDNS.update();

        if (otaTimeout % 10 == 0) {
            Serial.printf("Reboot in %d seconds\n", otaTimeout/10);
        }

        if (otaTimeout > 60) {
            if (otaTimeout-- % 10 == 0) {
                digitalWrite(LED_PIN, LOW);
                delay(50);
                digitalWrite(LED_PIN, HIGH);
                delay(50);
            }
        }
        else {
            if (otaTimeout-- % 5 == 0) {
                digitalWrite(LED_PIN, LOW);
                delay(50);
                digitalWrite(LED_PIN, HIGH);
                delay(50);
            }
        }

        if (otaTimeout <= 0) {
            ESP.deepSleep(1e6); // blinkNoUpdateAndSleep();
        }

        delay(100);
    }
}

uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

bool readRtcMemory()
{
    // Read struct from RTC memory
    if (ESP.rtcUserMemoryRead(0, (uint32_t*) &rtcData, sizeof(rtcData))) {
        uint32_t crcOfData = calculateCRC32((uint8_t*) &rtcData.dummydata, sizeof(rtcData) - 4);
        if (crcOfData == rtcData.crc32)
        {
            return true;
        }
    }

    return false;
}

bool writeRtcMemory()
{
    Serial.print("Writing RTC memory ... ");
    // Update CRC32 of data
    rtcData.crc32 = calculateCRC32((uint8_t*) &rtcData.dummydata, sizeof(rtcData) - 4);
    // Write struct to RTC memory
    if (ESP.rtcUserMemoryWrite(0, (uint32_t*) &rtcData, sizeof(rtcData)))
    {
        Serial.println("[DONE]");
        return true;
    }

    Serial.println("[FAILED]");
    return false;
}

bool getTemperature()
{
    float readTemp1;
    htu21.begin();
    delay(200);

    readTemp1 = htu21.readTemperature();

    // in case of i2c error (readTemperature() return 0.0), try again
    if (readTemp1 == 0.0f)
    {
        delay(100);
        readTemp1 = htu21.readTemperature();
    }

    temperature1 = readTemp1;

    lm75a.disableShutdownMode();
    delay(200);
    temperature2 = lm75a.readTemperatureC();
    lm75a.enableShutdownMode();

    bool ret = true;

    Serial.print("Collecting temperature data ... ");

    if (isnan(temperature1)) {  // check if 'is not a number'
        ret = false;
    }

    if (isnan(temperature2)) {  // check if 'is not a number'
        ret = false;
    }

    if (ret == true)
    {
        Serial.println("[DONE]");
    }
    else
    {
        Serial.println("[FAILED]");
    }

    Serial.print("Temperature 1 is: ");
    Serial.print(temperature1);
    Serial.println("°C");

    Serial.print("Temperature 2 is: ");
    Serial.print(temperature2);
    Serial.println("°C");

    return ret;
}

bool sendDataToServer(datatype which)
{
    bool ret = false;
    char feed[64];
    char attributes[10];

    uint8_t mac[6];
    wifi_get_macaddr(STATION_IF, mac);

    switch (which) {
        case TEMPERATURE1:
            sprintf(feed, "%s%02x_%s", MQTT_USERNAME"/feeds/", mac[5], "temperature1");
            sprintf(attributes, "%02d", temperature1);
            Serial.print("Sending temperature 1 to server ... ");
            break;

        case TEMPERATURE2:
            sprintf(feed, "%s%02x_%s", MQTT_USERNAME"/feeds/", mac[5], "temperature2");
            sprintf(attributes, "%02d", temperature2);
            Serial.print("Sending temperature 2 to server ... ");
            break;

        case ANALOG_PIN_VALUE:
            sprintf(feed, "%s%02x_%s", MQTT_USERNAME"/feeds/", mac[5], "analog");
            sprintf(attributes, "%04d", analogPinValue);
            Serial.print("Sending analog pin value to server ... ");
            break;
    }
    // Send payload
    if (mqttclient.publish(feed, attributes))
    {
        Serial.println("[DONE]");
        ret = true;
    }
    else
    {
        Serial.println("[FAILED]");
        ret = false;
    }

    mqttclient.loop();

    return ret;
}

bool initWiFi()
{
    int i = 0;

    // enable power for AP
    pinMode(PWR_FOR_WIFI_PIN, OUTPUT);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // attempt to connect to WiFi network
    while (WiFi.status() != WL_CONNECTED) {
        Serial.println();
        Serial.print("Connecting to SSID ");
        Serial.print(WIFI_SSID);
        Serial.print(" ... ");
        delay(1000);
        wdt_reset();
        if (i++ > waitForApStartupTime)
            break;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("[DONE]");
        return true;
    }
    else
    {
        Serial.print("[FAILED] [ rc = ");
        Serial.print(WiFi.status());
        Serial.println("]");
        return false;
    }
}

bool pingHost(IPAddress host)
{
    bool ret = false;
    int i = 0;

    while (true)
    {
        Serial.print("Pinging ");
        Serial.print(host);
        Serial.print(" ... ");

        delay(1000);
        i++;

        if (Ping.ping(host))
        {
            Serial.println("[DONE]");
            ret = true;
            break;
        }
        else if (i > pingTimeout)
        {
            Serial.println("[FAILED]");
            break;
        }
    }

    return ret;
}

bool connectToMqttServer()
{
    bool ret = false;
    int i = 0;

    mqttclient.setServer(SERVER, 1883);

    // Loop until we're connected
    while (!mqttclient.connected())
    {
        Serial.print("Connecting to ");
        Serial.print(SERVER);
        Serial.print(" ... ");
        // Attempt to connect (clientId, username, password)
        if (mqttclient.connect("Wemos D1", MQTT_USERNAME, MQTT_KEY))
        {
        Serial.println("[DONE]");
        ret = true;
        }
        else
        {
            Serial.print("[FAILED] [ rc = ");
            Serial.print(mqttclient.state());
            Serial.println("] : retrying in 5 seconds]");
            // Wait 5 seconds before retrying
            delay(5000);
            i++;
        }
        if (i > 5)
            break;
    }
    return ret;
}

int roundTo5(int in)
{
    const signed char round5delta[5] = {0, -1, -2, 2, 1};  // difference to the "rounded to nearest 5" value
    return in + round5delta[in%5];
}

// pattern ".."
void shortBlink()
{
#ifdef SERIAL_DEBUG
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);
    delay(50);
#endif
}

// pattern "- -"
void blinkSuccessAndSleep()
{
    Serial.print("Sleep for ");
    Serial.print(updateInterval);
    Serial.println(" seconds ...");
#ifdef SERIAL_DEBUG
    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);
#endif
    // sleep for x seconds
    ESP.deepSleep(updateInterval * 1e6);
    delay(100);
}

// pattern "- ..."
void blinkFailedAndSleep()
{
    Serial.print("Sleep for ");
    Serial.print(updateInterval);
    Serial.println(" seconds ...");
#ifdef SERIAL_DEBUG
    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);  // sleep for x seconds
#endif
    ESP.deepSleep(updateInterval * 1e6);
    delay(100);
}

// pattern ".."
void blinkNoUpdateAndSleep()
{
    Serial.print("Sleep for ");
    Serial.print(updateInterval);
    Serial.println(" seconds ...");
#ifdef SERIAL_DEBUG
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);
#endif
    ESP.deepSleep(updateInterval * 1e6);
    delay(100);
}
