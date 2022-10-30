
#include <Arduino.h>
#include <SPI.h>
#include <LittleFS.h>
#include <SoftwareSerial.h>

#include <NTPClient.h>
#include <TimeLib.h>
#include <Timezone.h>
#include <RTClib.h>

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

#define I2C_SDA_PIN             D1
#define I2C_SCL_PIN             D2
#define ENERGY_METER_RX_PIN     D3
#define ENERGY_METER_TX_PIN     D4
#define LED_PIN                 D4
#define PWR_FOR_ANALOG_READ_PIN D5
#define PWR_FOR_WIFI_PIN        D6
#define ENTER_OTA_UPDATE_PIN    D7

#define LM75A_I2C_ADDRESS       0x48

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
    uint32_t numStarts; // counts the number of starts
    int32_t dailyForcedUpdateDone; // set to false at 0:00, set to true after successful push to server
    int32_t temperature1; // contains the temperature that was sent at the last update
    int32_t temperature2; // contains the temperature that was sent at the last update
    int32_t analogPinValue; // contains the analog pin value that was sent at the last update
    byte data[512 - 7*4]; // nothing yet
} rtcData;

int32_t temperature1;
int32_t temperature2;
int32_t analogPinValue;
float energy;

const IPAddress google_host(8, 8, 8, 8); // for testing online status
const int dailyUpdateAtXoclock = 6;
const float minTemperatureDifference = 2; // only update server if newTemp > oldTemp +- diff
const int32_t minAnalogPinValueDifference = 20;
const int waitForApStartupTime = 80; // seconds
const int pingTimeout = 50; // seconds
uint8_t mac[6];

typedef enum {
    TEMPERATURE1 = 1,
    TEMPERATURE2,
    ANALOG_PIN_VALUE,
    ENERGY
} datatype;

// Initialize the WiFi and MQTT client object
WiFiClient espClient;
PubSubClient mqttclient(espClient);
WiFiUDP ntpUDP;

// required for OTA update
const char* otaHost = "webupdate";
const char* otaSsid = OTA_SSID;
const char* otaPassword = OTA_PASSWORD;

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

bool isOtaUpdate = false;
int otaTimeout = 600; // * 100ms

const char index_html_top[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang='de'>
<head>
    <meta charset='UTF-8'>
    <meta name= viewport content='width=device-width, initial-scale=1.0,' user-scalable=yes>
    <style type='text/css'>
        <!-- DIV.container { min-height: 10em; display: table-cell; vertical-align: middle }.button {height:35px; width:90px; font-size:16px} -->
        body {background-color: white;}
    </style>
</head>
<body>
<pre>
    <center>
        <h2>ESP 8266 LittleFS</h2>
    </center>
)rawliteral";

// Sensors
Generic_LM75 lm75a;

// RTC
RTC_DS3231 rtc;

#define NTP_SERVER "de.pool.ntp.org"
NTPClient timeClient(ntpUDP, NTP_SERVER);

TimeChangeRule summertime = {"CEST", Last, Sun, Mar, 2, 120}; //Daylight time = UTC + 2 hours
TimeChangeRule wintertime = {"CET", Last, Sun, Oct, 3, 60}; //Standard time = UTC + 1 hours
Timezone myTZ(summertime, wintertime);
TimeChangeRule *tcr;        //pointer to the time change rule, use to get TZ abbrev

SoftwareSerial swSer;

// function prototypes
bool readRtcMemory();
bool writeRtcMemory();
bool syncTime();
String prettyTime();
bool getTemperature();
bool getEnergy();
bool saveEnergyToFilesystem();
bool sendDataToServer(datatype which);
bool initWiFi();
bool pingHost(IPAddress host);
bool connectToMqttServer();
void startOtaUpdateModus();
void handleRoot();
bool handleFileRead(String path);
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
    bool updateEnergy = false;

    WiFi.disconnect(true);

    digitalWrite(LED_PIN, HIGH); // active low. disable LED
    digitalWrite(PWR_FOR_ANALOG_READ_PIN, LOW); // disable analog measurement. only enable power during measurement
    digitalWrite(PWR_FOR_WIFI_PIN, HIGH); // there is an external pulldown resistor on this pin
    pinMode(LED_PIN, OUTPUT); // LED
    pinMode(PWR_FOR_ANALOG_READ_PIN, OUTPUT); // enable output
    pinMode(PWR_FOR_WIFI_PIN, INPUT); // it is configured as input by default. it is pulled down by external resistor
    pinMode(D0, WAKEUP_PULLUP); // reset after deep sleep
    pinMode(ENTER_OTA_UPDATE_PIN, INPUT_PULLUP); // if low during boot -> ota update

    swSer.begin(300, SWSERIAL_7E1, ENERGY_METER_RX_PIN, ENERGY_METER_TX_PIN);
    swSer.setTimeout(2500);

    Serial.begin(115200);
    delay(500);
    Serial.println();

    // start OTA update modus if ENTER_OTA_UPDATE_PIN is low
    if (digitalRead(ENTER_OTA_UPDATE_PIN) == LOW)
    {
        startOtaUpdateModus();
        return;
    }

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    // read time from external RTC
    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
    }
    rtc.clearAlarm(1); // assume we were woken up by RTC alarm -> disable alarm

    time_t utc = rtc.now().unixtime();
    setTime(myTZ.toLocal(utc, &tcr));
    Serial.print("Start at ");
    Serial.println(prettyTime());

    if (readRtcMemory()) // CRC of rtcData is valid
    {
        rtcData.numStarts++;

        Serial.print("Started ");
        Serial.print(rtcData.numStarts);
        Serial.println(" times");
    }
    else // CRC of rtcData is invalid
    {
        rtcData.numStarts = 1;
        rtcData.dailyForcedUpdateDone = false;
        Serial.println("First start");

        // initialize external RTC at first start
        rtc.disable32K();

        DateTime alarmTime(2022, 01, 01, 0, 0, 0);
        rtc.writeSqwPinMode(DS3231_OFF);
        rtc.clearAlarm(1);
        rtc.clearAlarm(2);
        rtc.setAlarm1(alarmTime, DS3231_A1_Minute); /* Alarm when minutes and seconds match */
        rtc.disableAlarm(2);

        updateTemperature1 = true;
        updateTemperature2 = true;
        updateAnalogPinValue = true;
    }

    getTemperature();

    Serial.print("Reading analog pin value ... ");
    digitalWrite(PWR_FOR_ANALOG_READ_PIN, HIGH);
    delay(500);
    analogPinValue = roundTo5(analogRead(A0));
    analogPinValue = (analogPinValue >= 20) ? analogPinValue : 0;
    delay(200);
    digitalWrite(PWR_FOR_ANALOG_READ_PIN, LOW);
    Serial.println("[DONE]");

    Serial.print("Analog pin value is: ");
    Serial.println(analogPinValue);

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

    if (hour() < dailyUpdateAtXoclock)
    {
        rtcData.dailyForcedUpdateDone = false;
    }

    if (rtcData.numStarts == 1 || (hour() >= dailyUpdateAtXoclock && !rtcData.dailyForcedUpdateDone))
    {
        updateTemperature1 = true;
        updateTemperature2 = true;
        updateAnalogPinValue = true;
        updateEnergy = true;

        if (getEnergy())
        {
            saveEnergyToFilesystem();
        }
    }

    if (updateTemperature1 || updateTemperature2 || updateAnalogPinValue || updateEnergy || rtc.lostPower())
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

        if (!connectToMqttServer())
        {
            writeRtcMemory();
            blinkFailedAndSleep();
        }

        bool result = true;
        if (updateTemperature1)
        {
            if (sendDataToServer(TEMPERATURE1)) {
                rtcData.temperature1 = temperature1;
            }
            else {
                result = false;
            }
        }

        if (updateTemperature2)
        {
            if (sendDataToServer(TEMPERATURE2)) {
                rtcData.temperature2 = temperature2;
            }
            else {
                result = false;
            }
        }

        if (updateAnalogPinValue)
        {
            if (sendDataToServer(ANALOG_PIN_VALUE)) {
                rtcData.analogPinValue = analogPinValue;
            }
            else {
                result = false;
            }
        }

        if (updateEnergy)
        {
            if (!sendDataToServer(ENERGY)) {
                result = false;
            }
        }

        // sync time at first start, when RTC is not set and on first of each month
        if (rtcData.numStarts == 1 || rtc.lostPower() ||
            ((day() == 1) && (hour() == dailyUpdateAtXoclock)))
        {
            if (!syncTime())
            {
                result = false;
            }
        }

        if (result) {
            if (hour() >= dailyUpdateAtXoclock)
            {
                rtcData.dailyForcedUpdateDone = true;
            }
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
    if (isOtaUpdate)
    {
        static unsigned long previousMillis;
        if (millis() - previousMillis >= 100)
        {
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
                ESP.restart();
            }

            previousMillis = millis();
        }

        httpServer.handleClient();
        MDNS.update();
        delay(1);
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

bool syncTime(void)
{
    bool ret = false;
    Serial.print("Syncing time with NTP server ... ");

    timeClient.begin();
    if (timeClient.update())
    {
        rtc.adjust(DateTime(timeClient.getEpochTime()));
        setTime(timeClient.getEpochTime());

        Serial.println("[DONE]");
        ret = true;
    }
    else
    {
        Serial.println("[FAILED]");
    }

    return ret;
}

String prettyTime()
{
    char datebuffer[11], timebuffer[9];
    sprintf(datebuffer, "%02d.%02d.%04d", day(), month(), year());
    sprintf(timebuffer, "%02d:%02d:%02d", hour(), minute(), second());

    return String(datebuffer) + "-" + String(timebuffer);
}

bool getTemperature()
{
    bool ret = true;

    Serial.print("Collecting temperature data ... ");

    temperature1 = rtc.getTemperature() - 1; // measured offset

    lm75a.disableShutdownMode();
    delay(200);
    temperature2 = lm75a.readTemperatureC();
    lm75a.enableShutdownMode();

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

bool getEnergy()
{
// Interface zum Itron ACE3000 Type 260 Stromzähler
// https://wiki.volkszaehler.org/hardware/channels/meters/power/edl-ehz/itron_ace3000_type_260
// 300 Baud/s @ 7E1 = 34ms per character -> ~3.8 seconds for full response
/*
    example response (112 bytes):
    ^C^Z/?!
    /ACE0\3k260V01.18
    ^BF.F(00)
    C.1(1234567890123456)
    C.5.0(00)
    1.8.0(000285.4*kWh) <--- Verbrauch
    2.8.0(000120.1*kWh) <--- Einspeisung
    !
*/
    char receiveBuffer[128] = {0};
    bool ret = false;

    Serial.print("Collecting energy data ... ");

    //write "/?!"+CR+LF to init D0-Mode of the Itron ACE3000
    swSer.println(PSTR("/?!"));
    delay(200); // wait some time until response is available

    if (swSer.readBytes(receiveBuffer, sizeof(receiveBuffer) - 1) > 0)
    {
        if (char *pos = strstr(receiveBuffer, "1.8.0"))
        {
            pos += 6;
            energy = atof(pos);
            if (energy > 0)
            {
                ret = true;
            }
        }
    }

//    digitalWrite(LED_PIN, HIGH); //disable LED (pin is used by swSer)

    if (ret == true)
    {
        Serial.println("[DONE]");
    }
    else
    {
        Serial.println("[FAILED]");
    }

    // for debugging
//    Serial.println();
//    Serial.print(receiveBuffer);
//    Serial.println();

    Serial.print("Current energy count is: ");
    Serial.print(energy, 1);
    Serial.println("kWh");
    Serial.println();

    return ret;
}

bool saveEnergyToFilesystem()
{
    Serial.print("Writing energy data to LittleFS... ");

    if(!LittleFS.begin())
    {
        Serial.println("[FAILED]: An Error has occurred while mounting LittleFS");

        return false;
    }

    File file = LittleFS.open("/energy.txt", "a");
    if(!file)
    {
        Serial.println("[FAILED]: Failed to open file for appending");
        return false;
    }

    // file entry is dd.mm.yyyy-hh:mm:ss,vvvvvv.v
    char textBuffer[32] = {0};
    sprintf(textBuffer, prettyTime().c_str());
    textBuffer[19] = ',';
    sprintf(&textBuffer[20], "%08.1f", energy);

    file.println(textBuffer);
    file.close();
    LittleFS.end();

    Serial.println("[DONE]");
    return true;
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

        case ENERGY:
            sprintf(feed, "%s%02x_%s", MQTT_USERNAME"/feeds/", mac[5], "energy");
            sprintf(attributes, "%06.1f", energy);
            Serial.print("Sending energy value to server ... ");
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

void startOtaUpdateModus()
{
    memset(&rtcData, 0xff, sizeof(rtcData));
    ESP.rtcUserMemoryWrite(0, (uint32_t*) &rtcData, sizeof(rtcData));

    WiFi.softAP(otaSsid, otaPassword);

    MDNS.begin(otaHost);

    httpUpdater.setup(&httpServer);
    httpServer.begin();
    httpServer.on("/", handleRoot);
    httpServer.onNotFound([]() { // If the client requests any URI
        if (!handleFileRead(httpServer.uri())) // send it if it exists
            httpServer.send(404, "text/plain", "404: Not Found"); // otherwise, respond with a 404 (Not Found) error
    });

    MDNS.addService("http", "tcp", 80);
    Serial.println("HTTPUpdateServer ready!");
    Serial.printf("Connect to Wifi %s, password %s and\n", otaSsid, otaPassword);
    Serial.printf("open http://%s.local/update in your browser\n", otaHost);

    isOtaUpdate = true;
}

String getContentType(String filename)
{
    if(filename.endsWith(".htm")) return "text/html";
    else if(filename.endsWith(".html")) return "text/html";
    else if(filename.endsWith(".css")) return "text/css";
    else if(filename.endsWith(".jpg")) return "image/jpeg";
    return "text/plain";
}

bool handleFileRead(String path)
{
    Serial.println("handleFileRead: " + path);

    if (path.endsWith("/")) path += "index.html"; // If a folder is requested, send the index file
    String contentType = getContentType(path);

    if(LittleFS.begin())
    {
        if (LittleFS.exists(path))
        {
            File file = LittleFS.open(path, "r");
            httpServer.streamFile(file, contentType);

            file.close();
            return true;
        }

        Serial.println("\tFile Not Found");
        return false;
    }

    Serial.println(F("An Error has occurred while mounting LittleFS"));
    return false;
}

void handleRoot()
{
    FSInfo fs_info;
    LittleFS.info(fs_info);
    String buffer;
    buffer = FPSTR(index_html_top);

    if (!LittleFS.begin())
    {
        Serial.println(F("An Error has occurred while mounting LittleFS"));
    }

    Dir dir = LittleFS.openDir("/");

    while (dir.next())
    {
        buffer += "<a href ='" + dir.fileName() + "'>";
        buffer += dir.fileName() + "</a> " + String(dir.fileSize()) + " Bytes<br>\r\n";
    }

    buffer += "<br><br>Filesystem info:<br>";
    buffer += "================<br>";
    buffer += "totalBytes: " + String(fs_info.totalBytes) + "<br>";
    buffer += "usedBytes: " + String(fs_info.usedBytes) + "<br>";
    buffer += "blockSize: " + String(fs_info.blockSize) + "<br>";
    buffer += "pageSize: " + String(fs_info.pageSize) + "<br>";
    buffer += "maxOpenFiles: " + String(fs_info.maxOpenFiles) + "<br>";
    buffer += "maxPathLength: " + String(fs_info.maxPathLength) + "<br>";
    buffer += "</pre></body></html>\r\n";
    httpServer.send(200, "text/html", buffer);
}

int roundTo5(int in)
{
    const signed char round5delta[5] = {0, -1, -2, 2, 1};  // difference to the "rounded to nearest 5" value
    return in + round5delta[in%5];
}

// pattern ".."
void shortBlink()
{
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);
    delay(50);
}

// pattern "- -"
void blinkSuccessAndSleep()
{
    Serial.print("[DEEP.SLEEP]");
    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);

    ESP.deepSleep(0);
    delay(100);
}

// pattern "- ..."
void blinkFailedAndSleep()
{
    Serial.print("[DEEP.SLEEP]");
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

    ESP.deepSleep(0);
    delay(100);
}

// pattern ".."
void blinkNoUpdateAndSleep()
{
    Serial.print("[DEEP.SLEEP]");
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);

    ESP.deepSleep(0);
    delay(100);
}
