#include <Arduino.h>
#include <SPI.h>

#define MEASURE_TEMPERATURE
//#define MEASURE_HUMIDITY
#define MEASURE_BATTERY // check voltage on analog pin. immediately go to sleep if voltage is <= 3.0V
#define SERIAL_DEBUG // enable serial debug output and LED blinking
//#define SLOW_UPDATE
// https://www.element14.com/community/people/neilk/blog/2019/02/14/investigating-the-power-consumption-of-a-wemos-d1-mini-esp8266
#define LED_PIN     D4
#define LED2_PIN    D6

#include <Adafruit_SHT31.h>
#include <Adafruit_HTU21DF.h>

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <PubSubClient.h>

// required for OTA update
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>

#include "credentials.h"

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
  uint32_t sensorType;
  uint32_t numStarts; // counts the number of starts
  uint32_t lastUpdate; // contains the start number when the last update was sent
  float temperature; // contains the temperature that was sent at the last update
  float humidity; // contains the humidity that was sent at the last update
  float voltage; // contains the voltage that was sent at the last update
  byte data[512 - 8*4]; // nothing yet
} rtcData;

float temperature;
float humidity;
float voltage;

#ifdef SLOW_UPDATE
const unsigned int updateInterval = 7200; // in seconds (2 hours)
const unsigned int maxUpdateInterval = 12; // if value does not change, update at least every x's time (24 hours)
const float minTemperatureDifference = 1; // only update server if newTemp > oldTemp +- diff
const float minHumidityDifference = 5;
const float minVoltageDifference = 0.1;
#else
const unsigned int updateInterval = 300; // in seconds (5 minutes)
const unsigned int maxUpdateInterval = 72; // if value does not change, update at least every x's time (6 hours)
const float minTemperatureDifference = 0.5; // only update server if newTemp > oldTemp +- diff
const float minHumidityDifference = 2;
const float minVoltageDifference = 0.1;
#endif
uint8_t mac[6];

typedef enum {
    TEMPERATURE = 1,
    HUMIDITY,
    VOLTAGE
} datatype;

typedef enum {
    NONE = 0,
    SHT31,
    HTU21
} sensortype;

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

// Initialize SHT31 sensor
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// Initialize HTU21D sensor
Adafruit_HTU21DF htu21 = Adafruit_HTU21DF();

// function prototypes
bool readRtcMemory();
bool writeRtcMemory();
bool getTemperatureAndHumidity();
bool sendDataToServer(datatype which);
bool InitWiFi();
bool connectToMqttServer();
void shortBlink();
void blinkSuccessAndSleep();
void blinkFailedAndSleep();
void blinkNoUpdateAndSleep();

void setup()
{
    WiFi.disconnect(true);

    digitalWrite(LED_PIN, HIGH);
    digitalWrite(LED2_PIN, HIGH);
    pinMode(LED_PIN, OUTPUT); // LED
    pinMode(LED2_PIN, OUTPUT); // LED2
    pinMode(D0, WAKEUP_PULLUP); // reset after deep sleep
    pinMode(D7, INPUT_PULLUP); // if low during boot -> ota update

    Serial.begin(115200);
    delay(100);
    Serial.println();

#ifdef MEASURE_BATTERY
    int sensorValue = analogRead(A0);
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 4.2V):
    voltage = sensorValue * (4.163 / 1023.0);

    Serial.print("Voltage is: ");
    Serial.print(voltage, 1);
    Serial.println("V");

    if (voltage <= 3.0) {
        blinkNoUpdateAndSleep();
    }
#endif

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

    bool updateTemperature = false;
    bool updateHumidity = false;
    bool updateVoltage = false;

    if (readRtcMemory()) // CRC if rtcData is valid
    {
        rtcData.numStarts++;
        Serial.print("Started ");
        Serial.print(rtcData.numStarts);
        Serial.println(" times");
    }
    else // CRC of rtcData is invalid
    {
        rtcData.numStarts = 1;
        rtcData.lastUpdate = 1;
        rtcData.sensorType = NONE;
        Serial.println("First start");

#ifdef MEASURE_TEMPERATURE
        updateTemperature = true;
#endif
#ifdef MEASURE_HUMIDITY
        updateHumidity = true;
#endif
#ifdef MEASURE_BATTERY
        updateVoltage = true;
#endif
    }

    if (rtcData.numStarts - rtcData.lastUpdate >= maxUpdateInterval)
    {
        Serial.print("no update since ");
        Serial.print(maxUpdateInterval);
        Serial.println(" starts");

#ifdef MEASURE_TEMPERATURE
        updateTemperature = true;
#endif
#ifdef MEASURE_HUMIDITY
        updateHumidity = true;
#endif
#ifdef MEASURE_BATTERY
        updateVoltage = true;
#endif
    }

    if (rtcData.sensorType == SHT31) {
        sht31.begin(0x45);
    }
    else if (rtcData.sensorType == HTU21) {
        htu21.begin();
    }
    else {
        if (sht31.begin(0x45)) {
            Serial.println("Detected SHT31 sensor");
            rtcData.sensorType = SHT31;
        }
        else if (htu21.begin()) {
            Serial.println("Detected HTU21 sensor");
            rtcData.sensorType = HTU21;
        }
        else {
            Serial.println("No sensor detected");
            rtcData.sensorType = NONE;
            blinkFailedAndSleep();
        }
    }

    getTemperatureAndHumidity();

#ifdef MEASURE_TEMPERATURE
    if ((temperature < (rtcData.temperature - minTemperatureDifference)) || \
        (temperature > (rtcData.temperature + minTemperatureDifference)))
    {
        Serial.println("temperature changed");
        updateTemperature = true;
    }
#endif

#ifdef MEASURE_HUMIDITY
    if ((humidity < (rtcData.humidity - minHumidityDifference)) || \
        (humidity > (rtcData.humidity + minHumidityDifference)))
    {
        Serial.println("humidity changed");
        updateHumidity = true;
    }
#endif

#ifdef MEASURE_BATTERY
    if ((voltage < (rtcData.voltage - minVoltageDifference)) || \
        (voltage > (rtcData.voltage + minVoltageDifference)))
    {
        Serial.println("voltage changed");
        updateVoltage = true;
    }
#endif

    if (updateTemperature || updateHumidity || updateVoltage)
    {
        if (! InitWiFi())
        {
            writeRtcMemory();
            blinkFailedAndSleep();
        }

        if (!connectToMqttServer())
        {
            writeRtcMemory();
            blinkFailedAndSleep();
        }

        bool result = false;
        if (updateTemperature)
            result = sendDataToServer(TEMPERATURE);

        if (updateHumidity)
            result &= sendDataToServer(HUMIDITY);

        if (updateVoltage)
            result &= sendDataToServer(VOLTAGE);

        if (result) {
            mqttclient.loop();

            if (updateTemperature)
                rtcData.temperature = temperature;

            if (updateHumidity)
                rtcData.humidity = humidity;

            if (updateVoltage)
                rtcData.voltage = voltage;

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
            Serial.printf("Going to sleep in %d seconds\n", otaTimeout/10);
        }

        if (otaTimeout > 60) {
            if (otaTimeout-- % 10 == 0) {
                digitalWrite(LED2_PIN, LOW);
                delay(50);
                digitalWrite(LED2_PIN, HIGH);
                delay(50);
            }
        }
        else {
            if (otaTimeout-- % 5 == 0) {
                digitalWrite(LED2_PIN, LOW);
                delay(50);
                digitalWrite(LED2_PIN, HIGH);
                delay(50);
            }
        }

        if (otaTimeout <= 0) {
            blinkNoUpdateAndSleep();
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
    // Update CRC32 of data
    rtcData.crc32 = calculateCRC32((uint8_t*) &rtcData.dummydata, sizeof(rtcData) - 4);
    // Write struct to RTC memory
    if (ESP.rtcUserMemoryWrite(0, (uint32_t*) &rtcData, sizeof(rtcData)))
    {
        return true;
    }

    return false;
}

bool getTemperatureAndHumidity()
{
#ifdef MEASURE_TEMPERATURE
    if (rtcData.sensorType == SHT31) {
        temperature = sht31.readTemperature();
    }
    else if (rtcData.sensorType == HTU21) {
        temperature = htu21.readTemperature();
    }
#endif

#ifdef MEASURE_HUMIDITY
    if (rtcData.sensorType == SHT31) {
        humidity = sht31.readHumidity();
    }
    else if (rtcData.sensorType == HTU21) {
        humidity = htu21.readHumidity();
    }
#endif

    bool ret = true;

    Serial.print("Collecting temperature and humidity data ... ");

#ifdef MEASURE_TEMPERATURE
    if (isnan(temperature)) {  // check if 'is not a number'
        ret = false;
    }
#endif

#ifdef MEASURE_HUMIDITY
    if (isnan(humidity)) {  // check if 'is not a number'
        ret = false;
    }
#endif

    if (ret == true)
    {
        Serial.println("[DONE]");
    }
    else
    {
        Serial.println("[FAILED]");
    }

#ifdef MEASURE_TEMPERATURE
    Serial.print("Temperature is: ");
    Serial.print(temperature, 1);
    Serial.println("°C");
#endif

#ifdef MEASURE_HUMIDITY
    Serial.print("Humidity is: ");
    Serial.print(humidity, 1);
    Serial.println("%");
#endif

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
        case TEMPERATURE:
            sprintf(feed, "%s%02x_%s", MQTT_USERNAME"/feeds/", mac[5], "temperature");
            sprintf(attributes, "%02.1f", temperature);
            Serial.print("Sending temperature to server ... ");
            break;

        case HUMIDITY:
            sprintf(feed, "%s%02x_%s", MQTT_USERNAME"/feeds/", mac[5], "humidity");
            sprintf(attributes, "%02.1f", humidity);
            Serial.print("Sending humidity to server ... ");
            break;

        case VOLTAGE:
            sprintf(feed, "%s%02x_%s", MQTT_USERNAME"/feeds/", mac[5], "voltage");
            sprintf(attributes, "%01.2f", voltage);
            Serial.print("Sending voltage to server ... ");
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

    return ret;
}

bool InitWiFi()
{
    int i = 0;

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // attempt to connect to WiFi network
    while (WiFi.status() != WL_CONNECTED) {
        Serial.println();
        Serial.print("Connecting to SSID ");
        Serial.print(WIFI_SSID);
        Serial.print(" ... ");
        delay(1000);
        if (i++ > 5)
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
