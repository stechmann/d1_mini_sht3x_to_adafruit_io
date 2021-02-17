#include <Arduino.h>
#include <SPI.h>

#include <Adafruit_SHT31.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <PubSubClient.h>

#include "credentials.h"

// public variables
struct {
  uint32_t crc32;

  uint32_t dummydata;
  uint32_t numStarts; // counts the number of starts
  uint32_t lastUpdate; // contains the start number when the last update was sent
  float temperature; // contains the temperature that was sent at the last update
  float humidity; // contains the humidity that was sent at the last update
  byte data[512 - 6*8]; // nothing yet
} rtcData;

float temperature;
float humidity;

const unsigned int updateInterval = 300; // in seconds
const unsigned int maxUpdateInterval = 72; // if value does not change, update at least every x's time
uint8_t mac[6];

// Initialize the WiFi and MQTT client object
WiFiClient espClient;
PubSubClient mqttclient(espClient);

// Initialize SHT31 sensor
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// function prototypes
bool readRtcMemory();
bool writeRtcMemory();
bool getTemperatureAndHumidity();
bool sendData();
bool InitWiFi();
bool connect();
bool getPublicIP();
void blinkSuccessAndSleep();
void blinkFailedAndSleep();
void blinkNoUpdateAndSleep();

void setup()
{
    bool update = false;

    digitalWrite(D4, HIGH);
    pinMode(D4, OUTPUT); // LED
    pinMode(D0, WAKEUP_PULLUP);

    Serial.begin(115200);
    delay(100);
    Serial.println();

    if (! sht31.begin(0x45)) {
        Serial.println("Couldn't find SHT31");
        blinkFailedAndSleep();
    }

    getTemperatureAndHumidity();

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
        Serial.println("First start");
        update = true;
    }

    if (rtcData.lastUpdate - rtcData.numStarts >= maxUpdateInterval)
    {
        Serial.print("no update since ");
        Serial.print(maxUpdateInterval);
        Serial.println(" starts");
        update = true;
    }

    if ((temperature < (rtcData.temperature - 0.5)) || \
        (temperature > (rtcData.temperature + 0.5)))
    {
        Serial.println("temperature changed");
        update = true;
    }

    if ((humidity < (rtcData.humidity - 0.5)) || \
        (humidity > (rtcData.humidity + 0.5)))
    {
        Serial.println("humidity changed");
        update = true;
    }

    if (update)
    {
        rtcData.temperature = temperature;
        rtcData.humidity = humidity;
        rtcData.lastUpdate = rtcData.numStarts;
    }

    writeRtcMemory();

    if (update)
    {
        if (! InitWiFi())
        {
            blinkFailedAndSleep();
        }

    //  getPublicIP();

        mqttclient.setServer(SERVER, 1883);
        if (!connect())
        {
            blinkFailedAndSleep();
        }

        if (sendData())
        {
            mqttclient.loop();
            blinkSuccessAndSleep();
        }
        else
        {
            blinkFailedAndSleep();
        }
    }
    else
    {
        blinkNoUpdateAndSleep();
    }

    // do not put anything here, cpu will sleep and never reach this point
}

void loop()
{

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
    temperature = sht31.readTemperature();
    humidity = sht31.readHumidity();
    bool ret = true;

    Serial.print("Collecting temperature and humidity data ... ");

    if (isnan(temperature)) {  // check if 'is not a number'
        ret = false;
    }

    if (isnan(humidity)) {  // check if 'is not a number'
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

    Serial.print("Temperature is: ");
    Serial.print(temperature, 1);
    Serial.println("°C");

    Serial.print("Humidity is: ");
    Serial.print(humidity, 1);
    Serial.println("%");


    return ret;
}

bool sendData()
{
    bool ret = true;

    uint8_t mac[6];
    wifi_get_macaddr(STATION_IF, mac);

    char feed_temperature[64];
    sprintf(feed_temperature, "%s%02x_%s", MQTT_USERNAME"/feeds/", mac[5], "temperature");
    char feed_humidity[64];
    sprintf(feed_humidity, "%s%02x_%s", MQTT_USERNAME"/feeds/", mac[5], "humidity");

    // Send payload
    char attributes[10];
    sprintf(attributes, "%02.1f", temperature);
    Serial.print("Sending temperature to server ... ");

    if (mqttclient.publish(feed_temperature, attributes))
    {
        Serial.println("[DONE]");
    }
    else
    {
        Serial.println("[FAILED]");
    }

    sprintf(attributes, "%02.1f", humidity);
    Serial.print("Sending humidity to server ... ");

    if (mqttclient.publish(feed_humidity, attributes))
    {
        Serial.println("[DONE]");
    }
    else
    {
        Serial.println("[FAILED]");
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

bool connect()
{
    bool ret = false;
    int i = 0;

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

bool getPublicIP()
{
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(espClient, "http://api.ipify.org/");
        int httpCode = http.GET();
//        Serial.println(httpCode);
        if (httpCode == 200) {
            String payload = http.getString();
            IPAddress ipaddr;

            ipaddr.fromString(payload);
/*
            Serial.println(ipaddr[0]);
            Serial.println(ipaddr[1]);
            Serial.println(ipaddr[2]);
            Serial.println(ipaddr[3]);
*/
        }
        http.end();
        return true;
    }

    return false;
}

// pattern "- -"
void blinkSuccessAndSleep()
{
    Serial.print("Sleep for ");
    Serial.print(updateInterval);
    Serial.println(" seconds ...");
    digitalWrite(D4, LOW);
    delay(500);
    digitalWrite(D4, HIGH);
    delay(500);
    digitalWrite(D4, LOW);
    delay(500);
    digitalWrite(D4, HIGH);

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

    digitalWrite(D4, LOW);
    delay(500);
    digitalWrite(D4, HIGH);
    delay(500);
    digitalWrite(D4, LOW);
    delay(50);
    digitalWrite(D4, HIGH);
    delay(200);
    digitalWrite(D4, LOW);
    delay(50);
    digitalWrite(D4, HIGH);
    delay(200);
    digitalWrite(D4, LOW);
    delay(50);
    digitalWrite(D4, HIGH);  // sleep for x seconds

    ESP.deepSleep(updateInterval * 1e6);
    delay(100);
}

// pattern ".."
void blinkNoUpdateAndSleep()
{
    Serial.print("Sleep for ");
    Serial.print(updateInterval);
    Serial.println(" seconds ...");

    digitalWrite(D4, LOW);
    delay(50);
    digitalWrite(D4, HIGH);
    delay(200);
    digitalWrite(D4, LOW);
    delay(50);
    digitalWrite(D4, HIGH);

    ESP.deepSleep(updateInterval * 1e6);
    delay(100);
}
