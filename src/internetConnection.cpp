#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266Ping.h>
#include <PubSubClient.h>

#include "credentials.h"
#include "globalDefines.h"

extern int32_t temperature1;
extern int32_t temperature2;
extern int32_t analogPinValue;
extern float energy;

WiFiClient espClient;
PubSubClient mqttclient(espClient);

const uint32_t wifiTimeoutMs = 80000;
const uint32_t pingTimeoutMs = 60000;
const uint32_t mqttTimeoutMs = 30000;
const uint32_t ackTimeoutMs = 10000;

bool sendDataToServer(datatype which)
{
    bool ret = false;
    char feed[64];
    char attributes[16];

    switch (which)
    {
    case TEMPERATURE1:
        snprintf(feed, sizeof(feed), "%s/feeds/temperature1", MQTT_USERNAME);
        snprintf(attributes, sizeof(attributes), "%02d", temperature1);
        Serial.print("Sending temperature 1 to server ... ");
        break;

    case TEMPERATURE2:
        snprintf(feed, sizeof(feed), "%s/feeds/temperature2", MQTT_USERNAME);
        snprintf(attributes, sizeof(attributes), "%02d", temperature2);
        Serial.print("Sending temperature 2 to server ... ");
        break;

    case ANALOG_PIN_VALUE:
        snprintf(feed, sizeof(feed), "%s/feeds/analog", MQTT_USERNAME);
        snprintf(attributes, sizeof(attributes), "%04d", analogPinValue);
        Serial.print("Sending analog pin value to server ... ");
        break;

    case ENERGY:
        snprintf(feed, sizeof(feed), "%s/feeds/energy", MQTT_USERNAME);
        snprintf(attributes, sizeof(attributes), "%06.1f", energy);
        Serial.print("Sending energy value to server ... ");
        break;
    }
    // publish() only copies the data into the TCP buffer. Wait until the server has acknowledged
    // it, otherwise it may be lost when the AP is powered off during deep sleep
    if (mqttclient.publish(feed, attributes) && espClient.flush(ackTimeoutMs))
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
    // enable power for AP
    pinMode(PWR_FOR_WIFI_PIN, OUTPUT);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("Connecting to SSID ");
    Serial.print(WIFI_SSID);
    Serial.print(" ...");

    const uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < wifiTimeoutMs)
    {
        Serial.print(".");
        delay(1000);
    }
    Serial.print(" ");

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
    Serial.print("Pinging ");
    Serial.print(host);
    Serial.print(" ...");

    // a single echo request per attempt (the library default is 5, taking about 5 seconds)
    const uint32_t start = millis();
    while (millis() - start < pingTimeoutMs)
    {
        if (Ping.ping(host, 1))
        {
            Serial.println(" [DONE]");
            return true;
        }
        Serial.print(".");
        delay(1000);
    }

    Serial.println(" [FAILED]");
    return false;
}

bool connectToMqttServer()
{
    mqttclient.setServer(SERVER, 1883);

    const uint32_t start = millis();
    while (true)
    {
        Serial.print("Connecting to ");
        Serial.print(SERVER);
        Serial.print(" ... ");
        // Attempt to connect (clientId, username, password)
        if (mqttclient.connect("Wemos D1", MQTT_USERNAME, MQTT_KEY))
        {
            Serial.println("[DONE]");
            return true;
        }

        Serial.print("[FAILED] [ rc = ");
        Serial.print(mqttclient.state());
        Serial.println(" ]");

        if (millis() - start >= mqttTimeoutMs)
        {
            return false;
        }
        delay(5000);
    }
}

void disconnectFromServer()
{
    // sends MQTT DISCONNECT and stops the TCP client (waits at most 300 ms for pending acks)
    mqttclient.disconnect();
}