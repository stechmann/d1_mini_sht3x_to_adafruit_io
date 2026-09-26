#include <ESP8266WiFi.h>

#define I2C_SDA_PIN D1
#define I2C_SCL_PIN D2
#define ENERGY_METER_RX_PIN D3
#define ENERGY_METER_TX_PIN D4
#define LED_PIN D4
#define PWR_FOR_ANALOG_READ_PIN D5
#define PWR_FOR_WIFI_PIN D6

#define LM75A_I2C_ADDRESS 0x48
#define DS3231_I2C_ADDRESS 0x68

typedef enum
{
    TEMPERATURE1 = 1,
    TEMPERATURE2,
    ANALOG_PIN_VALUE,
    ENERGY
} datatype;

struct RtcData
{
    uint32_t crc32;

    uint32_t dummydata;
    uint32_t numStarts;            // counts the number of starts
    int32_t lastDailyUpdate;       // yyyymmdd of the last successful daily update
    int32_t temperature1;          // contains the temperature that was sent at the last update
    int32_t temperature2;          // contains the temperature that was sent at the last update
    int32_t analogPinValue;        // contains the analog pin value that was sent at the last update
    int32_t lastTimeSync;          // yyyymm of the last successful NTP sync
    byte data[512 - 8 * 4];        // nothing yet
};

// function prototypes
bool readRtcMemory();
bool writeRtcMemory();
bool syncTime();
String prettyTime();
bool getTemperature1();
bool getTemperature2();
void initSwSerial();
bool getEnergy();
bool sendDataToServer(datatype which);
bool initWiFi();
bool pingHost(IPAddress host);
bool connectToMqttServer();
void disconnectFromServer();
int roundTo5(int in);
void shortBlink();
void blinkSuccessAndSleep();
void blinkFailedAndSleep();
void blinkNoUpdateAndSleep();