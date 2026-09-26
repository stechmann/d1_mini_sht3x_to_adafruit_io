#include <Arduino.h>

#include <NTPClient.h>
#include <Timezone.h>
#include <RTClib.h>

#include <Temperature_LM75_Derived.h>

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include "globalDefines.h"

#define SERIAL_DEBUG
#ifndef SERIAL_DEBUG
// disable Serial output
#define Serial SomeOtherwiseUnusedName
static class
{
public:
    void begin(...) {}
    void print(...) {}
    void println(...) {}
    void printf(...) {}
} Serial;
#endif

// public variables
RtcData rtcData;

int32_t temperature1;
int32_t temperature2;
int32_t analogPinValue;
float energy;

const IPAddress google_host(8, 8, 8, 8); // for testing online status
const int dailyUpdateAtXoclock = 6;
const float minTemperatureDifference = 2; // only update server if newTemp > oldTemp +- diff
const int32_t minAnalogPinValueDifference = 20;

// Sensors
Generic_LM75 lm75a;

// RTC
RTC_DS3231 rtc;

// NTP
WiFiUDP ntpUDP;
#define NTP_SERVER "de.pool.ntp.org"
NTPClient timeClient(ntpUDP, NTP_SERVER);

TimeChangeRule summertime = {"CEST", Last, Sun, Mar, 2, 120}; // Daylight time = UTC + 2 hours
TimeChangeRule wintertime = {"CET", Last, Sun, Oct, 3, 60};   // Standard time = UTC + 1 hours
Timezone myTZ(summertime, wintertime);

static int32_t currentDate()
{
    return year() * 10000 + month() * 100 + day();
}

static int32_t currentMonth()
{
    return year() * 100 + month();
}

static bool i2cDevicePresent(uint8_t address)
{
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

enum class RunResult
{
    NoUpdate,
    Success,
    Failed
};

// single exit point of every run: does not return, the cpu goes to deep sleep
static void finish(RunResult result)
{
    disconnectFromServer();
    writeRtcMemory();

    switch (result)
    {
    case RunResult::Success:
        blinkSuccessAndSleep();
        break;
    case RunResult::Failed:
        blinkFailedAndSleep();
        break;
    case RunResult::NoUpdate:
        blinkNoUpdateAndSleep();
        break;
    }
}

void setup()
{
    bool updateTemperature1 = false;
    bool updateTemperature2 = false;
    bool updateAnalogPinValue = false;
    bool updateEnergy = false;
    bool energyOk = true;

    // the modem is off at boot (core 3.x) and is woken by WiFi.mode() when needed.
    // do not store WiFi config in flash on every connect
    WiFi.persistent(false);

    digitalWrite(LED_PIN, HIGH);                 // active low. disable LED
    digitalWrite(PWR_FOR_ANALOG_READ_PIN, LOW);  // disable analog measurement. only enable power during measurement
    digitalWrite(PWR_FOR_WIFI_PIN, HIGH);        // there is an external pulldown resistor on this pin
    pinMode(LED_PIN, OUTPUT);                    // LED
    pinMode(PWR_FOR_ANALOG_READ_PIN, OUTPUT);    // enable output
    pinMode(PWR_FOR_WIFI_PIN, INPUT);            // it is configured as input by default. it is pulled down by external resistor
    pinMode(D0, WAKEUP_PULLUP);                  // reset after deep sleep

    initSwSerial();

    Serial.begin(115200);
    delay(500);
    Serial.println();

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    // read time from external RTC
    if (!rtc.begin())
    {
        Serial.println("Couldn't find RTC");
    }
    rtc.clearAlarm(1); // assume we were woken up by RTC alarm -> disable alarm

    time_t utc = rtc.now().unixtime();
    setTime(myTZ.toLocal(utc));
    if (rtc.lostPower())
    {
        Serial.println("RTC lost power. No valid time.");
    }
    else
    {
        Serial.print("Start at ");
        Serial.println(prettyTime());
    }

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
        rtcData.lastDailyUpdate = 0;
        rtcData.lastTimeSync = 0;
        Serial.println("First start");

        // initialize external RTC at first start
        rtc.disable32K();

        DateTime alarmTime(2026, 01, 01, 0, 0, 0);
        rtc.writeSqwPinMode(DS3231_OFF);
        rtc.clearAlarm(1);
        rtc.clearAlarm(2);
        rtc.setAlarm1(alarmTime, DS3231_A1_Minute); /* Alarm when minutes and seconds match */
        rtc.disableAlarm(2);

        updateTemperature1 = true;
        updateTemperature2 = true;
        updateAnalogPinValue = true;
    }

    const bool temperature1Ok = getTemperature1();
    const bool temperature2Ok = getTemperature2();
    const bool temperatureOk = temperature1Ok && temperature2Ok;

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

    if (temperature1Ok &&
        ((temperature1 <= (rtcData.temperature1 - minTemperatureDifference)) ||
         (temperature1 >= (rtcData.temperature1 + minTemperatureDifference))))
    {
        Serial.print("Temperature 1 changed from ");
        Serial.print(rtcData.temperature1);
        Serial.print("°C to ");
        Serial.print(temperature1);
        Serial.println("°C");
        updateTemperature1 = true;
    }

    if (temperature2Ok &&
        ((temperature2 <= (rtcData.temperature2 - minTemperatureDifference)) ||
         (temperature2 >= (rtcData.temperature2 + minTemperatureDifference))))
    {
        Serial.print("Temperature 2 changed from ");
        Serial.print(rtcData.temperature2);
        Serial.print("°C to ");
        Serial.print(temperature2);
        Serial.println("°C");
        updateTemperature2 = true;
    }

    if ((analogPinValue <= (rtcData.analogPinValue - minAnalogPinValueDifference)) ||
        (analogPinValue >= (rtcData.analogPinValue + minAnalogPinValueDifference)))
    {
        Serial.print("analog pin value changed from ");
        Serial.print(rtcData.analogPinValue);
        Serial.print(" to ");
        Serial.println(analogPinValue);

        updateAnalogPinValue = true;
    }

    int32_t today = currentDate();
    const bool dailyUpdateDue = hour() >= dailyUpdateAtXoclock && rtcData.lastDailyUpdate != today;
    // at first start, when RTC is not set and once a month (retried until successful)
    const bool timeSyncDue = rtcData.numStarts == 1 || rtc.lostPower() ||
                             (hour() >= dailyUpdateAtXoclock && rtcData.lastTimeSync != currentMonth());

    if (rtcData.numStarts == 1 || dailyUpdateDue)
    {
        updateTemperature1 = true;
        updateTemperature2 = true;
        updateAnalogPinValue = true;
        updateEnergy = true;

        energyOk = getEnergy();
    }

    // only upload values of sensors that could be read
    if (!temperature1Ok)
    {
        updateTemperature1 = false;
    }
    if (!temperature2Ok)
    {
        updateTemperature2 = false;
    }

    if (updateTemperature1 || updateTemperature2 || updateAnalogPinValue || updateEnergy || timeSyncDue)
    {
        if (!initWiFi())
        {
            return finish(RunResult::Failed);
        }

        // the AP needs about 50 seconds after power on until the internet is reachable
        if (!pingHost(google_host))
        {
            return finish(RunResult::Failed);
        }

        bool timeSyncOk = true;
        if (timeSyncDue)
        {
            timeSyncOk = syncTime();
            if (timeSyncOk)
            {
                rtcData.lastTimeSync = currentMonth();
                today = currentDate(); // time may have been invalid before the sync
            }
        }

        if (!connectToMqttServer())
        {
            return finish(RunResult::Failed);
        }

        bool result = true;
        if (updateTemperature1)
        {
            if (sendDataToServer(TEMPERATURE1))
            {
                rtcData.temperature1 = temperature1;
            }
            else
            {
                result = false;
            }
        }

        if (updateTemperature2)
        {
            if (sendDataToServer(TEMPERATURE2))
            {
                rtcData.temperature2 = temperature2;
            }
            else
            {
                result = false;
            }
        }

        if (updateAnalogPinValue)
        {
            if (sendDataToServer(ANALOG_PIN_VALUE))
            {
                rtcData.analogPinValue = analogPinValue;
            }
            else
            {
                result = false;
            }
        }

        if (updateEnergy)
        {
            if (!energyOk || !sendDataToServer(ENERGY))
            {
                result = false;
            }
        }

        if (result && updateEnergy && hour() >= dailyUpdateAtXoclock)
        {
            rtcData.lastDailyUpdate = today;
        }

        return finish((result && temperatureOk && timeSyncOk) ? RunResult::Success : RunResult::Failed);
    }

    finish(temperatureOk ? RunResult::NoUpdate : RunResult::Failed);
}

void loop()
{
}

bool syncTime(void)
{
    bool ret = false;
    Serial.print("Syncing time with NTP server ... ");

    timeClient.begin();
    if (timeClient.update())
    {
        rtc.adjust(DateTime(timeClient.getEpochTime()));
        setTime(myTZ.toLocal(timeClient.getEpochTime()));

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

// operating range of both sensors; written this way to reject NaN as well
static bool temperatureValid(float t)
{
    return t >= -40 && t <= 85;
}

static void printTemperature(const char *name, bool ok, int32_t value)
{
    Serial.print(name);
    Serial.print(" is: ");
    if (ok)
    {
        Serial.print(value);
        Serial.println("°C");
    }
    else
    {
        Serial.println("[FAILED]");
    }
}

// DS3231 (RTC)
bool getTemperature1()
{
    bool ret = false;

    // the libraries do not report bus errors, so check that the sensor responds
    if (i2cDevicePresent(DS3231_I2C_ADDRESS))
    {
        float t = rtc.getTemperature() - 1; // measured offset
        if (temperatureValid(t))
        {
            temperature1 = lroundf(t);
            ret = true;
        }
    }

    printTemperature("Temperature 1", ret, temperature1);
    return ret;
}

// LM75A
bool getTemperature2()
{
    bool ret = false;

    // the libraries do not report bus errors, so check that the sensor responds
    if (i2cDevicePresent(LM75A_I2C_ADDRESS))
    {
        lm75a.disableShutdownMode();
        delay(200);
        float t = lm75a.readTemperatureC();
        lm75a.enableShutdownMode();
        if (temperatureValid(t))
        {
            temperature2 = lroundf(t);
            ret = true;
        }
    }

    printTemperature("Temperature 2", ret, temperature2);
    return ret;
}

int roundTo5(int in)
{
    return (in + 2) / 5 * 5; // round to nearest multiple of 5
}
