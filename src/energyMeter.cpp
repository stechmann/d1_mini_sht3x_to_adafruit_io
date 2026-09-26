#include <Arduino.h>
#include <SoftwareSerial.h>

#include "globalDefines.h"

extern float energy;

SoftwareSerial swSer;

void initSwSerial()
{
    swSer.begin(300, SWSERIAL_7E1, ENERGY_METER_RX_PIN, ENERGY_METER_TX_PIN);
    swSer.setTimeout(2500);
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

    // write "/?!"+CR+LF to init D0-Mode of the Itron ACE3000
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

    if (ret == true)
    {
        Serial.println("[DONE]");
    }
    else
    {
        Serial.println("[FAILED]");
    }

    Serial.print("Current energy count is: ");
    Serial.print(energy, 1);
    Serial.println("kWh");
    Serial.println();

    return ret;
}
