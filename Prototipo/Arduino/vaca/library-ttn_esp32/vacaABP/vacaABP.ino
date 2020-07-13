#include <TTN_esp32.h>

#include "TTN_CayenneLPP.h"
/***************************************************************************
 *  Go to your TTN console register a device then the copy fields
 *  and replace the CHANGE_ME strings below
 ****************************************************************************/
const char *devAddr = "26011853";
const char *nwkSKey = "14BF29C0EEF2A895937D7463C4668384";
const char *appSKey = "54F6B2D6CDF392948FDCB9C979ADAB49";

TTN_esp32 ttn ;
TTN_CayenneLPP lpp;

void message(const uint8_t* payload, size_t size, int rssi)
{
    Serial.println("-- MESSAGE");
    Serial.print("Received " + String(size) + " bytes RSSI= " + String(rssi) + "dB");

    for (int i = 0; i < size; i++)
    {
        Serial.print(" " + String(payload[i]));
        // Serial.write(payload[i]);
    }

    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting");
    ttn.begin();
    ttn.onMessage(message); // declare callback function when is downlink from server
    ttn.personalize(devAddr, nwkSKey, appSKey);
    ttn.showStatus();
}

void loop()
{
    static float nb = 18.2;
    nb += 0.1;
    lpp.reset();
    lpp.addTemperature(1, nb);
    if (ttn.sendBytes(lpp.getBuffer(), lpp.getSize()))
    {
        Serial.printf("Temp: %f TTN_CayenneLPP: %d %x %02X%02X\n", nb, lpp.getBuffer()[0], lpp.getBuffer()[1],
            lpp.getBuffer()[2], lpp.getBuffer()[3]);
    }
    delay(10000);
}
