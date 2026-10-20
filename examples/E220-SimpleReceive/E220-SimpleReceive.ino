/*
 * E220_SimpleReceive.ino
 *
 * Minimal EBYTE E220 receiver. Prints the struct sent by E220_SimpleSend.ino.
 *
 * Wiring (Teensy 3.2):
 *   E220 M0  -> pin 3
 *   E220 M1  -> pin 4
 *   E220 AUX -> pin 2
 *   E220 TXD -> pin 0 (RX1)
 *   E220 RXD -> pin 1 (TX1)
 */

#include <EBYTE-E220.h>

#define PIN_M0          3
#define PIN_M1          4
#define PIN_AUX         2

#define MY_ADDRESS      0x0002      // must match DEST_ADDRESS in the sender
#define RADIO_CHANNEL   18

EBYTE_E220_UART radio(&Serial1, PIN_M0, PIN_M1, PIN_AUX, E220_MODEL_900T22);

// Must be identical in the send and receive sketches
struct __attribute__((packed)) DataPacket {
    uint32_t packetId;
    float    temperature;
    float    humidity;
    int32_t  batteryMv;
};

DataPacket rxPacket;
uint8_t    rxBuffer[E220_MAX_PACKET_SIZE];

void setup()
{
    Serial.begin(115200);
    Serial1.begin(9600);

    if (radio.begin() != E220_SUCCESS) {
        Serial.println("E220 not responding");
        while (true) {
            delay(1000);
        }
    }

    // ----- Basic settings (uncomment to change, both modules must match) -----
    // radio.setAddress(MY_ADDRESS);
    // radio.setChannel(RADIO_CHANNEL);
    // radio.setAirDataRate(E220_AIR_2K4);
    // radio.setTransmitPower(E220_POWER_MAX);
    // radio.setTransmissionMethod(E220_TX_FIXED);
    // radio.saveParameters(E220_SAVE_PERMANENT);

    radio.printParameters(Serial);
    radio.setMode(E220_MODE_NORMAL);
}

void loop()
{
    if (radio.available()) {
        uint16_t rxLength = radio.receiveMessage(rxBuffer, sizeof(rxBuffer), 100);
        if (rxLength == sizeof(DataPacket)) {
            memcpy(&rxPacket, rxBuffer, sizeof(rxPacket));

            Serial.print("Received #");
            Serial.print(rxPacket.packetId);
            Serial.print("  temp=");
            Serial.print(rxPacket.temperature, 2);
            Serial.print("  hum=");
            Serial.print(rxPacket.humidity, 2);
            Serial.print("  batt=");
            Serial.println(rxPacket.batteryMv);
        }
    }
}
