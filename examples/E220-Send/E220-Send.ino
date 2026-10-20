/*
 * E220_Send.ino
 *
 * EBYTE E220 transmitter example.
 *
 * Shows how to:
 *   - program basic settings (address, channel, air data rate, power,
 *     fixed transmission, ambient RSSI) and save them permanently
 *   - read the product information block
 *   - read the ambient noise RSSI before transmitting
 *   - send a struct (uint32_t, float, float, int32_t) as a fixed (addressed)
 *     message to the receiver example
 *
 * Pair this sketch with E220_Receive.ino running on a second board.
 *
 * Default wiring (change the pin definitions below to suit your board):
 *
 *   E220        AVR (Uno/Nano)   ESP32           Other (Serial1)
 *   ----        --------------   -----           ---------------
 *   M0          D7               GPIO18          D7
 *   M1          D6               GPIO19          D6
 *   AUX         D5               GPIO4           D5
 *   TXD         D10 (SW RX)      GPIO16 (RX2)    RX1
 *   RXD         D11 (SW TX)      GPIO17 (TX2)    TX1
 *   VCC / GND   5V / GND         3V3-5V / GND    VCC / GND
 */

#include <EBYTE-E220.h>

// ---------------------------------------------------------------------------
// User settings
// ---------------------------------------------------------------------------
#define MY_ADDRESS          0x0001      // address of this (sending) module
#define DEST_ADDRESS        0x0002      // address of the receiving module
#define RADIO_CHANNEL       18          // 900 models: 850.125 + 18 = 868.125 MHz
                                        // 400 models: 410.125 + 23 = 433.125 MHz
#define MODULE_MODEL        E220_MODEL_900T22
#define SEND_INTERVAL_MS    5000UL

// ---------------------------------------------------------------------------
// Board specific serial port / pins
// ---------------------------------------------------------------------------
#if defined(ESP32)
  #define PIN_M0    18
  #define PIN_M1    19
  #define PIN_AUX   4
  #define PIN_RX    16
  #define PIN_TX    17
  #define E220_PORT Serial2
#elif defined(ARDUINO_ARCH_AVR)
  #include <SoftwareSerial.h>
  #define PIN_M0    7
  #define PIN_M1    6
  #define PIN_AUX   5
  #define PIN_RX    10
  #define PIN_TX    11
  SoftwareSerial softSerial(PIN_RX, PIN_TX);
  #define E220_PORT softSerial
#elif defined(TEENSYDUINO)
  #define PIN_M0    3
  #define PIN_M1    4
  #define PIN_AUX   2
  #define E220_PORT Serial1           // RX1 = pin 0, TX1 = pin 1
#else
  #define PIN_M0    7
  #define PIN_M1    6
  #define PIN_AUX   5
  #define E220_PORT Serial1
#endif

EBYTE_E220_UART radio(&E220_PORT, PIN_M0, PIN_M1, PIN_AUX, MODULE_MODEL);

// ---------------------------------------------------------------------------
// Data packet sent over the air - MUST be identical in E220_Send.ino and
// E220_Receive.ino.
// - packed      : no compiler padding, so the byte layout is the same on all MCUs
// - int32_t     : plain 'int' is 2 bytes on AVR but 4 bytes on ESP32/ARM
// - float       : 4 bytes on AVR, ESP32 and ARM (all little-endian)
// - syncWord    : fixed marker so a shifted or foreign packet is rejected
// - checksum    : 8-bit sum of all preceding bytes, detects corrupted data
// Total size = 2 + 4 + 4 + 4 + 4 + 1 = 19 bytes (must not exceed the sub packet size)
// ---------------------------------------------------------------------------
#define PACKET_SYNC_WORD    0xE220

struct __attribute__((packed)) SensorPacket {
    uint16_t syncWord;
    uint32_t packetId;
    float    temperature;
    float    humidity;
    int32_t  batteryMv;
    uint8_t  checksum;
};

// 8-bit sum of every byte in the packet except the checksum itself
uint8_t computeChecksum(const SensorPacket* packetPtr)
{
    const uint8_t* bytePtr = (const uint8_t*)packetPtr;
    uint8_t sumValue = 0;
    for (uint8_t byteIndex = 0; byteIndex < sizeof(SensorPacket) - 1; byteIndex++) {
        sumValue += bytePtr[byteIndex];
    }
    return sumValue;
}

SensorPacket txPacket;

uint32_t lastSendTime = 0;
uint16_t msgCounter   = 0;

// Re-opens the host serial port when the library needs a different baud rate
// (configuration mode always runs at 9600 8N1).
void openSerialPort(uint32_t baudRate, uint8_t parityMode)
{
#if defined(ESP32)
    uint32_t serialConfig = SERIAL_8N1;
    if (parityMode == E220_PARITY_8O1) {
        serialConfig = SERIAL_8O1;
    } else if (parityMode == E220_PARITY_8E1) {
        serialConfig = SERIAL_8E1;
    }
    E220_PORT.end();
    E220_PORT.begin(baudRate, serialConfig, PIN_RX, PIN_TX);
#elif defined(TEENSYDUINO)
    uint32_t serialConfig = SERIAL_8N1;
    if (parityMode == E220_PARITY_8O1) {
        serialConfig = SERIAL_8O1;
    } else if (parityMode == E220_PARITY_8E1) {
        serialConfig = SERIAL_8E1;
    }
    // No end() on Teensy 3.x: it underflows the UART FIFO and the next received
    // byte is lost. begin() alone safely reconfigures a running port.
    E220_PORT.begin(baudRate, serialConfig);
#else
    (void)parityMode;           // SoftwareSerial only supports 8N1
    E220_PORT.end();
    E220_PORT.begin(baudRate);
#endif
}

void printStatus(const __FlashStringHelper* actionText, uint8_t statusCode)
{
    Serial.print(actionText);
    Serial.print(F(": "));
    Serial.println(EBYTE_E220_UART::statusToString(statusCode));
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) {
        ;
    }
    Serial.println(F("\nEBYTE E220 - send example"));

    // Module ships at 9600 8N1
    openSerialPort(9600, E220_PARITY_8N1);
    radio.setSerialConfigCallback(openSerialPort);

    uint8_t statusCode = radio.begin();
    printStatus(F("begin"), statusCode);
    if (statusCode != E220_SUCCESS) {
        Serial.println(F("Check wiring and power. Halting."));
        while (true) {
            delay(1000);
        }
    }

    Serial.println(F("Current module settings:"));
    radio.printParameters(Serial);

    // ----- Program basic settings -----
    radio.setAddress(MY_ADDRESS);
    radio.setChannel(RADIO_CHANNEL);
    radio.setUARTBaudRate(E220_UART_9600);
    radio.setParity(E220_PARITY_8N1);
    radio.setAirDataRate(E220_AIR_2K4);
    radio.setSubPacketSize(E220_SUBPACKET_200);
    radio.setTransmitPower(E220_POWER_MAX);
    radio.setTransmissionMethod(E220_TX_FIXED);    // first 3 bytes = ADDH, ADDL, CHAN
    radio.setAmbientRSSIEnabled(true);             // required for RSSI commands
    radio.setPacketRSSIEnabled(false);
    radio.setLBTEnabled(false);
    radio.setWORCycle(E220_WOR_2000MS);
    // radio.setEncryptionKey(0x1234);             // must match on both modules

    // E220_SAVE_PERMANENT writes flash; use E220_SAVE_TEMPORARY for testing
    statusCode = radio.saveParameters(E220_SAVE_PERMANENT);
    printStatus(F("saveParameters"), statusCode);

    // Read back to verify what the module actually stored
    radio.readParameters();
    Serial.println(F("New module settings:"));
    radio.printParameters(Serial);

    uint8_t productInfo[E220_PRODUCT_INFO_LENGTH];
    if (radio.readProductInfo(productInfo) == E220_SUCCESS) {
        Serial.print(F("Product info: "));
        for (uint8_t infoIndex = 0; infoIndex < E220_PRODUCT_INFO_LENGTH; infoIndex++) {
            if (productInfo[infoIndex] < 0x10) {
                Serial.print('0');
            }
            Serial.print(productInfo[infoIndex], HEX);
            Serial.print(' ');
        }
        Serial.println();
    }

    radio.setMode(E220_MODE_NORMAL);
}

void loop()
{
    if (millis() - lastSendTime < SEND_INTERVAL_MS) {
        return;
    }
    lastSendTime = millis();

    // Ambient noise on the channel (lower is better, e.g. -110 dBm)
    int16_t noiseDbm = 0;
    uint8_t statusCode = radio.getAmbientRSSI(&noiseDbm);
    if (statusCode == E220_SUCCESS) {
        Serial.print(F("Ambient noise RSSI: "));
        Serial.print(noiseDbm);
        Serial.print(F(" dBm, "));
    } else {
        Serial.print(F("getAmbientRSSI: "));
        Serial.print(EBYTE_E220_UART::statusToString(statusCode));
        Serial.print(F(", "));
    }

    // Populate the struct with dummy data
    txPacket.syncWord    = PACKET_SYNC_WORD;
    txPacket.packetId    = msgCounter++;
    txPacket.temperature = 20.0f + (float)random(0, 1000) / 100.0f;    // 20.00 - 29.99
    txPacket.humidity    = 40.0f + (float)random(0, 2000) / 100.0f;    // 40.00 - 59.99
    txPacket.batteryMv   = 3300 + random(-200, 200);                   // 3100 - 3499
    txPacket.checksum    = computeChecksum(&txPacket);                 // always set last

    // Send the struct as raw bytes
    statusCode = radio.sendFixedMessage(DEST_ADDRESS, RADIO_CHANNEL,
                                        (const uint8_t*)&txPacket, sizeof(txPacket));

    Serial.print(F("Sent packet #"));
    Serial.print(txPacket.packetId);
    Serial.print(F("  temp="));
    Serial.print(txPacket.temperature, 2);
    Serial.print(F("  hum="));
    Serial.print(txPacket.humidity, 2);
    Serial.print(F("  batt="));
    Serial.print(txPacket.batteryMv);
    Serial.print(F(" mV  ("));
    Serial.print(sizeof(txPacket));
    Serial.print(F(" bytes) -> "));
    Serial.println(EBYTE_E220_UART::statusToString(statusCode));
}
