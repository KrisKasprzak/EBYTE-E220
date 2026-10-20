/*
 * E220_Receive.ino
 *
 * EBYTE E220 receiver example.
 *
 * Shows how to:
 *   - program basic settings (address, channel, fixed transmission,
 *     ambient RSSI and the appended packet RSSI byte)
 *   - receive a struct (uint32_t, float, float, int32_t) sent by E220_Send.ino,
 *     validated with a sync word and checksum
 *   - get RSSI data in two ways:
 *       1. the RSSI byte the module appends to every received packet
 *       2. the RSSI register commands (last packet RSSI and ambient noise)
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
#define MY_ADDRESS          0x0002      // must match DEST_ADDRESS in E220_Send
#define RADIO_CHANNEL       18          // must match the sender
#define MODULE_MODEL        E220_MODEL_900T22
#define NOISE_INTERVAL_MS   10000UL

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

SensorPacket rxPacket;
uint8_t  rxBuffer[E220_MAX_PACKET_SIZE];
uint32_t lastNoiseTime = 0;

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
    Serial.println(F("\nEBYTE E220 - receive example"));

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

    // ----- Program basic settings -----
    radio.setAddress(MY_ADDRESS);
    radio.setChannel(RADIO_CHANNEL);
    radio.setUARTBaudRate(E220_UART_9600);
    radio.setParity(E220_PARITY_8N1);
    radio.setAirDataRate(E220_AIR_2K4);            // must match the sender
    radio.setSubPacketSize(E220_SUBPACKET_200);
    radio.setTransmitPower(E220_POWER_MAX);
    radio.setTransmissionMethod(E220_TX_FIXED);
    radio.setAmbientRSSIEnabled(true);             // enables RSSI register commands
    radio.setPacketRSSIEnabled(true);              // appends RSSI byte to each packet
    radio.setLBTEnabled(false);

    statusCode = radio.saveParameters(E220_SAVE_PERMANENT);
    printStatus(F("saveParameters"), statusCode);

    radio.readParameters();
    radio.printParameters(Serial);

    radio.setMode(E220_MODE_NORMAL);
    Serial.println(F("Waiting for messages..."));
}

void loop()
{
    if (radio.available()) {
        uint16_t rxLength = radio.receiveMessage(rxBuffer, E220_MAX_PACKET_SIZE, 100);
        if (rxLength > 0 && rxLength != sizeof(SensorPacket)) {
            Serial.print(F("Ignored packet with unexpected size: "));
            Serial.print(rxLength);
            Serial.println(F(" bytes"));
        } else if (rxLength == sizeof(SensorPacket)) {
            // Copy the raw bytes into the struct (memcpy avoids alignment issues)
            memcpy(&rxPacket, rxBuffer, sizeof(rxPacket));

            // Reject shifted, foreign or corrupted packets
            if (rxPacket.syncWord != PACKET_SYNC_WORD ||
                rxPacket.checksum != computeChecksum(&rxPacket)) {
                Serial.println(F("Discarded corrupt packet (bad sync word or checksum)"));
                return;
            }

            Serial.print(F("Received packet #"));
            Serial.print(rxPacket.packetId);
            Serial.print(F("  temp="));
            Serial.print(rxPacket.temperature, 2);
            Serial.print(F("  hum="));
            Serial.print(rxPacket.humidity, 2);
            Serial.print(F("  batt="));
            Serial.print(rxPacket.batteryMv);
            Serial.print(F(" mV"));

            // 1. RSSI byte appended to the packet (captured by receiveMessage)
            Serial.print(F("  RSSI(byte)="));
            Serial.print(radio.getReceivedPacketRSSI());
            Serial.print(F(" dBm"));

            // 2. RSSI register command - last packet and ambient noise in one call
            int16_t noiseDbm  = 0;
            int16_t packetDbm = 0;
            uint8_t statusCode = radio.getRSSI(&noiseDbm, &packetDbm);
            if (statusCode == E220_SUCCESS) {
                Serial.print(F("  RSSI(reg)="));
                Serial.print(packetDbm);
                Serial.print(F(" dBm  noise="));
                Serial.print(noiseDbm);
                Serial.println(F(" dBm"));
            } else {
                Serial.print(F("  getRSSI: "));
                Serial.println(EBYTE_E220_UART::statusToString(statusCode));
            }
        }
    }

    // Periodically report channel noise while idle
    if (millis() - lastNoiseTime >= NOISE_INTERVAL_MS && !radio.available()) {
        lastNoiseTime = millis();
        int16_t noiseDbm = 0;
        if (radio.getAmbientRSSI(&noiseDbm) == E220_SUCCESS) {
            Serial.print(F("Idle ambient noise: "));
            Serial.print(noiseDbm);
            Serial.println(F(" dBm"));
        }
    }
}
