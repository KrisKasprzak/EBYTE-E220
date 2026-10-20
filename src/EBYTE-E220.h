/*
 * EBYTE-E220.h
 *
 * Arduino library for the EBYTE E220 series LoRa transceivers (UART / Serial
 * interface), e.g. E220-400T22D, E220-400T30D, E220-900T22D, E220-900T30D.
 * Based on the Semtech LLCC68 chipset.
 *
 * ---------------------------------------------------------------------------
 * MIT License
 *
 * Copyright (c) 2026
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * ---------------------------------------------------------------------------
 *
 * Revision History
 * +---------+------------+-------------------------------------------------+
 * | Version | Date       | Description                                     |
 * +---------+------------+-------------------------------------------------+
 * | 1.0.0   | 2026-10-18 | Initial release. Mode control, full register    |
 * |         |            | programming (permanent / temporary), product    |
 * |         |            | info, transparent / fixed / broadcast send,     |
 * |         |            | packet receive, ambient and packet RSSI, remote |
 * |         |            | (wireless) configuration, factory defaults.     |
 * | 1.0.1   | 2026-10-19 | Fix packet corruption caused by RSSI replies    |
 * |         |            | mixing with radio data: RSSI reply is now       |
 * |         |            | resynchronized on its header and fully consumed;|
 * |         |            | receiveMessage() frames packets using AUX.      |
 * | 1.0.2   | 2026-10-19 | Renamed library and files to EBYTE-E220 and the |
 * |         |            | class/macros to EBYTE_E220_UART to avoid a name |
 * |         |            | clash with another EBYTE_E220 library.          |
 * | 1.1.0   | 2026-10-19 | All status results, enum parameters, enum       |
 * |         |            | getters and the serial config callback parity   |
 * |         |            | argument are now plain uint8_t. Compare against |
 * |         |            | the E220_* enum constants.                      |
 * | 1.2.0   | 2026-10-20 | Added readHardwareModel() and readVersion(),    |
 * |         |            | which read the model name and firmware version  |
 * |         |            | with the AT+DEVTYPE=? and AT+FWCODE=? commands. |
 * |         |            | Replies are read up to the line end with a      |
 * |         |            | timeout, null-terminated and stripped of CR.    |
 * +---------+------------+-------------------------------------------------+
 *
 * Wiring notes
 *   M0, M1  : mode select pins (driven by the MCU)
 *   AUX     : module status output (LOW = busy, HIGH = idle). Optional but
 *             strongly recommended. Pass -1 if not connected.
 *   TXD/RXD : cross-connect to the MCU UART (module TXD -> MCU RX).
 *
 * The module only accepts configuration commands at 9600 baud, 8N1, while in
 * configuration (deep sleep) mode. If the module's operating UART baud rate is
 * not 9600 8N1, register a serial configuration callback with
 * setSerialConfigCallback() so the library can re-open the host serial port.
 * The callback is only invoked when the baud rate or parity actually changes.
 */

#ifndef EBYTE_E220_UART_H
#define EBYTE_E220_UART_H

#include <Arduino.h>

#define EBYTE_E220_UART_VERSION         "1.2.0"

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------
#define E220_CMD_WRITE_PERMANENT        0xC0
#define E220_CMD_READ                   0xC1
#define E220_CMD_WRITE_TEMPORARY        0xC2
#define E220_CMD_REMOTE_PREFIX          0xCF
#define E220_CMD_FORMAT_ERROR           0xFF

#define E220_REG_ADDH                   0x00
#define E220_REG_ADDL                   0x01
#define E220_REG_REG0                   0x02
#define E220_REG_REG1                   0x03
#define E220_REG_REG2                   0x04
#define E220_REG_REG3                   0x05
#define E220_REG_CRYPT_H                0x06
#define E220_REG_CRYPT_L                0x07
#define E220_REG_PID                    0x80

#define E220_CONFIG_REG_COUNT           6
#define E220_ALL_REG_COUNT              8
#define E220_PRODUCT_INFO_LENGTH        7
#define E220_MAX_REG_BLOCK              8

#define E220_RSSI_REG_AMBIENT           0x00
#define E220_RSSI_REG_LAST_PACKET       0x01

#define E220_BROADCAST_ADDRESS          0xFFFF
#define E220_MAX_PACKET_SIZE            200

#define E220_DEFAULT_TIMEOUT            1000UL
#define E220_TX_TIMEOUT                 6000UL
#define E220_NO_AUX_DELAY_MS            100UL
#define E220_AUX_SETTLE_MS              2UL
#define E220_CONFIG_BAUD                9600UL

// ---------------------------------------------------------------------------
// Enumerations (values match the register bit patterns)
// ---------------------------------------------------------------------------

// Operating modes, value = (M1 << 1) | M0
enum E220_Mode : uint8_t {
    E220_MODE_NORMAL    = 0,    // M0=0 M1=0 : transmission / reception
    E220_MODE_WOR_TX    = 1,    // M0=1 M1=0 : WOR transmitting (adds wake-up preamble)
    E220_MODE_WOR_RX    = 2,    // M0=0 M1=1 : WOR receiving (power saving)
    E220_MODE_CONFIG    = 3     // M0=1 M1=1 : deep sleep / configuration
};

enum E220_Model : uint8_t {
    E220_MODEL_400T22 = 0,      // 410.125 - 493.125 MHz, 22 dBm
    E220_MODEL_400T30 = 1,      // 410.125 - 493.125 MHz, 30 dBm
    E220_MODEL_900T22 = 2,      // 850.125 - 930.125 MHz, 22 dBm
    E220_MODEL_900T30 = 3       // 850.125 - 930.125 MHz, 30 dBm
};

// REG0 bits 7..5
enum E220_UARTBaud : uint8_t {
    E220_UART_1200   = 0,
    E220_UART_2400   = 1,
    E220_UART_4800   = 2,
    E220_UART_9600   = 3,       // default
    E220_UART_19200  = 4,
    E220_UART_38400  = 5,
    E220_UART_57600  = 6,
    E220_UART_115200 = 7
};

// REG0 bits 4..3
enum E220_Parity : uint8_t {
    E220_PARITY_8N1   = 0,      // default
    E220_PARITY_8O1   = 1,
    E220_PARITY_8E1   = 2,
    E220_PARITY_8N1_B = 3       // also 8N1
};

// REG0 bits 2..0
enum E220_AirDataRate : uint8_t {
    E220_AIR_2K4     = 2,       // default (0 and 1 also mean 2.4k)
    E220_AIR_4K8     = 3,
    E220_AIR_9K6     = 4,
    E220_AIR_19K2    = 5,
    E220_AIR_38K4    = 6,
    E220_AIR_62K5    = 7
};

// REG1 bits 7..6
enum E220_SubPacket : uint8_t {
    E220_SUBPACKET_200 = 0,     // default
    E220_SUBPACKET_128 = 1,
    E220_SUBPACKET_64  = 2,
    E220_SUBPACKET_32  = 3
};

// REG1 bits 1..0 (dBm values depend on the model, see getTransmitPowerDbm())
enum E220_TxPower : uint8_t {
    E220_POWER_MAX     = 0,     // 22 dBm (T22) / 30 dBm (T30), default
    E220_POWER_HIGH    = 1,     // 17 dBm (T22) / 27 dBm (T30)
    E220_POWER_MEDIUM  = 2,     // 13 dBm (T22) / 24 dBm (T30)
    E220_POWER_LOW     = 3      // 10 dBm (T22) / 21 dBm (T30)
};

// REG3 bit 6
enum E220_TxMethod : uint8_t {
    E220_TX_TRANSPARENT = 0,    // default
    E220_TX_FIXED       = 1
};

// REG3 bits 2..0, period = (1 + value) * 500 ms
enum E220_WORCycle : uint8_t {
    E220_WOR_500MS  = 0,
    E220_WOR_1000MS = 1,
    E220_WOR_1500MS = 2,
    E220_WOR_2000MS = 3,        // default
    E220_WOR_2500MS = 4,
    E220_WOR_3000MS = 5,
    E220_WOR_3500MS = 6,
    E220_WOR_4000MS = 7
};

enum E220_SaveType : uint8_t {
    E220_SAVE_PERMANENT = 0,    // C0 command: retained after power down
    E220_SAVE_TEMPORARY = 1     // C2 command: lost after power down
};

enum E220_Status : uint8_t {
    E220_SUCCESS = 0,
    E220_ERR_TIMEOUT,
    E220_ERR_AUX_TIMEOUT,
    E220_ERR_INVALID_PARAM,
    E220_ERR_FORMAT,            // module answered FF FF FF
    E220_ERR_BAD_RESPONSE,
    E220_ERR_WRONG_MODE,
    E220_ERR_NOT_ENABLED,
    E220_ERR_NOT_INITIALIZED
};

// Called when the host serial port must be re-opened with a new baud/parity.
// parityMode is an E220_Parity value.
typedef void (*E220_SerialConfigCallback)(uint32_t baudRate, uint8_t parityMode);

// ---------------------------------------------------------------------------
// Driver class
//
// Status results, enum-valued parameters and enum-valued getters are all plain
// uint8_t. Compare them against the E220_* enum constants above, e.g.
//   if (radio.begin() != E220_SUCCESS) { ... }
//   if (radio.getParity() == E220_PARITY_8E1) { ... }
// ---------------------------------------------------------------------------
class EBYTE_E220_UART {
public:
    EBYTE_E220_UART(Stream* serialPort, int8_t pinM0, int8_t pinM1, int8_t pinAUX = -1,
                    uint8_t moduleModel = E220_MODEL_900T22);   // E220_Model

    // ----- Initialization -----
    uint8_t          begin();
    // currentBaud/currentParity: how the host port is opened when the callback is set
    void             setSerialConfigCallback(E220_SerialConfigCallback callbackFunc,
                                             uint32_t currentBaud = E220_CONFIG_BAUD,
                                             uint8_t currentParity = E220_PARITY_8N1);
    void             setModel(uint8_t moduleModel);         // E220_Model
    uint8_t          getModel();

    // Read via AT commands (AT+DEVTYPE=? / AT+FWCODE=?) in configuration mode.
    // Return a pointer to an internal buffer, overwritten by the next call, or
    // NULL if there was no reply. readHardwareModel() also returns NULL if the
    // reply does not start with "E220".
    char*            readHardwareModel();
    char*            readVersion();

    // ----- Mode / status -----
    uint8_t          setMode(uint8_t newMode);              // E220_Mode
    uint8_t          getMode();
    bool             isBusy();
    bool             waitForAux(uint32_t timeoutMs = E220_DEFAULT_TIMEOUT);

    // ----- Programming (talks to the module) -----
    uint8_t          readParameters();
    uint8_t          saveParameters(uint8_t saveType = E220_SAVE_PERMANENT);   // E220_SaveType
    uint8_t          restoreDefaults(uint8_t saveType = E220_SAVE_PERMANENT);
    uint8_t          readProductInfo(uint8_t* infoBuffer);
    uint8_t          readRegisters(uint8_t startReg, uint8_t* dataBuffer, uint8_t dataLength);
    uint8_t          writeRegisters(uint8_t startReg, const uint8_t* dataBuffer, uint8_t dataLength,
                                    uint8_t saveType = E220_SAVE_PERMANENT);

    // ----- Cached parameter setters / getters (call saveParameters() to apply) -----
    void             setAddress(uint16_t newAddress);
    uint16_t         getAddress();
    void             setAddressHigh(uint8_t addrHigh);
    uint8_t          getAddressHigh();
    void             setAddressLow(uint8_t addrLow);
    uint8_t          getAddressLow();

    bool             setChannel(uint8_t newChannel);
    uint8_t          getChannel();
    uint8_t          getMaxChannel();
    float            getFrequency();
    float            getBaseFrequency();

    void             setUARTBaudRate(uint8_t baudSetting);  // E220_UARTBaud
    uint8_t          getUARTBaudRate();
    uint32_t         getUARTBaudRateValue();

    void             setParity(uint8_t paritySetting);      // E220_Parity
    uint8_t          getParity();

    void             setAirDataRate(uint8_t rateSetting);   // E220_AirDataRate
    uint8_t          getAirDataRate();
    uint32_t         getAirDataRateValue();

    void             setSubPacketSize(uint8_t packetSetting);   // E220_SubPacket
    uint8_t          getSubPacketSize();
    uint8_t          getSubPacketSizeValue();

    void             setAmbientRSSIEnabled(bool enableFlag);
    bool             getAmbientRSSIEnabled();

    void             setTransmitPower(uint8_t powerSetting);    // E220_TxPower
    uint8_t          getTransmitPower();
    int8_t           getTransmitPowerDbm();

    void             setPacketRSSIEnabled(bool enableFlag);
    bool             getPacketRSSIEnabled();

    void             setTransmissionMethod(uint8_t methodSetting);  // E220_TxMethod
    uint8_t          getTransmissionMethod();

    void             setLBTEnabled(bool enableFlag);
    bool             getLBTEnabled();

    void             setWORCycle(uint8_t cycleSetting);     // E220_WORCycle
    uint8_t          getWORCycle();
    uint16_t         getWORCycleMs();

    void             setEncryptionKey(uint16_t cryptKey);   // write-only register

    // ----- RSSI -----
    uint8_t          getAmbientRSSI(int16_t* rssiDbm);
    uint8_t          getLastPacketRSSI(int16_t* rssiDbm);
    uint8_t          getRSSI(int16_t* ambientDbm, int16_t* packetDbm);
    int16_t          getReceivedPacketRSSI();               // from appended RSSI byte
    static int16_t   convertRSSI(uint8_t rawValue);

    // ----- Transmit -----
    uint8_t          sendMessage(const uint8_t* dataBuffer, uint8_t dataLength);
    uint8_t          sendMessage(const char* textMessage);
    uint8_t          sendFixedMessage(uint16_t destAddress, uint8_t destChannel,
                                      const uint8_t* dataBuffer, uint8_t dataLength);
    uint8_t          sendFixedMessage(uint16_t destAddress, uint8_t destChannel,
                                      const char* textMessage);
    uint8_t          sendBroadcastMessage(uint8_t destChannel, const uint8_t* dataBuffer,
                                          uint8_t dataLength);
    uint8_t          sendBroadcastMessage(uint8_t destChannel, const char* textMessage);

    // ----- Receive -----
    int              available();
    int              read();
    uint16_t         receiveMessage(uint8_t* dataBuffer, uint16_t maxLength,
                                    uint32_t timeoutMs = E220_DEFAULT_TIMEOUT);

    // ----- Remote (wireless) configuration -----
    uint8_t          sendRemoteConfig(uint8_t commandByte, uint8_t startReg,
                                      const uint8_t* dataBuffer, uint8_t dataLength,
                                      uint8_t* responseBuffer, uint8_t responseSize,
                                      uint8_t* responseLength,
                                      uint32_t timeoutMs = E220_TX_TIMEOUT);

    // ----- Diagnostics -----
    void             printParameters(Print& outputPort);
    static const char* statusToString(uint8_t statusCode);

private:
    uint8_t          enterConfigMode(uint8_t* previousMode);
    uint8_t          leaveConfigMode(uint8_t previousMode);
    uint8_t          readResponse(uint8_t* respBuffer, uint8_t startReg, uint8_t dataLength,
                                  uint32_t timeoutMs);
    uint8_t          transmitRaw(const uint8_t* headerBuffer, uint8_t headerLength,
                                 const uint8_t* dataBuffer, uint8_t dataLength);
    uint8_t          readRSSIRegisters(uint8_t startReg, uint8_t* rawBuffer, uint8_t rawLength);
    void             clearInput();
    void             drainInput(uint32_t quietMs);
    void             applySerialConfig(uint32_t baudRate, uint8_t parityMode);
    uint32_t         packetGapMs();
    bool             readATValue(const char* atCommand, const char* replyPrefix,
                                 char* valueBuffer, uint8_t bufferSize);

    Stream*                   _serialPort;
    int8_t                    _pinM0;
    int8_t                    _pinM1;
    int8_t                    _pinAUX;
    uint8_t                   _moduleModel;
    uint8_t                   _currentMode;
    E220_SerialConfigCallback _serialCallback;
    uint32_t                  _hostBaud;
    uint8_t                   _hostParity;
    uint8_t                   _registers[E220_CONFIG_REG_COUNT];
    uint16_t                  _cryptKey;
    bool                      _cryptPending;
    bool                      _paramsLoaded;
    int16_t                   _packetRssi;
	char HardwareModel[30];
	char Version[30];
};

#endif // EBYTE_E220_UART_H
