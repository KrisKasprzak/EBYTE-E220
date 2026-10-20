/*
 * EBYTE-E220.cpp
 *
 * Implementation of the EBYTE E220 LoRa transceiver driver.
 * See EBYTE-E220.h for license and revision history.
 */

#include "EBYTE-E220.h"

// Register bit masks
#define E220_MASK_UART_BAUD     0xE0
#define E220_MASK_PARITY        0x18
#define E220_MASK_AIR_RATE      0x07
#define E220_MASK_SUB_PACKET    0xC0
#define E220_MASK_AMBIENT_RSSI  0x20
#define E220_MASK_TX_POWER      0x03
#define E220_MASK_PACKET_RSSI   0x80
#define E220_MASK_FIXED_TX      0x40
#define E220_MASK_LBT           0x10
#define E220_MASK_WOR_CYCLE     0x07

// Factory default register values
#define E220_DEFAULT_REG0       0x62    // 9600 baud, 8N1, 2.4k air data rate
#define E220_DEFAULT_REG1       0x00    // 200 byte sub packet, ambient RSSI off, max power
#define E220_DEFAULT_REG3       0x03    // RSSI byte off, transparent, LBT off, WOR 2000 ms
#define E220_DEFAULT_CH_400     0x17    // 433.125 MHz
#define E220_DEFAULT_CH_900     0x12    // 868.125 MHz

// ===========================================================================
// Construction / initialization
// ===========================================================================

EBYTE_E220_UART::EBYTE_E220_UART(Stream* serialPort, int8_t pinM0, int8_t pinM1, int8_t pinAUX,
                                 uint8_t moduleModel)
{
    _serialPort     = serialPort;
    _pinM0          = pinM0;
    _pinM1          = pinM1;
    _pinAUX         = pinAUX;
    _moduleModel    = moduleModel;
    _currentMode    = E220_MODE_NORMAL;
    _serialCallback = NULL;
    _hostBaud       = E220_CONFIG_BAUD;
    _hostParity     = E220_PARITY_8N1;
    _cryptKey      = 0;
    _cryptPending   = false;
    _paramsLoaded   = false;
    _packetRssi     = 0;
    HardwareModel[0] = '\0';
    Version[0]       = '\0';

    _registers[E220_REG_ADDH] = 0x00;
    _registers[E220_REG_ADDL] = 0x00;
    _registers[E220_REG_REG0] = E220_DEFAULT_REG0;
    _registers[E220_REG_REG1] = E220_DEFAULT_REG1;
    _registers[E220_REG_REG2] = (moduleModel <= E220_MODEL_400T30) ? E220_DEFAULT_CH_400
                                                                   : E220_DEFAULT_CH_900;
    _registers[E220_REG_REG3] = E220_DEFAULT_REG3;
}

uint8_t EBYTE_E220_UART::begin()
{
    if (_serialPort == NULL || _pinM0 < 0 || _pinM1 < 0) {
        return E220_ERR_INVALID_PARAM;
    }

    pinMode(_pinM0, OUTPUT);
    pinMode(_pinM1, OUTPUT);
    if (_pinAUX >= 0) {
        pinMode(_pinAUX, INPUT);
    }

    digitalWrite(_pinM0, LOW);
    digitalWrite(_pinM1, LOW);
    _currentMode = E220_MODE_NORMAL;

    // The module holds AUX low during its power-on self check
    if (!waitForAux(E220_DEFAULT_TIMEOUT)) {
        return E220_ERR_AUX_TIMEOUT;
    }

    clearInput();
    return readParameters();
}

void EBYTE_E220_UART::setSerialConfigCallback(E220_SerialConfigCallback callbackFunc,
                                              uint32_t currentBaud, uint8_t currentParity)
{
    _serialCallback = callbackFunc;
    _hostBaud       = currentBaud;
    _hostParity     = currentParity;
}

void EBYTE_E220_UART::setModel(uint8_t moduleModel)
{
    _moduleModel = moduleModel;
}

uint8_t EBYTE_E220_UART::getModel()
{
    return _moduleModel;
}

// ===========================================================================
// Mode / status
// ===========================================================================

uint8_t EBYTE_E220_UART::setMode(uint8_t newMode)
{
    if (newMode > E220_MODE_CONFIG) {
        return E220_ERR_INVALID_PARAM;
    }

    // Never switch mode while the module is still busy
    waitForAux(E220_TX_TIMEOUT);

    uint8_t oldMode = _currentMode;

    digitalWrite(_pinM0, (newMode & 0x01) ? HIGH : LOW);
    digitalWrite(_pinM1, (newMode & 0x02) ? HIGH : LOW);
    _currentMode = newMode;

    if (newMode == E220_MODE_CONFIG && oldMode != E220_MODE_CONFIG) {
        applySerialConfig(E220_CONFIG_BAUD, E220_PARITY_8N1);
    } else if (oldMode == E220_MODE_CONFIG && newMode != E220_MODE_CONFIG && _paramsLoaded) {
        applySerialConfig(getUARTBaudRateValue(), getParity());
    }

    // AUX drops shortly after a mode change and rises once the module is ready
    delay(E220_AUX_SETTLE_MS);
    if (!waitForAux(E220_DEFAULT_TIMEOUT)) {
        return E220_ERR_AUX_TIMEOUT;
    }

    clearInput();
    return E220_SUCCESS;
}

uint8_t EBYTE_E220_UART::getMode()
{
    return _currentMode;
}

bool EBYTE_E220_UART::isBusy()
{
    if (_pinAUX < 0) {
        return false;
    }
    return digitalRead(_pinAUX) == LOW;
}

bool EBYTE_E220_UART::waitForAux(uint32_t timeoutMs)
{
    if (_pinAUX < 0) {
        delay((timeoutMs < E220_NO_AUX_DELAY_MS) ? timeoutMs : E220_NO_AUX_DELAY_MS);
        return true;
    }

    uint32_t startTime = millis();
    while (digitalRead(_pinAUX) == LOW) {
        if (millis() - startTime >= timeoutMs) {
            return false;
        }
        yield();
    }

    delay(E220_AUX_SETTLE_MS);
    return true;
}

// ===========================================================================
// Programming
// ===========================================================================

uint8_t EBYTE_E220_UART::readParameters()
{
    uint8_t previousMode;
    uint8_t statusCode = enterConfigMode(&previousMode);
    if (statusCode != E220_SUCCESS) {
        return statusCode;
    }

    uint8_t regBuffer[E220_CONFIG_REG_COUNT];
    statusCode = readRegisters(E220_REG_ADDH, regBuffer, E220_CONFIG_REG_COUNT);
    if (statusCode == E220_SUCCESS) {
        for (uint8_t regIndex = 0; regIndex < E220_CONFIG_REG_COUNT; regIndex++) {
            _registers[regIndex] = regBuffer[regIndex];
        }
        _paramsLoaded = true;
    }

    uint8_t leaveStatus = leaveConfigMode(previousMode);
    return (statusCode != E220_SUCCESS) ? statusCode : leaveStatus;
}

uint8_t EBYTE_E220_UART::saveParameters(uint8_t saveType)
{
    uint8_t previousMode;
    uint8_t statusCode = enterConfigMode(&previousMode);
    if (statusCode != E220_SUCCESS) {
        return statusCode;
    }

    uint8_t regBuffer[E220_ALL_REG_COUNT];
    uint8_t regCount = E220_CONFIG_REG_COUNT;
    for (uint8_t regIndex = 0; regIndex < E220_CONFIG_REG_COUNT; regIndex++) {
        regBuffer[regIndex] = _registers[regIndex];
    }

    // The crypt registers are write-only; only send them when explicitly set
    if (_cryptPending) {
        regBuffer[E220_REG_CRYPT_H] = (uint8_t)(_cryptKey >> 8);
        regBuffer[E220_REG_CRYPT_L] = (uint8_t)(_cryptKey & 0xFF);
        regCount = E220_ALL_REG_COUNT;
    }

    statusCode = writeRegisters(E220_REG_ADDH, regBuffer, regCount, saveType);
    if (statusCode == E220_SUCCESS) {
        _cryptPending = false;
        _paramsLoaded = true;
    }

    uint8_t leaveStatus = leaveConfigMode(previousMode);
    return (statusCode != E220_SUCCESS) ? statusCode : leaveStatus;
}

uint8_t EBYTE_E220_UART::restoreDefaults(uint8_t saveType)
{
    _registers[E220_REG_ADDH] = 0x00;
    _registers[E220_REG_ADDL] = 0x00;
    _registers[E220_REG_REG0] = E220_DEFAULT_REG0;
    _registers[E220_REG_REG1] = E220_DEFAULT_REG1;
    _registers[E220_REG_REG2] = (_moduleModel <= E220_MODEL_400T30) ? E220_DEFAULT_CH_400
                                                                    : E220_DEFAULT_CH_900;
    _registers[E220_REG_REG3] = E220_DEFAULT_REG3;
    _cryptKey     = 0;
    _cryptPending = true;
    return saveParameters(saveType);
}

uint8_t EBYTE_E220_UART::readProductInfo(uint8_t* infoBuffer)
{
    return readRegisters(E220_REG_PID, infoBuffer, E220_PRODUCT_INFO_LENGTH);
}

uint8_t EBYTE_E220_UART::readRegisters(uint8_t startReg, uint8_t* dataBuffer, uint8_t dataLength)
{
    if (dataBuffer == NULL || dataLength == 0 || dataLength > E220_MAX_REG_BLOCK) {
        return E220_ERR_INVALID_PARAM;
    }

    uint8_t previousMode;
    uint8_t statusCode = enterConfigMode(&previousMode);
    if (statusCode != E220_SUCCESS) {
        return statusCode;
    }

    clearInput();
    uint8_t cmdBuffer[3] = { E220_CMD_READ, startReg, dataLength };
    _serialPort->write(cmdBuffer, sizeof(cmdBuffer));
    _serialPort->flush();

    uint8_t respBuffer[3 + E220_MAX_REG_BLOCK];
    statusCode = readResponse(respBuffer, startReg, dataLength, E220_DEFAULT_TIMEOUT);
    if (statusCode == E220_SUCCESS) {
        for (uint8_t byteIndex = 0; byteIndex < dataLength; byteIndex++) {
            dataBuffer[byteIndex] = respBuffer[3 + byteIndex];
        }
    }

    uint8_t leaveStatus = leaveConfigMode(previousMode);
    return (statusCode != E220_SUCCESS) ? statusCode : leaveStatus;
}

uint8_t EBYTE_E220_UART::writeRegisters(uint8_t startReg, const uint8_t* dataBuffer,
                                        uint8_t dataLength, uint8_t saveType)
{
    if (dataBuffer == NULL || dataLength == 0 || dataLength > E220_MAX_REG_BLOCK) {
        return E220_ERR_INVALID_PARAM;
    }

    uint8_t previousMode;
    uint8_t statusCode = enterConfigMode(&previousMode);
    if (statusCode != E220_SUCCESS) {
        return statusCode;
    }

    clearInput();
    uint8_t cmdBuffer[3] = {
        (saveType == E220_SAVE_PERMANENT) ? (uint8_t)E220_CMD_WRITE_PERMANENT
                                          : (uint8_t)E220_CMD_WRITE_TEMPORARY,
        startReg,
        dataLength
    };
    _serialPort->write(cmdBuffer, sizeof(cmdBuffer));
    _serialPort->write(dataBuffer, dataLength);
    _serialPort->flush();

    // The module echoes the written block with a C1 header
    uint8_t respBuffer[3 + E220_MAX_REG_BLOCK];
    statusCode = readResponse(respBuffer, startReg, dataLength, E220_DEFAULT_TIMEOUT);

    // Writing to flash keeps AUX low for a short time
    waitForAux(E220_DEFAULT_TIMEOUT);

    uint8_t leaveStatus = leaveConfigMode(previousMode);
    return (statusCode != E220_SUCCESS) ? statusCode : leaveStatus;
}

// ===========================================================================
// Cached parameters - addressing
// ===========================================================================

void EBYTE_E220_UART::setAddress(uint16_t newAddress)
{
    _registers[E220_REG_ADDH] = (uint8_t)(newAddress >> 8);
    _registers[E220_REG_ADDL] = (uint8_t)(newAddress & 0xFF);
}

uint16_t EBYTE_E220_UART::getAddress()
{
    return ((uint16_t)_registers[E220_REG_ADDH] << 8) | _registers[E220_REG_ADDL];
}

void EBYTE_E220_UART::setAddressHigh(uint8_t addrHigh)
{
    _registers[E220_REG_ADDH] = addrHigh;
}

uint8_t EBYTE_E220_UART::getAddressHigh()
{
    return _registers[E220_REG_ADDH];
}

void EBYTE_E220_UART::setAddressLow(uint8_t addrLow)
{
    _registers[E220_REG_ADDL] = addrLow;
}

uint8_t EBYTE_E220_UART::getAddressLow()
{
    return _registers[E220_REG_ADDL];
}

// ===========================================================================
// Cached parameters - channel / frequency
// ===========================================================================

bool EBYTE_E220_UART::setChannel(uint8_t newChannel)
{
    if (newChannel > getMaxChannel()) {
        return false;
    }
    _registers[E220_REG_REG2] = newChannel;
    return true;
}

uint8_t EBYTE_E220_UART::getChannel()
{
    return _registers[E220_REG_REG2];
}

uint8_t EBYTE_E220_UART::getMaxChannel()
{
    return (_moduleModel <= E220_MODEL_400T30) ? 83 : 80;
}

float EBYTE_E220_UART::getBaseFrequency()
{
    return (_moduleModel <= E220_MODEL_400T30) ? 410.125f : 850.125f;
}

float EBYTE_E220_UART::getFrequency()
{
    return getBaseFrequency() + (float)getChannel();
}

// ===========================================================================
// Cached parameters - REG0
// ===========================================================================

void EBYTE_E220_UART::setUARTBaudRate(uint8_t baudSetting)
{
    _registers[E220_REG_REG0] = (_registers[E220_REG_REG0] & ~E220_MASK_UART_BAUD) |
                                (((uint8_t)baudSetting << 5) & E220_MASK_UART_BAUD);
}

uint8_t EBYTE_E220_UART::getUARTBaudRate()
{
    return (uint8_t)((_registers[E220_REG_REG0] & E220_MASK_UART_BAUD) >> 5);
}

uint32_t EBYTE_E220_UART::getUARTBaudRateValue()
{
    static const uint32_t baudTable[8] = {
        1200UL, 2400UL, 4800UL, 9600UL, 19200UL, 38400UL, 57600UL, 115200UL
    };
    return baudTable[getUARTBaudRate()];
}

void EBYTE_E220_UART::setParity(uint8_t paritySetting)
{
    _registers[E220_REG_REG0] = (_registers[E220_REG_REG0] & ~E220_MASK_PARITY) |
                                (((uint8_t)paritySetting << 3) & E220_MASK_PARITY);
}

uint8_t EBYTE_E220_UART::getParity()
{
    return (uint8_t)((_registers[E220_REG_REG0] & E220_MASK_PARITY) >> 3);
}

void EBYTE_E220_UART::setAirDataRate(uint8_t rateSetting)
{
    _registers[E220_REG_REG0] = (_registers[E220_REG_REG0] & ~E220_MASK_AIR_RATE) |
                                ((uint8_t)rateSetting & E220_MASK_AIR_RATE);
}

uint8_t EBYTE_E220_UART::getAirDataRate()
{
    uint8_t rawRate = _registers[E220_REG_REG0] & E220_MASK_AIR_RATE;
    if (rawRate < E220_AIR_2K4) {
        rawRate = E220_AIR_2K4;     // 000 and 001 are aliases of 2.4k
    }
    return rawRate;
}

uint32_t EBYTE_E220_UART::getAirDataRateValue()
{
    static const uint32_t rateTable[8] = {
        2400UL, 2400UL, 2400UL, 4800UL, 9600UL, 19200UL, 38400UL, 62500UL
    };
    return rateTable[getAirDataRate()];
}

// ===========================================================================
// Cached parameters - REG1
// ===========================================================================

void EBYTE_E220_UART::setSubPacketSize(uint8_t packetSetting)
{
    _registers[E220_REG_REG1] = (_registers[E220_REG_REG1] & ~E220_MASK_SUB_PACKET) |
                                (((uint8_t)packetSetting << 6) & E220_MASK_SUB_PACKET);
}

uint8_t EBYTE_E220_UART::getSubPacketSize()
{
    return (uint8_t)((_registers[E220_REG_REG1] & E220_MASK_SUB_PACKET) >> 6);
}

uint8_t EBYTE_E220_UART::getSubPacketSizeValue()
{
    static const uint8_t sizeTable[4] = { 200, 128, 64, 32 };
    return sizeTable[getSubPacketSize()];
}

void EBYTE_E220_UART::setAmbientRSSIEnabled(bool enableFlag)
{
    if (enableFlag) {
        _registers[E220_REG_REG1] |= E220_MASK_AMBIENT_RSSI;
    } else {
        _registers[E220_REG_REG1] &= (uint8_t)~E220_MASK_AMBIENT_RSSI;
    }
}

bool EBYTE_E220_UART::getAmbientRSSIEnabled()
{
    return (_registers[E220_REG_REG1] & E220_MASK_AMBIENT_RSSI) != 0;
}

void EBYTE_E220_UART::setTransmitPower(uint8_t powerSetting)
{
    _registers[E220_REG_REG1] = (_registers[E220_REG_REG1] & ~E220_MASK_TX_POWER) |
                                ((uint8_t)powerSetting & E220_MASK_TX_POWER);
}

uint8_t EBYTE_E220_UART::getTransmitPower()
{
    return (uint8_t)(_registers[E220_REG_REG1] & E220_MASK_TX_POWER);
}

int8_t EBYTE_E220_UART::getTransmitPowerDbm()
{
    static const int8_t powerTable22[4] = { 22, 17, 13, 10 };
    static const int8_t powerTable30[4] = { 30, 27, 24, 21 };
    if (_moduleModel == E220_MODEL_400T30 || _moduleModel == E220_MODEL_900T30) {
        return powerTable30[getTransmitPower()];
    }
    return powerTable22[getTransmitPower()];
}

// ===========================================================================
// Cached parameters - REG3
// ===========================================================================

void EBYTE_E220_UART::setPacketRSSIEnabled(bool enableFlag)
{
    if (enableFlag) {
        _registers[E220_REG_REG3] |= E220_MASK_PACKET_RSSI;
    } else {
        _registers[E220_REG_REG3] &= (uint8_t)~E220_MASK_PACKET_RSSI;
    }
}

bool EBYTE_E220_UART::getPacketRSSIEnabled()
{
    return (_registers[E220_REG_REG3] & E220_MASK_PACKET_RSSI) != 0;
}

void EBYTE_E220_UART::setTransmissionMethod(uint8_t methodSetting)
{
    if (methodSetting == E220_TX_FIXED) {
        _registers[E220_REG_REG3] |= E220_MASK_FIXED_TX;
    } else {
        _registers[E220_REG_REG3] &= (uint8_t)~E220_MASK_FIXED_TX;
    }
}

uint8_t EBYTE_E220_UART::getTransmissionMethod()
{
    return (_registers[E220_REG_REG3] & E220_MASK_FIXED_TX) ? E220_TX_FIXED : E220_TX_TRANSPARENT;
}

void EBYTE_E220_UART::setLBTEnabled(bool enableFlag)
{
    if (enableFlag) {
        _registers[E220_REG_REG3] |= E220_MASK_LBT;
    } else {
        _registers[E220_REG_REG3] &= (uint8_t)~E220_MASK_LBT;
    }
}

bool EBYTE_E220_UART::getLBTEnabled()
{
    return (_registers[E220_REG_REG3] & E220_MASK_LBT) != 0;
}

void EBYTE_E220_UART::setWORCycle(uint8_t cycleSetting)
{
    _registers[E220_REG_REG3] = (_registers[E220_REG_REG3] & ~E220_MASK_WOR_CYCLE) |
                                ((uint8_t)cycleSetting & E220_MASK_WOR_CYCLE);
}

uint8_t EBYTE_E220_UART::getWORCycle()
{
    return (uint8_t)(_registers[E220_REG_REG3] & E220_MASK_WOR_CYCLE);
}

uint16_t EBYTE_E220_UART::getWORCycleMs()
{
    return (uint16_t)(1 + getWORCycle()) * 500;
}

void EBYTE_E220_UART::setEncryptionKey(uint16_t cryptKey)
{
    _cryptKey     = cryptKey;
    _cryptPending = true;
}

// ===========================================================================
// RSSI
// ===========================================================================

int16_t EBYTE_E220_UART::convertRSSI(uint8_t rawValue)
{
    return -(int16_t)(256 - (int16_t)rawValue);
}

uint8_t EBYTE_E220_UART::readRSSIRegisters(uint8_t startReg, uint8_t* rawBuffer, uint8_t rawLength)
{
    if (!getAmbientRSSIEnabled()) {
        return E220_ERR_NOT_ENABLED;
    }
    if (_currentMode != E220_MODE_NORMAL && _currentMode != E220_MODE_WOR_TX) {
        return E220_ERR_WRONG_MODE;
    }
    if (!waitForAux(E220_TX_TIMEOUT)) {
        return E220_ERR_AUX_TIMEOUT;
    }

    // Radio data and the RSSI reply share the UART: let any packet output finish
    // and discard it so it cannot be mistaken for the reply
    drainInput(packetGapMs());

    uint8_t cmdBuffer[6] = { 0xC0, 0xC1, 0xC2, 0xC3, startReg, rawLength };
    _serialPort->write(cmdBuffer, sizeof(cmdBuffer));
    _serialPort->flush();

    // Scan for the "C1 startReg rawLength" header, skipping stray bytes, so the
    // reply is always consumed completely and never left in the receive buffer
    uint8_t  respBuffer[5] = { 0 };
    uint8_t  rxCount   = 0;
    uint8_t  respTotal = 3 + rawLength;
    uint32_t startTime = millis();

    while (rxCount < respTotal) {
        if (_serialPort->available()) {
            uint8_t rxByte = (uint8_t)_serialPort->read();
            if (rxCount == 0) {
                if (rxByte == E220_CMD_READ) {
                    respBuffer[rxCount++] = rxByte;
                }
            } else if (rxCount == 1 || rxCount == 2) {
                uint8_t expectedByte = (rxCount == 1) ? startReg : rawLength;
                if (rxByte == expectedByte) {
                    respBuffer[rxCount++] = rxByte;
                } else {
                    rxCount = (rxByte == E220_CMD_READ) ? 1 : 0;
                }
            } else {
                respBuffer[rxCount++] = rxByte;
            }
        } else if (millis() - startTime >= E220_DEFAULT_TIMEOUT) {
            // Swallow a late reply so it cannot corrupt the next received packet
            drainInput(packetGapMs());
            return (rxCount == 0) ? E220_ERR_TIMEOUT : E220_ERR_BAD_RESPONSE;
        } else {
            yield();
        }
    }

    for (uint8_t byteIndex = 0; byteIndex < rawLength; byteIndex++) {
        rawBuffer[byteIndex] = respBuffer[3 + byteIndex];
    }
    return E220_SUCCESS;
}

uint8_t EBYTE_E220_UART::getAmbientRSSI(int16_t* rssiDbm)
{
    if (rssiDbm == NULL) {
        return E220_ERR_INVALID_PARAM;
    }
    uint8_t rawValue = 0;
    uint8_t statusCode = readRSSIRegisters(E220_RSSI_REG_AMBIENT, &rawValue, 1);
    if (statusCode == E220_SUCCESS) {
        *rssiDbm = convertRSSI(rawValue);
    }
    return statusCode;
}

uint8_t EBYTE_E220_UART::getLastPacketRSSI(int16_t* rssiDbm)
{
    if (rssiDbm == NULL) {
        return E220_ERR_INVALID_PARAM;
    }
    uint8_t rawValue = 0;
    uint8_t statusCode = readRSSIRegisters(E220_RSSI_REG_LAST_PACKET, &rawValue, 1);
    if (statusCode == E220_SUCCESS) {
        *rssiDbm = convertRSSI(rawValue);
    }
    return statusCode;
}

uint8_t EBYTE_E220_UART::getRSSI(int16_t* ambientDbm, int16_t* packetDbm)
{
    if (ambientDbm == NULL || packetDbm == NULL) {
        return E220_ERR_INVALID_PARAM;
    }
    uint8_t rawValues[2] = { 0, 0 };
    uint8_t statusCode = readRSSIRegisters(E220_RSSI_REG_AMBIENT, rawValues, 2);
    if (statusCode == E220_SUCCESS) {
        *ambientDbm = convertRSSI(rawValues[0]);
        *packetDbm  = convertRSSI(rawValues[1]);
    }
    return statusCode;
}

int16_t EBYTE_E220_UART::getReceivedPacketRSSI()
{
    return _packetRssi;
}

// ===========================================================================
// Transmit
// ===========================================================================

uint8_t EBYTE_E220_UART::transmitRaw(const uint8_t* headerBuffer, uint8_t headerLength,
                                     const uint8_t* dataBuffer, uint8_t dataLength)
{
    if (_currentMode != E220_MODE_NORMAL && _currentMode != E220_MODE_WOR_TX) {
        return E220_ERR_WRONG_MODE;
    }
    if ((headerLength + dataLength) == 0 || (dataLength > 0 && dataBuffer == NULL) ||
        (headerLength > 0 && headerBuffer == NULL)) {
        return E220_ERR_INVALID_PARAM;
    }
    if (!waitForAux(E220_TX_TIMEOUT)) {
        return E220_ERR_AUX_TIMEOUT;
    }

    if (headerLength > 0) {
        _serialPort->write(headerBuffer, headerLength);
    }
    if (dataLength > 0) {
        _serialPort->write(dataBuffer, dataLength);
    }
    _serialPort->flush();

    if (_pinAUX < 0) {
        // No AUX: estimate UART time + air time (+ WOR preamble) and wait it out
        uint32_t byteCount = (uint32_t)headerLength + dataLength;
        uint32_t waitTime  = (byteCount * 10000UL) / getUARTBaudRateValue() +
                             (byteCount * 8000UL) / getAirDataRateValue() + 50UL;
        if (_currentMode == E220_MODE_WOR_TX) {
            waitTime += getWORCycleMs();
        }
        delay(waitTime);
        return E220_SUCCESS;
    }

    delay(E220_AUX_SETTLE_MS);
    return waitForAux(E220_TX_TIMEOUT) ? E220_SUCCESS : E220_ERR_AUX_TIMEOUT;
}

uint8_t EBYTE_E220_UART::sendMessage(const uint8_t* dataBuffer, uint8_t dataLength)
{
    if (dataBuffer == NULL || dataLength == 0 || dataLength > getSubPacketSizeValue()) {
        return E220_ERR_INVALID_PARAM;
    }
    return transmitRaw(NULL, 0, dataBuffer, dataLength);
}

uint8_t EBYTE_E220_UART::sendMessage(const char* textMessage)
{
    if (textMessage == NULL) {
        return E220_ERR_INVALID_PARAM;
    }
    size_t textLength = strlen(textMessage);
    if (textLength > E220_MAX_PACKET_SIZE) {
        return E220_ERR_INVALID_PARAM;
    }
    return sendMessage((const uint8_t*)textMessage, (uint8_t)textLength);
}

uint8_t EBYTE_E220_UART::sendFixedMessage(uint16_t destAddress, uint8_t destChannel,
                                          const uint8_t* dataBuffer, uint8_t dataLength)
{
    if (getTransmissionMethod() != E220_TX_FIXED) {
        return E220_ERR_NOT_ENABLED;
    }
    if (destChannel > getMaxChannel() || dataBuffer == NULL || dataLength == 0 ||
        dataLength > getSubPacketSizeValue()) {
        return E220_ERR_INVALID_PARAM;
    }
    uint8_t headerBuffer[3] = {
        (uint8_t)(destAddress >> 8),
        (uint8_t)(destAddress & 0xFF),
        destChannel
    };
    return transmitRaw(headerBuffer, sizeof(headerBuffer), dataBuffer, dataLength);
}

uint8_t EBYTE_E220_UART::sendFixedMessage(uint16_t destAddress, uint8_t destChannel,
                                          const char* textMessage)
{
    if (textMessage == NULL) {
        return E220_ERR_INVALID_PARAM;
    }
    size_t textLength = strlen(textMessage);
    if (textLength > E220_MAX_PACKET_SIZE) {
        return E220_ERR_INVALID_PARAM;
    }
    return sendFixedMessage(destAddress, destChannel, (const uint8_t*)textMessage,
                            (uint8_t)textLength);
}

uint8_t EBYTE_E220_UART::sendBroadcastMessage(uint8_t destChannel, const uint8_t* dataBuffer,
                                              uint8_t dataLength)
{
    return sendFixedMessage(E220_BROADCAST_ADDRESS, destChannel, dataBuffer, dataLength);
}

uint8_t EBYTE_E220_UART::sendBroadcastMessage(uint8_t destChannel, const char* textMessage)
{
    return sendFixedMessage(E220_BROADCAST_ADDRESS, destChannel, textMessage);
}

// ===========================================================================
// Receive
// ===========================================================================

int EBYTE_E220_UART::available()
{
    return _serialPort->available();
}

int EBYTE_E220_UART::read()
{
    return _serialPort->read();
}

uint16_t EBYTE_E220_UART::receiveMessage(uint8_t* dataBuffer, uint16_t maxLength, uint32_t timeoutMs)
{
    if (dataBuffer == NULL || maxLength == 0) {
        return 0;
    }

    uint32_t startTime = millis();
    while (!_serialPort->available()) {
        if (millis() - startTime >= timeoutMs) {
            return 0;
        }
        yield();
    }

    // The module holds AUX low while it outputs a packet, so the packet ends when
    // AUX is high and the line has been quiet for a gap. Without AUX, only the gap
    // is used. AUX stuck low is bounded by E220_DEFAULT_TIMEOUT.
    uint32_t gapLimit   = packetGapMs();
    uint32_t lastRxTime = millis();
    uint16_t totalCount = 0;
    uint8_t  lastByte   = 0;

    while (true) {
        if (_serialPort->available()) {
            lastByte = (uint8_t)_serialPort->read();
            if (totalCount < maxLength) {
                dataBuffer[totalCount] = lastByte;
            }
            if (totalCount < 0xFFFF) {
                totalCount++;
            }
            lastRxTime = millis();
        } else {
            uint32_t quietTime = millis() - lastRxTime;
            if ((quietTime >= gapLimit && !isBusy()) || quietTime >= E220_DEFAULT_TIMEOUT) {
                break;
            }
            yield();
        }
    }

    uint16_t dataCount = totalCount;
    if (getPacketRSSIEnabled() && totalCount > 0) {
        // Last byte of the burst is the RSSI byte appended by the module
        _packetRssi = convertRSSI(lastByte);
        dataCount   = totalCount - 1;
    }
    if (dataCount > maxLength) {
        dataCount = maxLength;
    }
    return dataCount;
}

// ===========================================================================
// Remote (wireless) configuration
// ===========================================================================

uint8_t EBYTE_E220_UART::sendRemoteConfig(uint8_t commandByte, uint8_t startReg,
                                          const uint8_t* dataBuffer, uint8_t dataLength,
                                          uint8_t* responseBuffer, uint8_t responseSize,
                                          uint8_t* responseLength, uint32_t timeoutMs)
{
    if (commandByte != E220_CMD_WRITE_PERMANENT && commandByte != E220_CMD_READ &&
        commandByte != E220_CMD_WRITE_TEMPORARY) {
        return E220_ERR_INVALID_PARAM;
    }
    // For a read command dataLength is the number of registers to read and no data is sent
    bool    isRead      = (commandByte == E220_CMD_READ);
    uint8_t payloadSize = isRead ? 0 : dataLength;
    if (dataLength == 0 || dataLength > E220_MAX_REG_BLOCK ||
        (payloadSize > 0 && dataBuffer == NULL)) {
        return E220_ERR_INVALID_PARAM;
    }
    if (responseLength != NULL) {
        *responseLength = 0;
    }

    uint8_t headerBuffer[5] = {
        E220_CMD_REMOTE_PREFIX, E220_CMD_REMOTE_PREFIX, commandByte, startReg, dataLength
    };

    clearInput();
    uint8_t statusCode = transmitRaw(headerBuffer, sizeof(headerBuffer), dataBuffer, payloadSize);
    if (statusCode != E220_SUCCESS || responseBuffer == NULL || responseSize == 0) {
        return statusCode;
    }

    uint16_t rxCount = receiveMessage(responseBuffer, responseSize, timeoutMs);
    if (responseLength != NULL) {
        *responseLength = (uint8_t)rxCount;
    }
    return (rxCount > 0) ? E220_SUCCESS : E220_ERR_TIMEOUT;
}

// ===========================================================================
// Diagnostics
// ===========================================================================

void EBYTE_E220_UART::printParameters(Print& outputPort)
{
    static const char* const parityNames[4] = { "8N1", "8O1", "8E1", "8N1" };
    static const char* const modelNames[4]  = {
        "E220-400T22", "E220-400T30", "E220-900T22", "E220-900T30"
    };

    outputPort.println(F("---------- E220 parameters ----------"));
    outputPort.print(F("Model            : ")); outputPort.println(modelNames[_moduleModel & 0x03]);
    outputPort.print(F("Raw registers    : "));
    for (uint8_t regIndex = 0; regIndex < E220_CONFIG_REG_COUNT; regIndex++) {
        if (_registers[regIndex] < 0x10) {
            outputPort.print('0');
        }
        outputPort.print(_registers[regIndex], HEX);
        outputPort.print(' ');
    }
    outputPort.println();
    outputPort.print(F("Address          : 0x")); outputPort.println(getAddress(), HEX);
    outputPort.print(F("Channel          : ")); outputPort.print(getChannel());
    outputPort.print(F(" (")); outputPort.print(getFrequency(), 3); outputPort.println(F(" MHz)"));
    outputPort.print(F("UART baud        : ")); outputPort.println(getUARTBaudRateValue());
    outputPort.print(F("UART parity      : ")); outputPort.println(parityNames[getParity()]);
    outputPort.print(F("Air data rate    : ")); outputPort.print(getAirDataRateValue());
    outputPort.println(F(" bps"));
    outputPort.print(F("Sub packet size  : ")); outputPort.print(getSubPacketSizeValue());
    outputPort.println(F(" bytes"));
    outputPort.print(F("Ambient RSSI     : "));
    outputPort.println(getAmbientRSSIEnabled() ? F("enabled") : F("disabled"));
    outputPort.print(F("Transmit power   : ")); outputPort.print(getTransmitPowerDbm());
    outputPort.println(F(" dBm"));
    outputPort.print(F("Packet RSSI byte : "));
    outputPort.println(getPacketRSSIEnabled() ? F("enabled") : F("disabled"));
    outputPort.print(F("Transmission     : "));
    outputPort.println(getTransmissionMethod() == E220_TX_FIXED ? F("fixed") : F("transparent"));
    outputPort.print(F("LBT              : "));
    outputPort.println(getLBTEnabled() ? F("enabled") : F("disabled"));
    outputPort.print(F("WOR cycle        : ")); outputPort.print(getWORCycleMs());
    outputPort.println(F(" ms"));
    outputPort.println(F("-------------------------------------"));
}

const char* EBYTE_E220_UART::statusToString(uint8_t statusCode)
{
    switch (statusCode) {
        case E220_SUCCESS:            return "Success";
        case E220_ERR_TIMEOUT:        return "Timeout";
        case E220_ERR_AUX_TIMEOUT:    return "AUX timeout";
        case E220_ERR_INVALID_PARAM:  return "Invalid parameter";
        case E220_ERR_FORMAT:         return "Command format error";
        case E220_ERR_BAD_RESPONSE:   return "Bad response";
        case E220_ERR_WRONG_MODE:     return "Wrong mode";
        case E220_ERR_NOT_ENABLED:    return "Feature not enabled";
        case E220_ERR_NOT_INITIALIZED:return "Not initialized";
        default:                      return "Unknown";
    }
}

// ===========================================================================
// Private helpers
// ===========================================================================

uint8_t EBYTE_E220_UART::enterConfigMode(uint8_t* previousMode)
{
    if (_serialPort == NULL) {
        return E220_ERR_NOT_INITIALIZED;
    }
    *previousMode = _currentMode;
    if (_currentMode == E220_MODE_CONFIG) {
        return E220_SUCCESS;
    }
    return setMode(E220_MODE_CONFIG);
}

uint8_t EBYTE_E220_UART::leaveConfigMode(uint8_t previousMode)
{
    if (previousMode == E220_MODE_CONFIG) {
        return E220_SUCCESS;
    }
    return setMode(previousMode);
}

uint8_t EBYTE_E220_UART::readResponse(uint8_t* respBuffer, uint8_t startReg, uint8_t dataLength,
                                      uint32_t timeoutMs)
{
    // Scan for the "C1 startReg dataLength" header (or the FF FF FF format error),
    // skipping stray bytes so a lost or extra byte cannot shift the reply
    const uint8_t headerBytes[3] = { E220_CMD_READ, startReg, dataLength };
    uint8_t  respTotal  = 3 + dataLength;
    uint8_t  rxCount    = 0;
    uint8_t  errorCount = 0;
    bool     anyRx      = false;
    uint32_t startTime  = millis();

    while (rxCount < respTotal) {
        if (_serialPort->available()) {
            uint8_t rxByte = (uint8_t)_serialPort->read();
            anyRx = true;
            if (rxCount < 3) {
                errorCount = (rxByte == E220_CMD_FORMAT_ERROR) ? errorCount + 1 : 0;
                if (errorCount == 3) {
                    return E220_ERR_FORMAT;
                }
                if (rxByte == headerBytes[rxCount]) {
                    respBuffer[rxCount++] = rxByte;
                } else if (rxByte == E220_CMD_READ) {
                    respBuffer[0] = rxByte;
                    rxCount       = 1;
                } else {
                    rxCount = 0;
                }
            } else {
                respBuffer[rxCount++] = rxByte;
            }
        } else if (millis() - startTime >= timeoutMs) {
            return anyRx ? E220_ERR_BAD_RESPONSE : E220_ERR_TIMEOUT;
        } else {
            yield();
        }
    }
    return E220_SUCCESS;
}

void EBYTE_E220_UART::clearInput()
{
    while (_serialPort->available()) {
        _serialPort->read();
    }
}

void EBYTE_E220_UART::drainInput(uint32_t quietMs)
{
    // Discard bytes until the line has been quiet for quietMs (bounded in total)
    uint32_t startTime  = millis();
    uint32_t lastRxTime = startTime;
    while (millis() - lastRxTime < quietMs || isBusy()) {
        if (millis() - startTime >= E220_DEFAULT_TIMEOUT) {
            break;
        }
        if (_serialPort->available()) {
            _serialPort->read();
            lastRxTime = millis();
        } else {
            yield();
        }
    }
}

void EBYTE_E220_UART::applySerialConfig(uint32_t baudRate, uint8_t parityMode)
{
    // Re-opening a port can drop received bytes on some cores (e.g. Teensy 3.x
    // end() underflows the UART FIFO), so only do it when the settings change
    if (parityMode == E220_PARITY_8N1_B) {
        parityMode = E220_PARITY_8N1;
    }
    if (_serialCallback != NULL && (baudRate != _hostBaud || parityMode != _hostParity)) {
        _serialCallback(baudRate, parityMode);
        _hostBaud   = baudRate;
        _hostParity = parityMode;
        clearInput();
    }
}

uint32_t EBYTE_E220_UART::packetGapMs()
{
    // Roughly three character times at the module's UART speed, plus margin
    return (30000UL / getUARTBaudRateValue()) + 5UL;
}

// The "C1" register read does not report the model name or firmware version,
// so these use the module's AT commands instead

char *EBYTE_E220_UART::readHardwareModel()
{
    if (!readATValue("AT+DEVTYPE=?\r\n", "DEVTYPE=", HardwareModel, sizeof(HardwareModel))) {
        return NULL;
    }
    // simple check to see if this is an E220
    if (strncmp(HardwareModel, "E220", 4) != 0) {
        return NULL;
    }
    return HardwareModel;
}

char *EBYTE_E220_UART::readVersion()
{
    if (!readATValue("AT+FWCODE=?\r\n", "FWCODE=", Version, sizeof(Version))) {
        return NULL;
    }
    return Version;     // no format check, the version string is informational only
}

bool EBYTE_E220_UART::readATValue(const char* atCommand, const char* replyPrefix,
                                  char* valueBuffer, uint8_t bufferSize)
{
    valueBuffer[0] = '\0';

    uint8_t previousMode;
    if (enterConfigMode(&previousMode) != E220_SUCCESS) {
        return false;
    }

    clearInput();
    _serialPort->print(atCommand);
    _serialPort->flush();

    // Read one reply line, ending at '\n' or on timeout. '\r' is dropped, and
    // characters that do not fit are consumed but discarded.
    uint8_t  charCount = 0;
    bool     lineDone  = false;
    uint32_t startTime = millis();

    while (!lineDone) {
        if (_serialPort->available()) {
            char rxChar = (char)_serialPort->read();
            if (rxChar == '\n') {
                lineDone = (charCount > 0);     // skip blank lines before the reply
            } else if (rxChar != '\r' && charCount < bufferSize - 1) {
                valueBuffer[charCount++] = rxChar;
            }
        } else if (millis() - startTime >= E220_DEFAULT_TIMEOUT) {
            break;
        } else {
            yield();
        }
    }
    valueBuffer[charCount] = '\0';

    leaveConfigMode(previousMode);

    size_t prefixLength = strlen(replyPrefix);
    if (strncmp(valueBuffer, replyPrefix, prefixLength) == 0) {
        memmove(valueBuffer, valueBuffer + prefixLength, strlen(valueBuffer + prefixLength) + 1);
    }
    return valueBuffer[0] != '\0';
}
