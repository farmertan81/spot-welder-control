#include "ADS1256.h"

// ---- ADS1256 command & register definitions ----
enum {
    CMD_WAKEUP  = 0x00,
    CMD_RDATA   = 0x01,
    CMD_RDATAC  = 0x03,
    CMD_SDATAC  = 0x0F,
    CMD_RREG    = 0x10,
    CMD_WREG    = 0x50,
    CMD_SELFCAL = 0xF0,
    CMD_RESET   = 0xFE
};

enum {
    REG_STATUS = 0x00,
    REG_MUX    = 0x01,
    REG_ADCON  = 0x02,
    REG_DRATE  = 0x03,
};

Ads1256::Ads1256(int csPin, int drdyPin)
    : _cs(csPin), _drdy(drdyPin), _spi(nullptr)
{}

bool Ads1256::begin(SPIClass &spi) {
    _spi = &spi;

    pinMode(_cs, OUTPUT);
    pinMode(_drdy, INPUT);
    csHigh();

    // Basic reset
    csLow();
    delayMicroseconds(5);
    sendCommand(CMD_RESET);
    delay(2);
    sendCommand(CMD_SDATAC);  // stop any continuous conversion
    csHigh();
    delay(2);

    // Example config: 1000SPS, PGA=1
    csLow();
    writeRegister(REG_DRATE, 0x82);   // data rate (approx 1000SPS)
    csHigh();

    csLow();
    writeRegister(REG_ADCON, 0x20);   // gain=1, clock out off
    csHigh();

    return true;
}

void Ads1256::startContinuous(uint8_t ch) {
    if (!_spi) return;

    // Single‑ended: AINp = ch, AINn = AINCOM (0x08)
    uint8_t mux = (ch << 4) | 0x08;
    csLow();
    writeRegister(REG_MUX, mux);
    csHigh();
    delayMicroseconds(10);

    csLow();
    sendCommand(CMD_RDATAC);
    csHigh();
}

void Ads1256::stopContinuous() {
    if (!_spi) return;
    csLow();
    sendCommand(CMD_SDATAC);
    csHigh();
}

bool Ads1256::readCurrentFast(float &amps) {
    if (!_spi) return false;
    if (!drdyLow()) return false;   // no new sample yet

    long raw = readData();

    // Placeholder scaling: we will replace this with your exact math
    const float VREF   = 2.5f;      // adjust later
    const float GAIN   = 16.0f;     // adjust to match your setup
    const float RSHUNT = 0.00005f;  // 50 µΩ new shunt

    float volts = (raw / 8388607.0f) * (VREF / GAIN); // ±Vref/gain full‑scale
    amps = volts / RSHUNT;
    return true;
}

// ---- Low‑level helpers ----

void Ads1256::sendCommand(uint8_t cmd) {
    _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    _spi->transfer(cmd);
    _spi->endTransaction();
}

void Ads1256::writeRegister(uint8_t reg, uint8_t value) {
    _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    _spi->transfer(CMD_WREG | (reg & 0x0F));
    _spi->transfer(0x00);     // write 1 register
    _spi->transfer(value);
    _spi->endTransaction();
    delayMicroseconds(5);
}

uint8_t Ads1256::readRegister(uint8_t reg) {
    _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    _spi->transfer(CMD_RREG | (reg & 0x0F));
    _spi->transfer(0x00);     // read 1 register
    delayMicroseconds(5);
    uint8_t v = _spi->transfer(0xFF);
    _spi->endTransaction();
    return v;
}

long Ads1256::readData() {
    _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    _spi->transfer(CMD_RDATA);
    delayMicroseconds(5);
    uint8_t b0 = _spi->transfer(0xFF);
    uint8_t b1 = _spi->transfer(0xFF);
    uint8_t b2 = _spi->transfer(0xFF);
    _spi->endTransaction();

    long raw = ((long)b0 << 16) | ((long)b1 << 8) | b2;
    if (raw & 0x800000) {
        raw |= 0xFF000000;   // sign‑extend 24‑bit
    }
    return raw;
}
