#include "ADS1256.h"

ADS1256::ADS1256(SPIClass* spi, uint8_t cs_pin, uint8_t drdy_pin) {
    _spi = spi;
    _cs_pin = cs_pin;
    _drdy_pin = drdy_pin;
    
    // Calculate voltage per code
    volts_per_code = (2.0 * Vref) / (FS * 2.0 * gain_adc);
}

bool ADS1256::begin() {
    pinMode(_cs_pin, OUTPUT);
    pinMode(_drdy_pin, INPUT);
    csHigh();
    
    delay(100);
    
    // Reset
    writeCmd(CMD_RESET);
    delay(100);
    
    // Wait for DRDY
    waitDRDY();
    
    // Stop continuous read
    writeCmd(CMD_SDATAC);
    delayMicroseconds(100);
    
    // Configure MUX for AIN2-AIN3
    writeReg(REG_MUX, MUX_AIN2_AIN3);
    
    // Configure ADCON (PGA=1, buffer disabled)
    writeReg(REG_ADCON, 0x00);
    
    // Set data rate to 30kSPS
    writeReg(REG_DRATE, DRATE_30000SPS);
    
    // Self calibration
    writeCmd(CMD_SELFCAL);
    delay(500);
    
    Serial.println("✓ ADS1256 initialized");
    return true;
}

int32_t ADS1256::readRaw() {
    waitDRDY();
    
    csLow();
    _spi->transfer(CMD_RDATA);
    delayMicroseconds(7);  // t6 delay
    
    uint8_t data[3];
    data[0] = _spi->transfer(0x00);
    data[1] = _spi->transfer(0x00);
    data[2] = _spi->transfer(0x00);
    csHigh();
    
    // Convert to signed 24-bit
    int32_t result = ((int32_t)data[0] << 16) | ((int32_t)data[1] << 8) | data[2];
    
    // Sign extend
    if (result & 0x800000) {
        result |= 0xFF000000;
    }
    
    return result;
}

float ADS1256::readCurrent() {
    int32_t raw = readRaw();
    float voltage = raw * volts_per_code;
    float current = (voltage / G_AMC) / Rsh;
    return current;
}

void ADS1256::writeCmd(uint8_t cmd) {
    csLow();
    _spi->transfer(cmd);
    csHigh();
    delayMicroseconds(1);
}

void ADS1256::writeReg(uint8_t reg, uint8_t value) {
    csLow();
    _spi->transfer(CMD_WREG | reg);
    _spi->transfer(0x00);  // Write 1 register
    _spi->transfer(value);
    csHigh();
    delayMicroseconds(1);
}

uint8_t ADS1256::readReg(uint8_t reg) {
    csLow();
    _spi->transfer(CMD_RREG | reg);
    _spi->transfer(0x00);  // Read 1 register
    delayMicroseconds(7);
    uint8_t value = _spi->transfer(0x00);
    csHigh();
    return value;
}

void ADS1256::waitDRDY(uint32_t timeout_us) {
    uint32_t start = micros();
    while (digitalRead(_drdy_pin) == HIGH) {
        if (micros() - start > timeout_us) {
            Serial.println("⚠ ADS1256 DRDY timeout");
            return;
        }
    }
}

void ADS1256::csLow() {
    digitalWrite(_cs_pin, LOW);
    delayMicroseconds(1);
}

void ADS1256::csHigh() {
    digitalWrite(_cs_pin, HIGH);
    delayMicroseconds(1);
}
