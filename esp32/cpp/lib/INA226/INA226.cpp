#include "INA226.h"

INA226::INA226(uint8_t addr) : _addr(addr), _wire(nullptr) {}

bool INA226::begin(TwoWire *wire) {
    _wire = wire;
    _wire->begin();
    
    // Reset
    writeRegister(REG_CONFIG, 0x8000);
    delay(20);
    
    // CONFIG: AVG=16, VBUSCT=1.1ms, VSHCT=1.1ms, MODE=cont shunt+bus
    uint16_t config = (0b010 << 9) | (0b100 << 6) | (0b100 << 3) | 0b111;
    writeRegister(REG_CONFIG, config);
    delay(6);
    
    // Set calibration (from your MicroPython code)
    setCalibration(0x23F8);
    delay(4);
    
    // Dummy read to clear
    readRegister(REG_CURRENT);
    delay(3);
    
    return true;
}

float INA226::readVoltage() {
    uint16_t raw = readRegister(REG_BUS_VOLT);
    return raw * VBUS_LSB * VBUS_SCALE;
}

float INA226::readCurrent() {
    uint16_t raw = readRegister(REG_CURRENT);
    int16_t signed_raw = (int16_t)raw;
    return signed_raw * CURRENT_LSB * 2.65; // I_USER_SCALE from your code
}

float INA226::readPower() {
    uint16_t raw = readRegister(REG_POWER);
    return raw * POWER_LSB;
}

float INA226::readShuntVoltage() {
    uint16_t raw = readRegister(REG_SHUNT_VOLT);
    int16_t signed_raw = (int16_t)raw;
    return signed_raw * VSHUNT_LSB;
}

void INA226::setCalibration(uint16_t cal) {
    writeRegister(REG_CALIB, cal);
}

uint16_t INA226::getCalibration() {
    return readRegister(REG_CALIB);
}

void INA226::writeRegister(uint8_t reg, uint16_t value) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write((value >> 8) & 0xFF);
    _wire->write(value & 0xFF);
    _wire->endTransmission();
}

uint16_t INA226::readRegister(uint8_t reg) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission(false);
    
    _wire->requestFrom(_addr, (uint8_t)2);
    uint16_t value = _wire->read() << 8;
    value |= _wire->read();
    return value;
}
