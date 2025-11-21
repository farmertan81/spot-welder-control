#ifndef INA226_H
#define INA226_H

#include <Arduino.h>
#include <Wire.h>

// INA226 Registers
#define INA226_REG_CONFIG       0x00
#define INA226_REG_SHUNT_V      0x01
#define INA226_REG_BUS_V        0x02
#define INA226_REG_POWER        0x03
#define INA226_REG_CURRENT      0x04
#define INA226_REG_CALIBRATION  0x05

class INA226 {
public:
    INA226(TwoWire* wire, uint8_t addr);
    bool begin();
    void configure(uint16_t config);
    void calibrate(uint16_t cal_value);
    
    float readBusVoltage();      // Pack voltage
    float readShuntVoltage();    // Shunt voltage (mV)
    float readCurrent();         // Current (A)
    float readPower();           // Power (W)
    
private:
    TwoWire* _wire;
    uint8_t _addr;
    
    void writeRegister(uint8_t reg, uint16_t value);
    uint16_t readRegister(uint8_t reg);
};

#endif
