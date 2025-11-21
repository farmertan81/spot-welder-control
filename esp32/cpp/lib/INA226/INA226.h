#ifndef INA226_H
#define INA226_H

#include <Wire.h>

class INA226 {
public:
    INA226(uint8_t addr = 0x40);
    bool begin(TwoWire *wire = &Wire);
    
    float readVoltage();      // Bus voltage in V
    float readCurrent();      // Current in A
    float readPower();        // Power in W
    float readShuntVoltage(); // Shunt voltage in V
    
    void setCalibration(uint16_t cal);
    uint16_t getCalibration();
    
private:
    uint8_t _addr;
    TwoWire *_wire;
    
    void writeRegister(uint8_t reg, uint16_t value);
    uint16_t readRegister(uint8_t reg);
    
    static constexpr uint8_t REG_CONFIG = 0x00;
    static constexpr uint8_t REG_SHUNT_VOLT = 0x01;
    static constexpr uint8_t REG_BUS_VOLT = 0x02;
    static constexpr uint8_t REG_POWER = 0x03;
    static constexpr uint8_t REG_CURRENT = 0x04;
    static constexpr uint8_t REG_CALIB = 0x05;
    
    static constexpr float VSHUNT_LSB = 2.5e-6;
    static constexpr float VBUS_LSB = 0.00125;
    static constexpr float CURRENT_LSB = 0.00278;
    static constexpr float POWER_LSB = 0.0695;  // 25 * CURRENT_LSB
    static constexpr float VBUS_SCALE = 0.9982; // For 0x40
};

#endif
