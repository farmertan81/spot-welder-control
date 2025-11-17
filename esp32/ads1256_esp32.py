# ADS1256 Driver for ESP32 MicroPython
# 24-bit ADC with SPI interface and DRDY interrupt

from machine import Pin, SPI
import time

# ADS1256 Commands
CMD_WAKEUP   = 0x00
CMD_RDATA    = 0x01
CMD_RDATAC   = 0x03
CMD_SDATAC   = 0x0F
CMD_RREG     = 0x10
CMD_WREG     = 0x50
CMD_SELFCAL  = 0xF0
CMD_SYNC     = 0xFC
CMD_RESET    = 0xFE

# ADS1256 Registers
REG_STATUS   = 0x00
REG_MUX      = 0x01
REG_ADCON    = 0x02
REG_DRATE    = 0x03

# Data rates
DRATE_30000SPS = 0xF0
DRATE_15000SPS = 0xE0
DRATE_7500SPS  = 0xD0
DRATE_3750SPS  = 0xC0
DRATE_2000SPS  = 0xB0
DRATE_1000SPS  = 0xA1
DRATE_500SPS   = 0x92
DRATE_100SPS   = 0x82
DRATE_60SPS    = 0x72
DRATE_50SPS    = 0x63
DRATE_30SPS    = 0x53
DRATE_25SPS    = 0x43
DRATE_15SPS    = 0x33
DRATE_10SPS    = 0x23
DRATE_5SPS     = 0x13
DRATE_2_5SPS   = 0x03

# PGA Gain settings
PGA_GAIN_1  = 0x00
PGA_GAIN_2  = 0x01
PGA_GAIN_4  = 0x02
PGA_GAIN_8  = 0x03
PGA_GAIN_16 = 0x04
PGA_GAIN_32 = 0x05
PGA_GAIN_64 = 0x06

class ADS1256:
    def __init__(self, spi, cs_pin, drdy_pin):
        self.spi = spi
        self.cs = Pin(cs_pin, Pin.OUT, value=1)
        self.drdy = Pin(drdy_pin, Pin.IN)
        
        self.vref = 2.5  # Reference voltage
        self.gain = 1
        
        # Initialize
        time.sleep_ms(100)
        self.reset()
        self.wait_drdy()
        
    def wait_drdy(self, timeout_ms=1000):
        """Wait for DRDY to go low (data ready)"""
        start = time.ticks_ms()
        while self.drdy.value() == 1:
            if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                return False
        return True
    
    def reset(self):
        """Software reset"""
        self.cs.value(0)
        self.spi.write(bytes([CMD_RESET]))
        self.cs.value(1)
        time.sleep_ms(100)
        
    def write_reg(self, reg, value):
        """Write to a register"""
        self.cs.value(0)
        self.spi.write(bytes([CMD_WREG | reg, 0x00, value]))
        self.cs.value(1)
        
    def read_reg(self, reg):
        """Read from a register"""
        self.cs.value(0)
        self.spi.write(bytes([CMD_RREG | reg, 0x00]))
        result = self.spi.read(1)
        self.cs.value(1)
        return result[0]
    
    def set_channel(self, pos_ch, neg_ch=0x08):
        """Set input channel (neg_ch=0x08 for AINCOM)"""
        mux = (pos_ch << 4) | neg_ch
        self.write_reg(REG_MUX, mux)
        
    def set_data_rate(self, drate):
        """Set data rate"""
        self.write_reg(REG_DRATE, drate)
        
    def set_pga_gain(self, gain):
        """Set PGA gain (0-6 for 1x to 64x)"""
        self.gain = 1 << gain
        self.write_reg(REG_ADCON, 0x20 | gain)  # 0x20 = CLKOUT off, sensor detect off
        
    def calibrate(self):
        """Perform self-calibration"""
        self.cs.value(0)
        self.spi.write(bytes([CMD_SELFCAL]))
        self.cs.value(1)
        self.wait_drdy(timeout_ms=2000)
        
    def read_raw(self):
        """Read 24-bit raw ADC value"""
        if not self.wait_drdy():
            return None
            
        self.cs.value(0)
        self.spi.write(bytes([CMD_RDATA]))
        time.sleep_us(7)  # t6 delay
        
        data = self.spi.read(3)
        self.cs.value(1)
        
        # Convert 24-bit two's complement to signed integer
        raw = (data[0] << 16) | (data[1] << 8) | data[2]
        if raw & 0x800000:
            raw -= 0x1000000
        
        return raw
    
    def read_voltage(self):
        """Read voltage in volts"""
        raw = self.read_raw()
        if raw is None:
            return None
        
        # Convert to voltage
        # Full scale = ±VREF/GAIN
        voltage = (raw / 8388608.0) * (self.vref / self.gain)
        return voltage

