# This will be inserted into main.py

# Add after line 16 (after other imports):
# Import ADS1256 driver
try:
    import ads1256_esp32
    from machine import SPI
    ADS1256_AVAILABLE = True
except:
    ADS1256_AVAILABLE = False
    print("ADS1256 driver not found - current measurement disabled")

# Add after thermistor section (around line 90):
# ---- ADS1256 Current Sensor ----
if ADS1256_AVAILABLE:
    try:
        # Initialize SPI for ADS1256
        spi_ads = SPI(1, baudrate=1000000, polarity=0, phase=1,
                      sck=Pin(40), mosi=Pin(39), miso=Pin(38))
        
        # Initialize ADS1256 (bottom pins: CS=17, DRDY=18)
        adc_current = ads1256_esp32.ADS1256(spi_ads, cs_pin=17, drdy_pin=18)
        print("✓ ADS1256 current sensor initialized")
    except Exception as e:
        print(f"ADS1256 init failed: {e}")
        ADS1256_AVAILABLE = False
        adc_current = None
else:
    adc_current = None

# Weld data collection
weld_samples = []
MAX_WELD_SAMPLES = 1000  # Buffer for up to 1000 samples (~33ms at 30kHz)

