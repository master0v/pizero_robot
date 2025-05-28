# pip install adafruit-ads1x15

import Adafruit_ADS1x15

adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1

# Read raw value from ADC channel 0
value = adc.read_adc(0, gain=GAIN)

# Convert to voltage (depends on your voltage divider and ADC resolution)
# Example: If using 10k and 10k resistors for divider, and 3.3V reference
voltage = value * 4.096 / 32767 * 2  # adjust multiplier based on your setup

print(f"Battery Voltage: {voltage:.2f} V")
