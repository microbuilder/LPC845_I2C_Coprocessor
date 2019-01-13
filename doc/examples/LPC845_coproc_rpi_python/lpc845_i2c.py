import sys
import time
import smbus

I2C_BUS = 1

LPC845_I2C_ADDR = 0x7E
LPC845_OPCODE_WHOAMI = 0x01
LPC845_OPCODE_VERSION = 0x02
LPC845_OPCODE_ADC_HI = 0x11
LPC845_OPCODE_ADC_LO = 0x12
LPC845_OPCODE_ADC_STAT = 0x13

# Declare an instance of the I2C bus
bus = smbus.SMBus(I2C_BUS)

# Check the WHOAMI register and read ADC if the co-processor was found
whoami = bus.read_byte_data(LPC845_I2C_ADDR, LPC845_OPCODE_WHOAMI)
if whoami != 0X45:
    print("Unable to find the LPC845 I2C co-processor. Check pullups?")
    sys.exit()

# Read from the ADC every second until program execution stops
while True:
    adc_hi = bus.read_byte_data(LPC845_I2C_ADDR, LPC845_OPCODE_ADC_HI)
    adc_lo = bus.read_byte_data(LPC845_I2C_ADDR, LPC845_OPCODE_ADC_LO)
    adc = adc_hi<<8 | adc_lo
    print("ADC Output: ", adc)
    time.sleep(1.0)
