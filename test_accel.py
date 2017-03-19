import smbus
DEVICE_ADDRESS = 0x1c
MMA8451_REG_WHOAMI = 0x0D
bus = smbus.SMBus(1)
ret = bus.read_byte_data(DEVICE_ADDRESS, MMA8451_REG_WHOAMI)
print ret
