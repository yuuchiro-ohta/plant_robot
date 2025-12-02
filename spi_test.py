import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000

def read_mcp3002(channel=0):
    cmd = 0b01100000 | (channel << 4)
    resp = spi.xfer2([cmd, 0x00])
    value = ((resp[0] & 0x03) << 8) | resp[1]
    return value

while True:
    print("CH0 =", read_mcp3002(0))
    print("CH1 =", read_mcp3002(1))
    time.sleep(0.2)