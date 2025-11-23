import gpiod
from gpiod.line_settings import LineSettings, Direction, Value
import time

CHIP = "/dev/gpiochip4"

RED_GPIO = 17
GREEN_GPIO = 18

chip = gpiod.Chip(CHIP)

settings = LineSettings()
settings.direction = Direction.OUTPUT


red_led = chip.request_lines(
    consumer="blink",
    config={RED_GPIO: settings}
)

green_led = chip.request_lines(
    consumer="blink",
    config={GREEN_GPIO: settings}
)


try:
    while True:
        red_led.set_value(RED_GPIO, Value.ACTIVE)   # 1 の代わりに Value.ACTIVE
        time.sleep(0.5)
        red_led.set_value(RED_GPIO, Value.INACTIVE) # 0 の代わりに Value.INACTIVE
        time.sleep(0.5)
        green_led.set_value(GREEN_GPIO, Value.ACTIVE)   # 1 の代わりに Value.ACTIVE
        time.sleep(0.5)
        green_led.set_value(GREEN_GPIO, Value.INACTIVE) # 0 の代わりに Value.INACTIVE
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
finally:
    green_led.release()
    red_led.release()
