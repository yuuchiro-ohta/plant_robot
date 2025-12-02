import gpiod
from gpiod.line_settings import LineSettings, Direction, Value
import time

# ===== GPIO 設定 =====
CHIP = "/dev/gpiochip4"

# モータ1方向ピン
M1_IN1 = 17
M1_IN2 = 27

# モータ2方向ピン
M2_IN1 = 23
M2_IN2 = 22

chip = gpiod.Chip(CHIP)
settings = LineSettings()
settings.direction = Direction.OUTPUT

m1_in1_line = chip.request_lines(consumer="motor", config={M1_IN1: settings})
m1_in2_line = chip.request_lines(consumer="motor", config={M1_IN2: settings})
m2_in1_line = chip.request_lines(consumer="motor", config={M2_IN1: settings})
m2_in2_line = chip.request_lines(consumer="motor", config={M2_IN2: settings})

def motor1_forward():
    m1_in1_line.set_value(M1_IN1, Value.INACTIVE)
    m1_in2_line.set_value(M1_IN2, Value.ACTIVE)

def motor1_backward():
    m1_in1_line.set_value(M1_IN1, Value.ACTIVE)
    m1_in2_line.set_value(M1_IN2, Value.INACTIVE)

def motor1_stop():
    m1_in1_line.set_value(M1_IN1, Value.INACTIVE)
    m1_in2_line.set_value(M1_IN2, Value.INACTIVE)

def motor2_forward():
    m2_in1_line.set_value(M2_IN1, Value.INACTIVE)
    m2_in2_line.set_value(M2_IN2, Value.ACTIVE)

def motor2_backward():
    m2_in1_line.set_value(M2_IN1, Value.ACTIVE)
    m2_in2_line.set_value(M2_IN2, Value.INACTIVE)

def motor2_stop():
    m2_in1_line.set_value(M2_IN1, Value.INACTIVE)
    m2_in2_line.set_value(M2_IN2, Value.INACTIVE)

def stop():
    motor1_stop()
    motor2_stop()

def forward_motion():
    motor1_forward()
    motor2_forward()

def back_motion():
    motor1_backward()
    motor2_backward()

def right_motion():
    motor1_forward()
    motor2_backward()

def left_motion():
    motor1_backward()
    motor2_forward()
