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

# ===== モータ制御関数 (PWMなし) =====
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

# ===== 回転動作 =====
def right_turn():
    motor1_forward()
    motor2_backward()

def left_turn():
    motor1_backward()
    motor2_forward()

# ===== 動作テスト =====
try:
    while True:
        print("前進")
        motor1_forward()
        motor2_forward()
        time.sleep(0.5)

        print("後退")
        motor1_backward()
        motor2_backward()
        time.sleep(0.5)

        print("停止")
        motor1_stop()
        motor2_stop()
        time.sleep(0.5)

        print("右回転")
        right_turn()
        time.sleep(0.5)

        print("左回転")
        left_turn()
        time.sleep(0.5)

        print("停止")
        motor1_stop()
        motor2_stop()
        time.sleep(1)

except KeyboardInterrupt:
    motor1_stop()
    motor2_stop()
    print("終了")

finally:
    m1_in1_line.release()
    m1_in2_line.release()
    m2_in1_line.release()
    m2_in2_line.release()
