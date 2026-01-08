import board
import busio
from adafruit_pca9685 import PCA9685
from time import sleep

# I2C初期化
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 1000  # LED用は高めでもOK

channel0 = pca.channels[0]  # チャンネル0を使用
channel1 = pca.channels[1]  # チャンネル1を使用
channel4 = pca.channels[4]  # チャンネル1を使用

try:
    while True:
        # フェードイン
        for duty in range(0, 65536, 512):  # 0〜65535を512刻み
            channel0.duty_cycle = duty
            channel1.duty_cycle = duty
            channel4.duty_cycle = duty
            sleep(0.01)  # 10ms間隔で徐々に明るく

        # フェードアウト
        for duty in range(65535, -1, -512):
            channel0.duty_cycle = duty
            channel1.duty_cycle = duty
            channel4.duty_cycle = duty
            sleep(0.01)

except KeyboardInterrupt:
    # Ctrl+Cで停止したら消灯
    channel0.duty_cycle = 0
    channel1.duty_cycle = 0
    channel4.duty_cycle = 0
    print("LEDフェード終了")
