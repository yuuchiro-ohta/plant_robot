# ビルド前
gitクローンした直後だと、IMUやLiDARなどデバイスが記述されているため、ラズパイに接続されていない状態だとビルドが失敗する。docker-composeファイルに記述されているため、不要な場合はコメントアウトしておく。

```yml
    devices:
      - /dev/snd:/dev/snd
      #- /dev/imu_bwt901cl:/dev/imu_bwt901cl
      #- /dev/lidar_ydlidar:/dev/lidar_ydlidar
      - /dev/ttyAMA0:/dev/ttyAMA0
      - /dev/gpiochip0
      - /dev/gpiochip4
```

# 初期設定

## .envファイルを作成する。
env_exampleをコピーして.envファイルを作成する。

```t
# 録音デバイスの番号に合わせる
# 例
ARECORD_CARD = 1
ARECORD_DEVICE = 0

# ollamaのパスとモデルを合わせる
# 例
OLLAMA_PATH = http://localhost:11434/api/generate
OLLAMA_MODEL = gemma2:2b
```

## 音声・テキストの一時保管ファイルを作成する
下記のディレクトリを作成し、ファイルを作成する
/app/voice/generate_text/input.txt
/app/voice/input_voice/input.wav
/app/voice/output_voice/output.wav


## main.pyを起動する

```bash
python3 main.py
```

## スピーカーの音量を上げたい場合

```
amixer set PCM 80%
```

## ROSの使い方

端末1
```bash 
cd ros2_ws
colcon build
source install/setup.bash
ros2 run robot_controller action
```

端末2
```bash
# 前進
ros2 topic pub /motion_command std_msgs/String "data: 'forward'" --once
# 後退
ros2 topic pub /motion_command std_msgs/String "data: 'back'" --once
# 左回転
ros2 topic pub /motion_command std_msgs/String "data: 'left'" --once
# 右回転
ros2 topic pub /motion_command std_msgs/String "data: 'right'" --once
# 停止（モータ動作停止）
ros2 topic pub /motion_command std_msgs/String "data: 'stop'" --once
# 会話モード（モータ動作停止）
ros2 topic pub /motion_command std_msgs/String "data: 'talk'" --once
```