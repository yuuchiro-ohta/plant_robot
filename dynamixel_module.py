import serial
import time,math

# シリアルポートとボーレート
ser = serial.Serial('/dev/ttyAMA0', baudrate=1000000, timeout=1)

def crc16(data, poly=0x8005, init_crc=0x0000, reflect_data=False, reflect_crc=False):
    crc = init_crc
    for byte in data:
        if reflect_data:
            byte = reflect_byte(byte)
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ poly
            else:
                crc <<= 1
            crc &= 0xFFFF
    if reflect_crc:
        crc = reflect_byte(crc >> 8) | (reflect_byte(crc & 0xFF) << 8)
    return crc

def angle_to_data(angle):
    """
    0〜360度の角度を0〜4095のデータに変換する関数
    
    Parameters:
    angle (int): 0〜360度の範囲のデータ
    
    Returns:
    int: 0〜4095の範囲のデータ
    """
    if angle < 0 or angle >= 360:
        raise ValueError("Angle must be between 0 and 360 degrees")
    return int((angle / 360) * 4095)

def parse_dynamixel_angle(response_bytes):
    """
    Dynamixelのステータスパケットから現在位置を読み取り、角度に変換して返す。

    Parameters:
        response_bytes (bytes or bytearray): 例) b'\xFF\xFF\xFD\x00\x01\x08\x00\x55\x00\x0B\x08\x00\x00\x1F\x24'

    Returns:
        tuple: (position, angle_deg)
            - position: 現在位置（0〜4095）
            - angle_deg: 角度（0〜360°）
    """
    if len(response_bytes) < 15:
        raise ValueError("不正なパケット長")

    # パラメータ（現在位置）部分は index 9〜12（0始まり）
    pos_bytes = response_bytes[9:13]  # 4バイト
    position = pos_bytes[0] | (pos_bytes[1] << 8) | (pos_bytes[2] << 16) | (pos_bytes[3] << 24)

    # 4095スケールで角度に変換
    angle_deg = position / 4095 * 360

    return position, angle_deg

def parse_dynamixel_velocity(response_bytes):
    """
    Dynamixelのステータスパケットから回転速度（Present Velocity）を抽出し、rpmに変換。

    Parameters:
        response_bytes (bytes or bytearray): ステータスパケット
            例: b'\xFF\xFF\xFD\x00\x01\x08\x00\x55\x00\xE7\xFF\xFF\xFF\x93\x58'

    Returns:
        tuple: (raw_velocity, velocity_rpm)
            - raw_velocity: signed int (-2^31〜2^31-1)
            - velocity_rpm: 回転速度 [rpm]
    """
    if len(response_bytes) < 15:
        raise ValueError("パケット長が不正です")

    # パラメータ部（9〜12バイト目）をリトルエンディアンでsigned intに変換
    param_bytes = response_bytes[9:13]
    raw_velocity = int.from_bytes(param_bytes, byteorder='little', signed=True)

    # 単位換算: 1 unit ≈ 0.229 rpm（XL330基準）
    velocity_rpm = raw_velocity * 0.229

    return raw_velocity, velocity_rpm

# ダイナミクセルのLEDを点灯/消灯させる関数
def dynamixel_lED_control(id, data):
    # コマンドパケットを作成
    header = bytearray([0xFF, 0xFF, 0xFD, 0x00])
    id_byte = bytearray([id])
    Length=bytearray([0x06, 0x00])
    command=bytearray([0x03])
    if(data==1):
        LED_list =bytearray([0x41, 0x00, 0x01])
        check_sum=bytearray([0xCC, 0xE6])
    else:
        LED_list =bytearray([0x41, 0x00, 0x00])
        check_sum=bytearray([0xC9, 0x66])
    
    packet = header + id_byte + Length + command  + LED_list
    
    # 設定でCRC計算
    settings = {
        "poly": 0x8005, 
        "init_crc": 0x0000, 
        "reflect_data": False, 
        "reflect_crc": False
    }    
    #チェックサムの計算
    crc = crc16(packet, **settings)
    lower_byte = crc & 0xFF
    upper_byte = (crc >> 8) & 0xFF
    crc_bytes = bytearray([lower_byte, upper_byte])
    packet.extend(crc_bytes)
    
    # コマンドパケットを送信  
    ser.write(packet)
    time.sleep(0.05)

    #print(' '.join(f'{byte:02X}' for byte in packet))

# ダイナミクセルのトルクをON/OFFさせる関数
def dynamixel_torque_control(id, data):
    # コマンドパケットを作成
    header = bytearray([0xFF, 0xFF, 0xFD, 0x00])
    id_byte = bytearray([id])
    Length=bytearray([0x06, 0x00])
    command=bytearray([0x03])
    if(data==1):
        Tortque_list =bytearray([0x40, 0x00, 0x01])
    else:
        Tortque_list =bytearray([0x40, 0x00, 0x00])
    packet = header + id_byte + Length  + command  + Tortque_list
    
    # 設定でCRC計算
    settings = {
        "poly": 0x8005, 
        "init_crc": 0x0000, 
        "reflect_data": False, 
        "reflect_crc": False
    }    
    #チェックサムの計算
    crc = crc16(packet, **settings)
    lower_byte = crc & 0xFF
    upper_byte = (crc >> 8) & 0xFF
    crc_bytes = bytearray([lower_byte, upper_byte])
    packet.extend(crc_bytes)
    
    # コマンドパケットを送信  
    ser.write(packet)
    time.sleep(0.05)

    #print(' '.join(f'{byte:02X}' for byte in packet))

# 回転させる関数
def dynamixel_position_rotate(id, angle):
    """
    ダイナミクセルを回転させるためのパケット送信用の関数
    （トルクをONする必要あり）
    Parameters:
    id: ダイナミクセルのID (int(1～N))
    angle: float (0～359.91)  
    header: command[0]～[3] # ヘッダ(固定値)
    id_byte: ダイナミクセルのID
    Length: command以降のすべてのバイト数を指定
    command: Instruction write[0x03]
    Position_list:1つめの要素:アドレス[0x74] GoalPosition
                  3～4つめの要素: 0～4095の16進数が入る
    crc_bytes：headerからPosition_listまでのチェックサム値(CRC-16-IBM)。
    Returns:なし（コマンドパケットの送信）
    """  
    # コマンドパケットを作成
    header = bytearray([0xFF, 0xFF, 0xFD, 0x00])
    id_byte = bytearray([id])
    Length=bytearray([0x09, 0x00])
    command=bytearray([0x03])
    angle_data = angle_to_data(angle)
    low_angle_data = angle_data & 0xFF
    high_angle_data = (angle_data >> 8) & 0xFF
    Position_list =bytearray([0x74, 0x00, 0x00, 0x04, 0x00, 0x00])
    # Position_listにデータを格納
    Position_list[2] = low_angle_data
    Position_list[3] = high_angle_data    
    packet = header + id_byte + Length  + command  + Position_list
    
    # 設定でCRC計算
    settings = {
        "poly": 0x8005, 
        "init_crc": 0x0000, 
        "reflect_data": False, 
        "reflect_crc": False
    }
    
    #CRCの計算
    crc = crc16(packet, **settings)
    lower_byte = crc & 0xFF
    upper_byte = (crc >> 8) & 0xFF
    crc_bytes = bytearray([lower_byte, upper_byte])
    packet.extend(crc_bytes)
    
    # コマンドパケットを送信　
    ser.write(packet)
    time.sleep(0.05)

    #print(' '.join(f'{byte:02X}' for byte in packet))

def dynamixel_current(id, current):
    """
    ダイナミクセルを回転させるためのパケット送信用の関数
    （トルクをONする必要あり）
    Parameters:
    id: ダイナミクセルのID (int(1～N))
    angle: float (0～359.91)  
    header: command[0]～[3] # ヘッダ(固定値)
    id_byte: ダイナミクセルのID
    Length: command以降のすべてのバイト数を指定
    command: Instruction write[0x03]
    Current_list:1つめの要素:アドレス[0x64] GoalPwm
                  3～4つめの要素: -885～885の16進数が入る
    crc_bytes：headerからPosition_listまでのチェックサム値(CRC-16-IBM)。
    Returns:なし（コマンドパケットの送信）
    """  
    # コマンドパケットを作成
    header = bytearray([0xFF, 0xFF, 0xFD, 0x00])
    id_byte = bytearray([id])
    Length=bytearray([0x07, 0x00])
    command=bytearray([0x03])
    if current >=500:
        current = 500
    elif current <= -500:
        current = -500
        
    if(current >=0):
        current_list = bytearray([0x66, 0x00, int(current), 0x00])
        
    if(current <0):
        current_list = bytearray([0x66, 0x00, int(-1.0*current), 0xFF])
        
    packet = header + id_byte + Length  + command  + current_list
    
    # 設定でCRC計算
    settings = {
        "poly": 0x8005, 
        "init_crc": 0x0000, 
        "reflect_data": False, 
        "reflect_crc": False
    }
    
    #CRCの計算
    crc = crc16(packet, **settings)
    lower_byte = crc & 0xFF
    upper_byte = (crc >> 8) & 0xFF
    crc_bytes = bytearray([lower_byte, upper_byte])
    packet.extend(crc_bytes)
    
    # コマンドパケットを送信　
    ser.write(packet)
    #print(' '.join(f'{byte:02X}' for byte in packet))

def inverse_kinematics_2link(L1, L2, x, y):
    def to_deg_360(rad):
        deg = math.degrees(rad) % 360
        if abs(deg - 360) < 1e-6:  # 誤差を吸収して0にする
            deg = 0.0
        return deg

    r2 = x**2 + y**2
    r = math.sqrt(r2)

    cos_theta2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))

    theta2 = math.pi - math.acos(cos_theta2)  # 肘下げ構成
    # theta2 = -math.acos(cos_theta2)         # 肘上げならこっち

    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)

    return to_deg_360(theta1)+90, to_deg_360(theta2)
        

def dynamixel_position_read(id):
    
    # コマンドパケットを作成
    header = bytearray([0xFF, 0xFF, 0xFD, 0x00])
    id_byte = bytearray([id])
    Length=bytearray([0x07, 0x00])
    command=bytearray([0x02])
    Position_list =bytearray([0x84, 0x00, 0x04, 0x00])

    packet = header + id_byte + Length  + command  + Position_list
    
    # 設定でCRC計算
    settings = {
        "poly": 0x8005, 
        "init_crc": 0x0000, 
        "reflect_data": False, 
        "reflect_crc": False
    }
    
    #チェックサムの計算
    crc = crc16(packet, **settings)
    lower_byte = crc & 0xFF
    upper_byte = (crc >> 8) & 0xFF
    crc_bytes = bytearray([lower_byte, upper_byte])
    packet.extend(crc_bytes)
    
    # コマンドパケットを送信　
    ser.reset_input_buffer()
    ser.write(packet)
    ser.reset_input_buffer()
    time.sleep(0.05)
    #print(' '.join(f'{byte:02X}' for byte in packet))

    # 受信処理
    time.sleep(0.05)  # モーター応答待ち
    if ser.in_waiting:
        response = ser.read(ser.in_waiting)
        #print('Received:', ' '.join(f'{b:02X}' for b in response))
        position, angle = parse_dynamixel_angle(response)
        return position, angle
    else:
        print("No response received.")
        return None

def dynamixel_velocity_read(id):
    # コマンドパケットを作成
    header = bytearray([0xFF, 0xFF, 0xFD, 0x00])
    id_byte = bytearray([id])
    Length=bytearray([0x07, 0x00])
    command=bytearray([0x02])
    Position_list =bytearray([0x80, 0x00, 0x04, 0x00])

    packet = header + id_byte + Length  + command  + Position_list
    
    # 設定でCRC計算
    settings = {
        "poly": 0x8005, 
        "init_crc": 0x0000, 
        "reflect_data": False, 
        "reflect_crc": False
    }
    
    #チェックサムの計算
    crc = crc16(packet, **settings)
    lower_byte = crc & 0xFF
    upper_byte = (crc >> 8) & 0xFF
    crc_bytes = bytearray([lower_byte, upper_byte])
    packet.extend(crc_bytes)
    
    # コマンドパケットを送信　
    ser.reset_input_buffer()
    ser.write(packet)
    ser.reset_input_buffer()

    # 受信処理
    time.sleep(0.01)  # モーター応答待ち
    if ser.in_waiting:
        response = ser.read(ser.in_waiting)
        #print(' '.join(f'{byte:02X}' for byte in response))
        count_velocity, velocity = parse_dynamixel_velocity(response)
        #print('Received:', ' '.join(f'{b:02X}' for b in response))
        return count_velocity, velocity
    else:
        print("No response received.")
        return None