import subprocess
import wave
import os
import json
import subprocess
import requests
import time
import serial
import math
import json
import re
import concurrent.futures
from robot_controller.enums import Path, Const
from dotenv import load_dotenv
from vosk import Model, KaldiRecognizer

load_dotenv()
ARECORD_CARD = os.getenv("ARECORD_CARD")
ARECORD_DEVICE = os.getenv("ARECORD_DEVICE")
OLLAMA_PATH = os.getenv("OLLAMA_PATH")
OLLAMA_MODEL = os.getenv("OLLAMA_MODEL") 

def record_audio(filename, duration=10):
    """
    指定された録音時間で声を録音する
    """
    # arecordで録音（16bit, 16kHz, モノラル）
    print(f"[INFO] {duration}秒録音中...")
    cmd = [
        "arecord",
        "-D", f"plughw:{ARECORD_CARD},{ARECORD_DEVICE}", 
        "-f", "S16_LE",
        "-r", "16000",
        "-c", "1",
        "-d", str(duration),
        filename
    ]
    subprocess.run(cmd, check=True)
    print("[INFO] 録音完了")

def generate_text_from_audiofile(filename, model_path):
    """
    voskで音声ファイルからテキストに変換
    """
    if not os.path.exists(model_path):
        print(f"[ERROR] モデルが {model_path} に存在しません")
        return

    wf = wave.open(filename, "rb")
    if wf.getnchannels() != 1 or wf.getsampwidth() != 2 or wf.getframerate() != 16000:
        print("[ERROR] 録音フォーマットが不正です（16bit, 16kHz, mono）")
        return

    model = Model(model_path)
    rec = KaldiRecognizer(model, wf.getframerate())

    print("[INFO] 音声をテキストに変換中...")
    final_result = ""

    while True:
        data = wf.readframes(4000)
        if len(data) == 0:
            break
        rec.AcceptWaveform(data)

    result_json = json.loads(rec.FinalResult())
    raw_text = result_json.get("text", "")
    final_result = raw_text.replace(" ", "")
    return final_result

def generate_text_with_ollama(input_text):
    """
    ollamaでテキストを生成
    """
    prompt = input_text
    roll = """
    （あなたは右手と左手がある人工知能ロボットです。返答は日本語のみで50文字以内で返してください。絵文字はなし。
    [talk]として会話であなたが返答する内容を、[action]として会話の内容であなたが行う行動を記載してJSON形式で返答してください。）
    """
    prompt = prompt + roll

    response = requests.post(
        OLLAMA_PATH,  # ←ホストのOllamaへ
        json={
            "model": OLLAMA_MODEL,
            "prompt": prompt,
            "stream": False
        }
    )

    error_dict = { "talk": "すみません、よく分かりませんでした", "action": "None" }

    match = re.search(r"```json\s*(\{.*\})\s*```?", response.json()["response"], re.DOTALL)
    if match:
        json_str = match.group(1)
        result_dict = json.loads(json_str)
        return result_dict
    else:
        print("JSON部分が見つかりません")
        return error_dict
    
    #return response.json()["response"]

def speak_text_with_openjtalk(text):
    """
    openjtalkで生成された返答を音声に変換、aplayで再生
    """
    # テキストをファイルに保存
    with open(Path.GENERATE_TEXT_DIR, "w", encoding="utf-8") as f:
        f.write(text)
    
    # OpenJTalkでwav生成
    cmd_oj = [
        "open_jtalk",
        "-x", Path.OPEN_JTOLK_DIR,
        "-m", Path.HTS_VOICE_DIR,
        "-ow", Path.OUTPUT_VOICE_DIR,
        Path.GENERATE_TEXT_DIR
    ]
    subprocess.run(cmd_oj, check=True)
    
    # aplayで再生
    subprocess.run(["paplay", Path.OUTPUT_VOICE_DIR], check=True)

def talk():

    wait_questions = "何か質問はありますか"

    speak_text_with_openjtalk(wait_questions)
    time.sleep(0.1)

    with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
        record_voice = executor.submit(record_audio, Path.INPUT_VOICE_DIR, duration=Const.RECORD_TIME)

        # --- Ollama の返答待ち中に別の処理を挟む ---
        while not record_voice.done():
            # ここに「返答を待つ間にしたいこと」を入れる
            print("録音処理中です")
            time.sleep(5.0)

    input_text = generate_text_from_audiofile(Path.INPUT_VOICE_DIR, Path.AUDIO_MODEL_DIR)

    if input_text:
        print(f"\n[INFO] 音声認識結果: {input_text}")

        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
            generate_text = executor.submit(generate_text_with_ollama, input_text)

            # --- Ollama の返答待ち中に別の処理を挟む ---
            while not generate_text.done():
                # ここに「返答を待つ間にしたいこと」を入れる
                speak_text_with_openjtalk("答える内容を考えています")
                time.sleep(0.5)

        print(f"\n[INFO] テキスト生成結果: {generate_text.result()}")
        speak_text_with_openjtalk(generate_text.result()["talk"])

    else:
        speak_text_with_openjtalk("質問はないようですね、さようなら")

