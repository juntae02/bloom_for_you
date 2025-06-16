from datetime import datetime

def log_voice_msg(msg: str):
    """콘솔에 타임스탬프와 함께 출력"""
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{ts}] {msg}")
