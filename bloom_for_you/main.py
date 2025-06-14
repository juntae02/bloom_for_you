import numpy as np
import requests

from speech_utils import tts, record_audio, transcribe_audio
from auth import authenticate
from robot_control import fetch_flower, place_flower_on_table
from logger import log
import config

def main():
    # 1) 인증 단계
    if not authenticate():
        tts("서비스를 종료합니다")
        return

    # 2) 꽃 가져오기 & 테이블에 놓기
    fetch_flower()
    place_flower_on_table()

    # 3) 문장 남기기
    tts("문장을 남겨주세요")
    log("문장을 저장중..")
    log("마이크를 정면으로 바라보고 5초간 말씀해주세요")
    record_audio(5, "message.wav")
    message = transcribe_audio("message.wav")
    log(f"인식된 문장: {message}")

    # 4) NumPy로 저장
    arr = np.array([message], dtype=object)
    np.save("message.npy", arr)
    log("message.npy로 저장 완료")

    # 5) 로컬에 신호 전송
    try:
        resp = requests.post(config.LOCAL_SIGNAL_URL, json={"status": "saved"})
        resp.raise_for_status()
        log("로컬 신호 전송 완료")
    except Exception as e:
        log(f"로컬 신호 전송 실패: {e}")

    tts("메시지가 저장되었습니다")

if __name__ == "__main__":
    main()
