import numpy as np
import requests
import re
from bloom_for_you.function_modules.tts import tts
from bloom_for_you.function_modules.stt import stt_with_save
import bloom_for_you.function_modules.config as config
from datetime import datetime

def log_voice_msg(msg: str):
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{ts}] {msg}")

def authenticate() -> str | None:
    attempts = 0
    while attempts < 2:
        tts("예약번호를 말씀해주세요")
        raw = stt_with_save(duration=5)
        log_voice_msg(f"인증 시도 - 원문 인식: {raw}")

        res_num = "".join(re.findall(r"\d", raw))
        log_voice_msg(f"정제된 예약번호: {res_num}")

        if res_num in config.VALID_RESERVATIONS:
            tts("예약번호 확인 완료했습니다")
            return res_num
        else:
            attempts += 1
            if attempts < 2:
                tts("예약번호가 틀렸습니다. 다시 시도해주세요")
            else:
                tts("두 번 틀리셨습니다. 고객센터로 문의해주세요")
                return None

def main():
    # 1) 인증 단계
    res_num = authenticate()
    if not res_num:
        tts("서비스를 종료합니다")
        return

    # 2) 꽃 가져오기 & 테이블에 놓기
    tts("화분을 가져오는 중입니다")
    # fetch_flower()
    # place_flower_on_table()

    # 3) 문장 남기기
    tts("문장을 남겨주세요")
    log_voice_msg("문장을 저장중..")
    log_voice_msg("마이크를 정면으로 바라보고 5초간 말씀해주세요")
    message = stt_with_save(duration=5)
    log_voice_msg(f"인식된 문장: {message}")

    # 4) 녹음 파일명 지정 (stt_with_save가 저장하는 파일명과 동일하게!)
    audio_path = "message.wav"  # ← stt_with_save의 파일명으로 변경 필요!
    # 만약 stt_with_save가 message.wav로 저장하면 "message.wav"로 변경!

    # 5) NumPy로 로컬 저장 (optional)
    np.save("message.npy", np.array([message], dtype=object))
    log_voice_msg("message.npy로 저장 완료")

    # 6) 서버에 예약번호·메시지·오디오 파일 form-data로 전송!
    try:
        with open(audio_path, 'rb') as audio_file:
            files = {'audio': audio_file}
            data = {'res_num': res_num, 'message': message}
            resp = requests.post(config.LOCAL_SIGNAL_URL, files=files, data=data)
            resp.raise_for_status()
            log_voice_msg("로컬 신호 전송 완료")
    except Exception as e:
        log_voice_msg(f"로컬 신호 전송 실패: {e}")

    tts("메시지가 저장되었습니다")

if __name__ == "__main__":
    main()
