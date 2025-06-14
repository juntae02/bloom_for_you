import re
from speech_utils import tts, record_audio, transcribe_audio
from logger import log
import config

def authenticate() -> bool:
    """
    1) 최대 2회까지 예약번호 인증
       – 음성 인식 결과에서 숫자만 추출해 비교
    2) 유효한 번호라면 확인 멘트 후 True 리턴
       아니면 재시도 또는 종료
    """
    attempts = 0
    while attempts < 2:
        # 예약번호 요청
        tts("예약번호를 말씀해주세요")
        record_audio(5, "res_num.wav")  # 녹음 시간 5초
        raw = transcribe_audio("res_num.wav")
        log(f"인증 시도 - 원문 인식: {raw}")

        # 숫자만 추출
        res_num = "".join(re.findall(r"\d", raw))
        log(f"정제된 예약번호: {res_num}")

        if res_num in config.VALID_RESERVATIONS:
            # 성공 멘트
            tts("예약번호 확인 완료했습니다")
            tts("화분을 가져올 동안 잠시 기다려주세요")
            return True
        else:
            attempts += 1
            if attempts < 2:
                tts("예약번호가 틀렸습니다. 다시 시도해주세요")
            else:
                tts("두 번 틀리셨습니다. 고객센터로 문의해주세요")
                return False
