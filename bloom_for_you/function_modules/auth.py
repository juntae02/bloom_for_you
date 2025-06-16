# path: bloom_for_you/bloom_for_you/function_modules/auth.py

import re
from bloom_for_you.function_modules.tts import tts
from bloom_for_you.function_modules.stt import stt
from bloom_for_you.function_modules.logger import log
import bloom_for_you.function_modules.config as config

def authenticate() -> str | None:
    """
    1) 최대 2회까지 예약번호 인증 (stt() 로 통합)
    2) 성공하면 예약번호(res_num)를 리턴, 아니면 None
    """
    attempts = 0
    while attempts < 2:
        tts("예약번호를 말씀해주세요")
        raw = stt(duration=5)
        log(f"인증 시도 - 원문 인식: {raw}")

        res_num = "".join(re.findall(r"\d", raw))
        log(f"정제된 예약번호: {res_num}")

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
