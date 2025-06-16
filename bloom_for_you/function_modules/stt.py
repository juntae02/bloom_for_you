from langchain_community.chat_models import ChatOpenAI
import openai
import sounddevice as sd
import scipy.io.wavfile as wav
import numpy as np
import tempfile
import os


samplerate = 16000


def stt(openai_api_key=None, duration=5) -> str:
    '''
    마이크로부터 지정한 시간만큼 음성을 녹음한 후,
    OpenAI Whisper API를 이용해 텍스트로 변환합니다.

    in: 
        duration (int): 녹음 시간 (초). 기본값은 5초.
        openai_api_key (str or None): OpenAI API 키. 전달하지 않으면 환경변수 'OPENAI_API_KEY'를 사용합니다.
    out: 
        음성 인식 결과 텍스트
    '''
    if openai_api_key is None:
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if openai_api_key is None:
            raise ValueError("API 키를 인자로 전달하거나 환경변수에 설정하세요.")

    # 녹음 설정
    print(f"음성 녹음을 시작합니다. \n {duration}초 동안 말해주세요...")
    audio = sd.rec(
        int(duration * samplerate),
        samplerate=samplerate,
        channels=1,
        dtype="int16",
    )
    sd.wait()
    print("녹음 완료. Whisper에 전송 중...")

    # 임시 WAV 파일 저장
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
        wav.write(temp_wav.name, samplerate, audio)

        # Whisper API 호출
        with open(temp_wav.name, "rb") as f:
            transcript = openai.Audio.transcribe(
                model="whisper-1", file=f, api_key=openai_api_key
            )

    print("STT 결과: ", transcript["text"])
    return transcript["text"]



# class STT:
#     def __init__(self, openai_api_key):
#         self.openai_api_key = openai_api_key
#         self.duration = 5  # seconds
#         self.samplerate = 16000  # Whisper는 16kHz를 선호

    # def speech2text(self):
    #     # 녹음 설정
    #     print("음성 녹음을 시작합니다. \n 5초 동안 말해주세요...")
    #     audio = sd.rec(
    #         int(self.duration * self.samplerate),
    #         samplerate=self.samplerate,
    #         channels=1,
    #         dtype="int16",
    #     )
    #     sd.wait()
    #     print("녹음 완료. Whisper에 전송 중...")

    #     # 임시 WAV 파일 저장
    #     with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
    #         wav.write(temp_wav.name, self.samplerate, audio)

    #         # Whisper API 호출
    #         with open(temp_wav.name, "rb") as f:
    #             transcript = openai.Audio.transcribe(
    #                 model="whisper-1", file=f, api_key=self.openai_api_key
    #             )

    #     print("STT 결과: ", transcript["text"])
    #     return transcript["text"]


# if __name__ == "__main__":
#     stt = STT(openai_api_key)
#     output_message = stt.speech2text()