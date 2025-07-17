from langchain_community.chat_models import ChatOpenAI
import openai
import sounddevice as sd
import scipy.io.wavfile as wav
import numpy as np
import tempfile
import shutil            # 🆕 임시 파일 복사에 사용
import os

samplerate = 16000

# 패키지 루트 경로 계산 (여기에 음성 파일을 저장합니다)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.abspath(os.path.join(BASE_DIR, "..", ".."))
openai.api_key = os.getenv("OPENAI_API_KEY")
# ───────────────────────────────

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

def stt_with_save(openai_api_key=None, duration=5) -> str:
    if openai_api_key is None:
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if openai_api_key is None:
            raise ValueError("API 키를 인자로 전달하거나 환경변수에 설정하세요.")

    print(f"음성 녹음을 시작합니다. \n {duration}초 동안 말해주세요...")
    audio = sd.rec(
        int(duration * samplerate),
        samplerate=samplerate,
        channels=1,
        dtype="int16",
    )
    sd.wait()
    print("녹음 완료. Whisper에 전송 중...")

    # 프로젝트 루트에 message.wav로 저장
    dst_wav = os.path.join(PKG_ROOT, "message.wav")
    wav.write(dst_wav, samplerate, audio)
    print(f"[stt_with_save] saved WAV → {dst_wav}")

    # Whisper API 호출
    with open(dst_wav, "rb") as f:
        transcript = openai.Audio.transcribe(
            model="whisper-1", file=f, api_key=openai_api_key
        )

    print("STT 결과: ", transcript["text"])
    return transcript["text"]
