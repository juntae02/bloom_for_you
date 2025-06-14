import sounddevice as sd
import wavio
import openai
import config
from gtts import gTTS
import playsound
import tempfile
import os

# OpenAI 키 세팅
openai.api_key = config.OPENAI_API_KEY

def tts(text: str):
    """음성 출력 (Google TTS, 한국어 여성 음성)"""
    # 임시 mp3 파일 생성
    with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as fp:
        tmp_path = fp.name
    # TTS 변환 (lang="ko" → 한국어, 기본 여성 음성)
    tts_obj = gTTS(text=text, lang="ko")
    tts_obj.save(tmp_path)
    # 재생
    playsound.playsound(tmp_path, True)
    # 임시 파일 삭제
    os.remove(tmp_path)

def record_audio(duration: float, filename: str, fs: int = 16000) -> str:
    """
    duration 초 동안 마이크 녹음 후 WAV 파일로 저장
    """
    print(f"[record_audio] {duration}s 녹음 시작 → {filename}")
    data = sd.rec(int(duration * fs), samplerate=fs, channels=1)
    sd.wait()
    wavio.write(filename, data, fs, sampwidth=2)
    return filename

def transcribe_audio(filename: str) -> str:
    """
    Whisper API로 STT 수행하고 텍스트 리턴
    """
    print(f"[transcribe_audio] {filename} 전송 중…")
    with open(filename, "rb") as f:
        res = openai.Audio.transcribe("whisper-1", f)
    return res["text"].strip()
