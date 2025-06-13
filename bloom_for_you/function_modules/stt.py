import openai
import sounddevice as sd
import scipy.io.wavfile as wav
import tempfile

class STT:
    def __init__(self, openai_api_key):
        self.openai_api_key = openai_api_key
        self.duration = 5  # 녹음 시간 (초)
        self.samplerate = 16000  # Whisper는 16kHz 사용

        # API 키 설정 (전역)
        openai.api_key = self.openai_api_key

    def speech2text(self):
        print("음성 녹음을 시작합니다. 5초 동안 말해주세요...")
        audio = sd.rec(
            int(self.duration * self.samplerate),
            samplerate=self.samplerate,
            channels=1,
            dtype='int16'
        )
        sd.wait()
        print("녹음 완료. Whisper에 전송 중...")

        # 임시 파일로 저장
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
            wav.write(temp_wav.name, self.samplerate, audio)

            # Whisper API 호출
            with open(temp_wav.name, "rb") as f:
                transcript = openai.audio.transcriptions.create(
                    model="whisper-1",
                    file=f
                )

        print("STT 결과:", transcript.text)
        return transcript.text

if __name__ == "__main__":
    stt = STT(openai_api_key="sk-...")
    result = stt.speech2text()
    print("최종 텍스트:", result)