# import pyttsx3

# engine = pyttsx3.init()             # 엔진 초기화
# engine.setProperty('rate',150)      # 말하는 속도 조절 (기본값 200)
# engine.setProperty('volume', 1.0)   # 말하는 속도 조절 (기본값 200)

# text = "안녕하세요. text to speech 예제입니다."

# engine.say(text)                    # 읽어줄 문장 설정
# engine.runAndWait()

import tempfile
from gtts import gTTS
import os
import subprocess
# from playsound import playsound
def make_txt(template:str, input_list:list) -> str:
    '''
    템플릿 문자열에 입력값들을 채워 문장을 생성

    in: 
        template (str): 포맷팅 문자열 (예: "안녕하세요, {}님. 오늘은 {}을(를) 하실 건가요?")
        input_list (list): 템플릿에 채워 넣을 값들의 리스트 (예: ["지민", "공부"])
    out: 
        입력값이 채워진 완성된 문장
    '''
    return template.format(*input_list)

def tts(text:str):
    text = text
    tts = gTTS(text=text, lang='ko')

    with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as temp_mp3:  # tempfile.NamedTemporaryFile()을 호출 시, 임시 파일 객체 반환 
        tts.write_to_fp(temp_mp3)  # 임시 파일에 저장(파일 객체(temp_mp3)는 일반 파일 객체처럼 읽기/쓰기 가능)
        temp_mp3_path = temp_mp3.name   # .name 속성: 임시 파일이 디스크 상 실제 저장된 위치가 문자열로 저장됨

    try:
        # os.system(f'mpg123 "{temp_mp3_path}"')
        subprocess.run(
            ['mpg123', temp_mp3_path],
            stdout=subprocess.DEVNULL,      # 표준 출력 버림
            stderr=subprocess.DEVNULL       # 표준 에러 버림
        )
    finally:
        os.remove(temp_mp3_path)  # 재생 후 임시 파일 삭제
#추가

# 필요하다면 tts.py 안에서 바로 이렇게 호출도 가능합니다:
if __name__ == "__main__":
    # TTS 테스트
    tts("안녕하세요, STT 테스트를 시작하겠습니다.")
