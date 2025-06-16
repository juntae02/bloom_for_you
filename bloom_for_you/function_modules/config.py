import os

# 반드시 환경변수로 OPENAI_API_KEY를 세팅해주세요.
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

# 로컬 신호를 받을 서버 URL
LOCAL_SIGNAL_URL = "http://localhost:5000/signal"

VALID_RESERVATIONS = [
    "0001",
    "1234",
    "5678",
]