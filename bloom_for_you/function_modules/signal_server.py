# path: ros2_ws/src/bloom_for_you/bloom_for_you/function_modules/signal_server.py

from flask import Flask, request, send_from_directory, redirect, url_for
import numpy as np
import os
from datetime import datetime

# 프로젝트 루트와 static 폴더 경로 설정
BASE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.abspath(os.path.join(BASE, "..", ".."))
STATIC_ROOT = os.path.join(PKG_ROOT, "static")
AUDIO_ROOT = os.path.join(STATIC_ROOT, "audio")

app = Flask(
    __name__,
    static_folder=STATIC_ROOT,
    static_url_path="/static"
)

# POST 받는 엔드포인트: 음성(.wav) + 메시지(JSON) 처리
@app.route("/signal", methods=["POST"])
def signal():
    data = request.get_json(silent=True)
    res_num = data.get("res_num")
    message = data.get("message")
    if not (res_num and message):
        return "bad request", 400

    # 1) 타임스탬프 생성
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    # 2) static/audio/<res_num>/ 폴더 준비
    audio_dir = os.path.join(AUDIO_ROOT, res_num)
    os.makedirs(audio_dir, exist_ok=True)
    # 3) 음성 파일 원본(message.wav)을 복사해 두기
    src_wav = os.path.join(PKG_ROOT, "message.wav")
    dst_wav = os.path.join(audio_dir, f"{ts}.wav")
    try:
        os.replace(src_wav, dst_wav)
    except FileNotFoundError:
        print(f"[signal_server] warning: {src_wav} not found, skipping audio save")

    # 4) messages_<res_num>.npy 로드·업데이트
    npy_path = os.path.join(PKG_ROOT, f"messages_{res_num}.npy")
    if os.path.exists(npy_path):
        entries = list(np.load(npy_path, allow_pickle=True))
    else:
        entries = []
    entries.append((ts, message))
    np.save(npy_path, np.array(entries, dtype=object))

    return "", 204  # 성공

# GET / → 예약번호 입력 폼
@app.route("/", methods=["GET"])
def index():
    return """
<!DOCTYPE html>
<html lang="ko">
<head><meta charset="utf-8"><title>조회</title></head>
<body>
  <h1>예약번호별 메시지 조회</h1>
  <form action="/messages" method="get">
    <label>예약번호: <input name="res_num" required></label>
    <button type="submit">조회</button>
  </form>
</body>
</html>
"""

# GET /messages?res_num=XXXX → 해당 예약번호 메시지+오디오 렌더
@app.route("/messages", methods=["GET"])
def messages():
    res_num = request.args.get("res_num", "").strip()
    if not res_num:
        return redirect(url_for("index"))

    # 1) npy 읽기
    npy_path = os.path.join(PKG_ROOT, f"messages_{res_num}.npy")
    entries = []
    if os.path.exists(npy_path):
        entries = list(np.load(npy_path, allow_pickle=True))

    # 2) HTML 빌드
    html_lines = [
        "<!DOCTYPE html>",
        "<html lang='ko'>",
        "<head>",
        "  <meta charset='utf-8'>",
        f"  <title>Messages for {res_num}</title>",
        "</head>",
        "<body>",
        f"  <h1>예약번호 {res_num}의 메시지</h1>",
        "  <ul>"
    ]
    for ts, msg in entries:
        audio_url = f"/static/audio/{res_num}/{ts}.wav"
        html_lines.append(
            f"    <li>[{ts}] {msg}<br>"
            f"        <audio controls src='{audio_url}'></audio></li>"
        )
    html_lines += [
        "  </ul>",
        "  <a href='/'>다른 예약번호 조회</a>",
        "</body>",
        "</html>"
    ]
    html_str = "\n".join(html_lines)

    # ───────── 파일로 저장 ─────────
    html_file = os.path.join(PKG_ROOT, f"messages_{res_num}.html")
    with open(html_file, "w", encoding="utf-8") as f:
        f.write(html_str)
    print(f"[signal_server] Saved HTML → {html_file}")
    # ───────────────────────────────

    return html_str

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
