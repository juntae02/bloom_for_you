from flask import Flask, request, redirect, url_for
import numpy as np
import os
from datetime import datetime

BASE        = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT    = os.path.abspath(os.path.join(BASE, "..", ".."))
STATIC_ROOT = os.path.join(PKG_ROOT, "static")
AUDIO_ROOT  = os.path.join(STATIC_ROOT, "audio")

app = Flask(
    __name__,
    static_folder=STATIC_ROOT,
    static_url_path="/static"
)

@app.route("/signal", methods=["POST"])
def signal():
    data    = request.get_json(silent=True)
    res_num = data.get("res_num")
    message = data.get("message")
    if not (res_num and message):
        return "bad request", 400

    # 타임스탬프 생성
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    # audio 디렉토리 준비
    audio_dir = os.path.join(AUDIO_ROOT, res_num)
    os.makedirs(audio_dir, exist_ok=True)

    # 음성 파일 저장
    src_wav = os.path.join(PKG_ROOT, "message.wav")
    dst_wav = os.path.join(audio_dir, f"{ts}.wav")
    try:
        os.replace(src_wav, dst_wav)
    except FileNotFoundError:
        print(f"[signal_server] warning: {src_wav} not found, skipping audio save")

    # 메시지 기록 저장
    npy_path = os.path.join(PKG_ROOT, f"messages_{res_num}.npy")
    if os.path.exists(npy_path):
        entries = list(np.load(npy_path, allow_pickle=True))
    else:
        entries = []
    entries.append((ts, message))
    np.save(npy_path, np.array(entries, dtype=object))

    return "", 204

@app.route("/", methods=["GET"])
def index():
    return """
<!DOCTYPE html>
<html lang="ko">
<head>
  <meta charset="utf-8" />
  <title>BLOOM FOR YOU - 예약번호별 메시지 조회</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body, html { width: 100%; height: 100%; font-family: 'Segoe UI', sans-serif; background: #f5f7fa; }
    .container {
      display: flex;
      width: 100%; max-width: 1200px; height: 100vh;
      margin: 0 auto; background: #fff; border-radius: 8px; overflow: hidden;
      box-shadow: 0 4px 12px rgba(0,0,0,0.1);
    }
    .sidebar {
      flex: 1; padding: 60px 40px; display: flex; flex-direction: column; justify-content: center;
    }
    .sidebar h1.brand { font-size: 2.5rem; margin-bottom: 40px; color: #3b5998; }
    .sidebar h2 { font-size: 1.6rem; margin-bottom: 20px; color: #222; }
    .sidebar form { display: flex; flex-direction: column; gap: 15px; }
    .sidebar input {
      padding: 12px 14px; font-size: 1rem; border: 1px solid #ccc;
      border-radius: 4px;
    }
    .sidebar button {
      padding: 12px 14px; font-size: 1rem; color: #fff;
      background: #3b5998; border: none; border-radius: 4px;
      cursor: pointer;
    }
    .sidebar button:hover { background: #354d82; }

    .hero {
      flex: 1.2;
      background: #e5ebf2;
      display: flex; align-items: center; justify-content: center;
      position: relative;
    }
    .hero .hero-header {
      position: absolute;
      top: 20px;
      right: 20px;
      font-size: 1.5rem;
      font-weight: bold;
      color: #333;
      background: rgba(255,255,255,0.8);
      padding: 8px 12px;
      border-radius: 4px;
    }
    .hero img {
      max-width: 100%; height: auto; object-fit: contain;
    }
  </style>
</head>
<body>
  <div class="container">
    <div class="sidebar">
      <h1 class="brand">BLOOM FOR YOU</h1>
      <h2>로그인</h2>
      <form action="/messages" method="get">
        <input name="res_num" type="text" placeholder="예약번호" required />
        <button type="submit">조회</button>
      </form>
    </div>
    <div class="hero">
      <div class="hero-header">CARPE-DIEM</div>
      <img src="/static/images/congrats.png" alt="축하하며 꽃을 건네주는 그림" />
    </div>
  </div>
</body>
</html>
"""

@app.route("/messages", methods=["GET"])
def messages():
    res_num = request.args.get("res_num", "").strip()
    if not res_num:
        return redirect(url_for("index"))

    # npy에서 기록 로드
    npy_path = os.path.join(PKG_ROOT, f"messages_{res_num}.npy")
    entries = []
    if os.path.exists(npy_path):
        entries = list(np.load(npy_path, allow_pickle=True))

    # HTML 생성
    html_lines = [
        "<!DOCTYPE html>",
        "<html lang='ko'>",
        "<head>",
        "  <meta charset='utf-8'>",
        f"  <title>예약번호 {res_num}님의 메시지</title>",
        "  <style>",
        "    body {",
        "      margin: 0;",
        "      font-family: sans-serif;",
        "      /* 꽃 배경 */",
        "      background: url('/static/images/flower.jpg') no-repeat center center fixed;",
        "      background-size: cover;",
        "    }",
        "    .top-right {",
        "      position: absolute;",
        "      top: 20px;",
        "      right: 30px;",
        "      z-index: 2;",
        "    }",
        "    .center-content {",
        "      display: flex;",
        "      flex-direction: column;",
        "      align-items: center;",
        "      justify-content: center;",
        "      min-height: 100vh;",
        "      text-align: center;",
        "      /* 메시지 가독성 위해 반투명 박스 */",
        "      background-color: rgba(255, 255, 255, 0.8);",
        "      border-radius: 12px;",
        "      margin: 30px;",
        "      padding: 20px;",
        "    }",
        "    h1 { font-size: 2rem; margin-bottom: 30px; }",
        "    ul { list-style: none; padding: 0; }",
        "    li { margin-bottom: 20px; }",
        "    audio { margin-top: 5px; }",
        "    a { color: #3b5998; text-decoration: none; font-weight: bold; }",
        "    a:hover { text-decoration: underline; }",
        "  </style>",
        "</head>",
        "<body>",
        "  <div class='top-right'><a href='/'>다른 예약번호 조회</a></div>",
        "  <div class='center-content'>",
        f"    <h1>예약번호 {res_num}님의 메시지</h1>",
        "    <ul>"
    ]

    for ts, msg in entries:
        audio_url = f"/static/audio/{res_num}/{ts}.wav"
        html_lines.append(
            f"      <li>[{ts}] {msg}<br><audio controls src='{audio_url}'></audio></li>"
        )

    html_lines += [
        "    </ul>",
        "  </div>",
        "</body>",
        "</html>"
    ]

    html_str = "\n".join(html_lines)

    # 파일로도 저장
    html_file = os.path.join(PKG_ROOT, f"messages_{res_num}.html")
    with open(html_file, "w", encoding="utf-8") as f:
        f.write(html_str)
    print(f"[signal_server] Saved HTML → {html_file}")

    return html_str

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
