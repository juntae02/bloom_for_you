from flask import Flask, request, redirect, url_for, send_from_directory
import numpy as np
import os
from datetime import datetime

# ───────── 경로 설정 ─────────
BASE          = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT      = os.path.abspath(os.path.join(BASE, "..", ".."))
# personal_folder 경로
PERSONAL_ROOT = os.path.abspath(os.path.join(PKG_ROOT, "personal_folder"))
STATIC_ROOT   = os.path.join(PKG_ROOT, "static")

# personal_folder 디렉토리 미리 생성
os.makedirs(PERSONAL_ROOT, exist_ok=True)

app = Flask(
    __name__,
    static_folder=STATIC_ROOT,
    static_url_path="/static"
)

# ───────── POST /signal ─────────
@app.route("/signal", methods=["POST"])
def signal():
    # form-data: audio, res_num, message
    audio   = request.files.get("audio")
    res_num = request.form.get("res_num")
    message = request.form.get("message")
    print(f"[DEBUG] 1. audio: {audio}, res_num: {res_num}, message: {message}")

    # fallback: JSON + message.wav
    use_fallback = False
    if not (audio and res_num and message):
        data = request.get_json(silent=True)
        print(f"[DEBUG] 2. fallback data: {data}")
        if not data:
            print("[DEBUG] 3. no data in fallback - bad request")
            return "bad request", 400
        res_num = data.get("res_num")
        message = data.get("message")
        if not (res_num and message):
            print("[DEBUG] 4. fallback data missing fields - bad request")
            return "bad request", 400
        use_fallback = True

    print(f"[DEBUG] 5. use_fallback: {use_fallback}")

    # timestamp
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{ts}.wav"

    # personal_folder/<res_num> 생성
    print(f"[DEBUG] 6. PERSONAL_ROOT: {PERSONAL_ROOT}")
    personal_dir = os.path.join(PERSONAL_ROOT, res_num)
    print(f"[DEBUG] 7. personal_dir: {personal_dir}")
    os.makedirs(personal_dir, exist_ok=True)

    # 오디오 저장: personal_folder에만
    if not use_fallback:
        save_path = os.path.join(personal_dir, filename)
        print(f"[DEBUG] 8. save_path: {save_path}")
        try:
            audio.save(save_path)
            print(f"[DEBUG] 9. audio saved successfully")
        except Exception as e:
            print(f"[ERROR] 10. audio 파일 저장 실패: {e}")
    else:
        src_wav = os.path.join(PKG_ROOT, "message.wav")
        dst = os.path.join(personal_dir, filename)
        print(f"[DEBUG] 11. src_wav: {src_wav}, dst: {dst}")
        try:
            os.replace(src_wav, dst)
            print(f"[DEBUG] 12. message.wav moved")
        except FileNotFoundError:
            print(f"[signal_server] warning: {src_wav} not found, skipping audio save")
        except Exception as e:
            print(f"[ERROR] 13. message.wav 이동 실패: {e}")

    # 메시지 기록 (.npy)
    npy_path = os.path.join(PKG_ROOT, f"messages_{res_num}.npy")
    print(f"[DEBUG] 14. npy_path: {npy_path}")
    if os.path.exists(npy_path):
        entries = list(np.load(npy_path, allow_pickle=True))
        print(f"[DEBUG] 15. loaded entries: {entries}")
    else:
        entries = []
        print(f"[DEBUG] 16. new entries list")
    entries.append((ts, message))
    np.save(npy_path, np.array(entries, dtype=object))
    print(f"[DEBUG] 17. saved npy: {npy_path}")

    return "", 204

# ───────── personal_audio 라우트 ─────────
@app.route("/personal_audio/<res_num>/<filename>")
def personal_audio(res_num, filename):
    """ personal_folder/<res_num> 에 저장된 오디오 제공 """
    directory = os.path.join(PERSONAL_ROOT, res_num)
    return send_from_directory(directory, filename)

# ───────── GET / (인덱스 페이지) ─────────
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
    body, html {
      width: 100%; height: 100%;
      font-family: 'Segoe UI', sans-serif;
      background: #f5f7fa;
    }
    .container {
      display: flex;
      width: 100%; max-width: 1200px; height: 100vh;
      margin: 0 auto; background: #fff;
      border-radius: 8px; overflow: hidden;
      box-shadow: 0 4px 12px rgba(0,0,0,0.1);
    }
    .sidebar {
      flex: 1; padding: 60px 40px;
      display: flex; flex-direction: column;
      justify-content: center;
    }
    .sidebar h1.brand { font-size: 2.5rem; margin-bottom: 40px; color: #3b5998; }
    .sidebar h2 { font-size: 1.6rem; margin-bottom: 20px; color: #222; }
    .sidebar form { display: flex; flex-direction: column; gap: 15px; }
    .sidebar input {
      padding: 12px 14px; font-size: 1rem;
      border: 1px solid #ccc; border-radius: 4px;
    }
    .sidebar button {
      padding: 12px 14px; font-size: 1rem; color: #fff;
      background: #3b5998; border: none; border-radius: 4px;
      cursor: pointer;
    }
    .sidebar button:hover { background: #354d82; }
    .hero {
      flex: 1.2; background: #e5ebf2;
      display: flex; align-items: center; justify-content: center;
      position: relative;
    }
    .hero .hero-header {
      position: absolute; top: 20px; right: 20px;
      font-size: 1.5rem; font-weight: bold; color: #333;
      background: rgba(255,255,255,0.8);
      padding: 8px 12px; border-radius: 4px;
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

# ───────── GET /messages (메시지 조회) ─────────
@app.route("/messages", methods=["GET"])
def messages():
    res_num = request.args.get("res_num", "").strip()
    if not res_num:
        return redirect(url_for("index"))

    npy_path = os.path.join(PKG_ROOT, f"messages_{res_num}.npy")
    entries = list(np.load(npy_path, allow_pickle=True)) if os.path.exists(npy_path) else []

    html_lines = [
        "<!DOCTYPE html>",
        "<html lang='ko'>",
        "<head>",
        "  <meta charset='utf-8'>",
        f"  <title>예약번호 {res_num}의 메시지</title>",
        "  <style>",
        "    html, body {",
        "      margin:0; padding:0;",
        "      font-family:sans-serif;",
        "      background: url('/static/images/flower.jpg') no-repeat center center fixed;",
        "      background-size: cover;",
        "    }",
        "    .top-right { position:absolute; top:20px; right:30px; }",
        "    .center-content {",
        "      display:flex; flex-direction:column; align-items:center;",
        "      justify-content:center; min-height:100vh;",
        "      text-align:center; padding:60px 20px;",
        "      background:rgba(255,255,255,0.85);",
        "      border-radius:8px; margin:30px;",
        "    }",
        "    h1 { margin-bottom:30px; font-size:2rem; }",
        "    ul { list-style:none; padding:0; }",
        "    li { margin-bottom:20px; }",
        "    audio { margin-top:5px; }",
        "    a { color:#3b5998; text-decoration:none; font-weight:bold; }",
        "    a:hover { text-decoration:underline; }",
        "  </style>",
        "</head>",
        "<body>",
        "  <div class='top-right'><a href='/'>다른 예약번호 조회</a></div>",
        "  <div class='center-content'>",
        f"    <h1>예약번호 {res_num}의 메시지</h1>",
        "    <ul>"
    ]
    for ts, msg in entries:
        audio_url = url_for('personal_audio', res_num=res_num, filename=f"{ts}.wav")
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

    # (옵션) HTML 파일로 저장
    html_file = os.path.join(PKG_ROOT, f"messages_{res_num}.html")
    with open(html_file, "w", encoding="utf-8") as f:
        f.write(html_str)
    print(f"[signal_server] Saved HTML → {html_file}")

    return html_str

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
