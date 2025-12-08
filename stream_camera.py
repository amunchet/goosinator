from flask import Flask, Response, render_template_string
from picamera2 import Picamera2
import cv2
import threading

app = Flask(__name__)
picam2 = Picamera2()

# Configure camera
config = picam2.create_video_configuration(main={"size": (1280, 720)})
picam2.configure(config)
picam2.start()

# Simple HTML page that shows the stream
HTML_PAGE = """
<!doctype html>
<html>
  <head>
    <title>Pi Camera Stream</title>
    <style>
      body { margin: 0; background: #111; display: flex; justify-content: center; align-items: center; height: 100vh;}
      img { max-width: 100%; height: auto; }
    </style>
  </head>
  <body>
    <img src="{{ url_for('video_feed') }}">
  </body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML_PAGE)

def generate_frames():
    while True:
        # Capture frame as numpy array
        frame = picam2.capture_array()
        # Convert from RGB to BGR for OpenCV
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # Encode as JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        jpg_bytes = buffer.tobytes()

        # Yield frame in multipart/x-mixed-replace format
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + jpg_bytes + b"\r\n"
        )

@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

if __name__ == "__main__":
    # 0.0.0.0 makes it visible on your LAN, change port if you want
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)

