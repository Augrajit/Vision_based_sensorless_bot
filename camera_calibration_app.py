"""
camera_calibration_app.py — Flask web app for camera homography calibration.
Runs on RPi 4, opened in any browser on the LAN.
Allows selecting a camera, viewing its live MJPEG stream, clicking 4 calibration
points, entering real-world coordinates, and computing/saving the homography matrix.
"""

import time
import json
import os
import cv2
import numpy as np
from flask import Flask, Response, request, jsonify

app = Flask(__name__)

CAMERAS_JSON_PATH = "cameras.json"

# Shared camera capture objects (lazy-initialized)
_camera_captures = {}


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}")


def get_camera_capture(camera_index):
    """Get or create a VideoCapture for the given camera index."""
    if camera_index not in _camera_captures:
        # Try to load RTSP URL from cameras.json if it exists
        rtsp_url = None
        if os.path.exists(CAMERAS_JSON_PATH):
            try:
                with open(CAMERAS_JSON_PATH, "r") as f:
                    cameras = json.load(f)
                for cam in cameras:
                    if cam.get("camera_index") == camera_index:
                        rtsp_url = cam.get("rtsp_url")
                        break
            except Exception:
                pass

        src = rtsp_url if rtsp_url else camera_index
        log(f"Opening camera {camera_index}: {src}")
        cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            log(f"WARNING: Could not open camera {camera_index}")
            return None
        _camera_captures[camera_index] = cap

    return _camera_captures[camera_index]


def generate_mjpeg(camera_index):
    """Generator that yields MJPEG frames from the given camera."""
    cap = get_camera_capture(camera_index)
    if cap is None:
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.1)
            continue
        _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')


HTML_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Camera Calibration Tool</title>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body {
    font-family: 'Segoe UI', Arial, sans-serif;
    background: #0f0f1a;
    color: #e0e0e0;
    min-height: 100vh;
  }
  .header {
    background: linear-gradient(135deg, #1a1a3e 0%, #2d1b69 100%);
    padding: 20px 30px;
    border-bottom: 2px solid #6c3baa;
    text-align: center;
  }
  .header h1 {
    font-size: 24px;
    color: #c9a0ff;
    letter-spacing: 1px;
  }
  .header p { color: #888; font-size: 13px; margin-top: 4px; }
  .container { max-width: 1100px; margin: 20px auto; padding: 0 20px; }
  .controls {
    display: flex;
    gap: 15px;
    align-items: center;
    margin-bottom: 15px;
    flex-wrap: wrap;
  }
  label { font-size: 14px; color: #aaa; }
  select, input[type="text"] {
    background: #1e1e3a;
    border: 1px solid #444;
    color: #e0e0e0;
    padding: 8px 12px;
    border-radius: 6px;
    font-size: 14px;
  }
  select:focus, input:focus { outline: none; border-color: #6c3baa; }
  .stream-container {
    position: relative;
    display: inline-block;
    border: 2px solid #333;
    border-radius: 8px;
    overflow: hidden;
    background: #000;
  }
  .stream-container img {
    display: block;
    max-width: 100%;
  }
  .stream-container canvas {
    position: absolute;
    top: 0;
    left: 0;
    cursor: crosshair;
  }
  .instructions {
    background: #1a1a2e;
    border: 1px solid #333;
    border-radius: 8px;
    padding: 15px;
    margin: 15px 0;
    font-size: 13px;
    line-height: 1.7;
  }
  .instructions ol { padding-left: 20px; }
  .instructions li { margin-bottom: 4px; }
  .instructions strong { color: #c9a0ff; }
  .points-grid {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 10px;
    margin: 15px 0;
  }
  .point-card {
    background: #1a1a2e;
    border: 1px solid #333;
    border-radius: 8px;
    padding: 12px;
    text-align: center;
  }
  .point-card h4 { color: #c9a0ff; margin-bottom: 8px; font-size: 13px; }
  .point-card .pixel { color: #6c6; font-size: 12px; margin-bottom: 6px; }
  .point-card input {
    width: 60px;
    text-align: center;
    margin: 2px;
  }
  .btn {
    padding: 10px 24px;
    border: none;
    border-radius: 6px;
    font-size: 14px;
    font-weight: 600;
    cursor: pointer;
    transition: all 0.2s;
  }
  .btn:disabled { opacity: 0.4; cursor: not-allowed; }
  .btn-primary {
    background: linear-gradient(135deg, #6c3baa 0%, #4a1d8e 100%);
    color: white;
  }
  .btn-primary:hover:not(:disabled) { transform: translateY(-1px); box-shadow: 0 4px 15px rgba(108,59,170,0.4); }
  .btn-reset { background: #333; color: #e0e0e0; }
  .btn-reset:hover { background: #444; }
  .actions { display: flex; gap: 10px; margin: 15px 0; }
  .rtsp-input { flex: 1; }
  .confirmation {
    background: #1a3a1a;
    border: 1px solid #2d6b2d;
    border-radius: 8px;
    padding: 15px;
    margin: 15px 0;
    color: #6c6;
    font-size: 16px;
    text-align: center;
    display: none;
  }
  .status { color: #888; font-size: 12px; margin-top: 10px; }
</style>
</head>
<body>

<div class="header">
  <h1>&#128247; Camera Calibration Tool</h1>
  <p>Click 4 points on the stream, enter real-world coordinates, and compute homography</p>
</div>

<div class="container">
  <div class="controls">
    <label for="camSelect">Camera:</label>
    <select id="camSelect" onchange="switchCamera()">
      <option value="0">Camera 0</option>
      <option value="1">Camera 1</option>
      <option value="2">Camera 2</option>
      <option value="3">Camera 3</option>
    </select>
    <label for="rtspUrl">RTSP URL:</label>
    <input type="text" id="rtspUrl" class="rtsp-input" placeholder="rtsp://192.168.1.X/stream" />
  </div>

  <div class="instructions">
    <ol>
      <li><strong>Select a camera</strong> from the dropdown and optionally enter its RTSP URL</li>
      <li><strong>Click 4 points</strong> on the video stream (numbered circles will appear)</li>
      <li><strong>Enter the real-world coordinates</strong> (in cm) for each clicked point</li>
      <li>Click <strong>"Compute &amp; Save"</strong> to calculate the homography and save to cameras.json</li>
    </ol>
  </div>

  <div class="stream-container" id="streamContainer">
    <img id="streamImg" src="/stream/0" alt="Camera stream" onload="resizeCanvas()" />
    <canvas id="overlay" onclick="handleCanvasClick(event)"></canvas>
  </div>

  <div class="points-grid" id="pointsGrid"></div>

  <div class="actions">
    <button class="btn btn-primary" id="computeBtn" disabled onclick="computeAndSave()">
      Compute &amp; Save
    </button>
    <button class="btn btn-reset" onclick="resetPoints()">Reset Points</button>
  </div>

  <div class="confirmation" id="confirmation"></div>
  <div class="status" id="statusText"></div>
</div>

<script>
  const points = [];       // [{px, py, wx, wy}, ...]
  const POINT_COLORS = ['#ff4444', '#44ff44', '#4488ff', '#ffaa00'];
  const POINT_LABELS = ['P1', 'P2', 'P3', 'P4'];

  function switchCamera() {
    const idx = document.getElementById('camSelect').value;
    document.getElementById('streamImg').src = '/stream/' + idx + '?' + Date.now();
    resetPoints();
  }

  function resizeCanvas() {
    const img = document.getElementById('streamImg');
    const canvas = document.getElementById('overlay');
    canvas.width = img.clientWidth;
    canvas.height = img.clientHeight;
    redrawOverlay();
  }
  window.addEventListener('resize', resizeCanvas);

  function handleCanvasClick(e) {
    if (points.length >= 4) return;
    const canvas = document.getElementById('overlay');
    const rect = canvas.getBoundingClientRect();
    const img = document.getElementById('streamImg');
    const scaleX = img.naturalWidth / img.clientWidth;
    const scaleY = img.naturalHeight / img.clientHeight;
    const px = Math.round((e.clientX - rect.left) * scaleX);
    const py = Math.round((e.clientY - rect.top) * scaleY);
    points.push({ px, py, wx: 0, wy: 0 });
    redrawOverlay();
    updatePointsGrid();
    if (points.length === 4) {
      document.getElementById('computeBtn').disabled = false;
    }
  }

  function redrawOverlay() {
    const canvas = document.getElementById('overlay');
    const ctx = canvas.getContext('2d');
    const img = document.getElementById('streamImg');
    const scaleX = img.clientWidth / (img.naturalWidth || img.clientWidth);
    const scaleY = img.clientHeight / (img.naturalHeight || img.clientHeight);
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Draw crosshair under cursor (handled by CSS cursor: crosshair)

    points.forEach((pt, i) => {
      const dx = pt.px * scaleX;
      const dy = pt.py * scaleY;

      // Circle
      ctx.beginPath();
      ctx.arc(dx, dy, 12, 0, Math.PI * 2);
      ctx.fillStyle = POINT_COLORS[i] + '66';
      ctx.fill();
      ctx.strokeStyle = POINT_COLORS[i];
      ctx.lineWidth = 2;
      ctx.stroke();

      // Label
      ctx.fillStyle = '#fff';
      ctx.font = 'bold 11px sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(POINT_LABELS[i], dx, dy);
    });
  }

  function updatePointsGrid() {
    const grid = document.getElementById('pointsGrid');
    grid.innerHTML = '';
    points.forEach((pt, i) => {
      const card = document.createElement('div');
      card.className = 'point-card';
      card.innerHTML = `
        <h4 style="color:${POINT_COLORS[i]}">${POINT_LABELS[i]}</h4>
        <div class="pixel">Pixel: (${pt.px}, ${pt.py})</div>
        <div>
          <label>X cm:</label>
          <input type="text" id="wx${i}" value="${pt.wx}" onchange="updateWorldCoord(${i})" />
        </div>
        <div style="margin-top:4px">
          <label>Y cm:</label>
          <input type="text" id="wy${i}" value="${pt.wy}" onchange="updateWorldCoord(${i})" />
        </div>
      `;
      grid.appendChild(card);
    });
  }

  function updateWorldCoord(i) {
    points[i].wx = parseFloat(document.getElementById('wx' + i).value) || 0;
    points[i].wy = parseFloat(document.getElementById('wy' + i).value) || 0;
  }

  function resetPoints() {
    points.length = 0;
    document.getElementById('computeBtn').disabled = true;
    document.getElementById('confirmation').style.display = 'none';
    updatePointsGrid();
    redrawOverlay();
  }

  async function computeAndSave() {
    // Collect world coords from inputs
    for (let i = 0; i < 4; i++) {
      points[i].wx = parseFloat(document.getElementById('wx' + i).value) || 0;
      points[i].wy = parseFloat(document.getElementById('wy' + i).value) || 0;
    }

    const camIdx = parseInt(document.getElementById('camSelect').value);
    const rtspUrl = document.getElementById('rtspUrl').value.trim();

    const payload = {
      camera_index: camIdx,
      rtsp_url: rtspUrl || `camera_${camIdx}`,
      pixel_points: points.map(p => [p.px, p.py]),
      world_points: points.map(p => [p.wx, p.wy])
    };

    document.getElementById('statusText').textContent = 'Computing homography...';

    try {
      const resp = await fetch('/calibrate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      const data = await resp.json();
      if (data.status === 'ok') {
        const conf = document.getElementById('confirmation');
        conf.textContent = '\\u2713 Camera ' + camIdx + ' calibrated';
        conf.style.display = 'block';
        document.getElementById('statusText').textContent =
          'Homography saved to cameras.json. Matrix: ' + JSON.stringify(data.matrix);
      } else {
        document.getElementById('statusText').textContent = 'Error: ' + (data.error || 'Unknown');
      }
    } catch (err) {
      document.getElementById('statusText').textContent = 'Request failed: ' + err;
    }
  }
</script>
</body>
</html>
"""


@app.route("/")
def index():
    """Serve the calibration HTML page."""
    return HTML_PAGE


@app.route("/stream/<int:camera_index>")
def stream(camera_index):
    """MJPEG stream from the given camera index."""
    return Response(
        generate_mjpeg(camera_index),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/calibrate", methods=["POST"])
def calibrate():
    """
    Compute homography from 4 pixel-to-world point correspondences.
    Expects JSON: {camera_index, rtsp_url, pixel_points, world_points}
    """
    try:
        data = request.get_json()
        camera_index = int(data["camera_index"])
        rtsp_url = data.get("rtsp_url", f"camera_{camera_index}")
        pixel_points = np.float32(data["pixel_points"])
        world_points = np.float32(data["world_points"])

        if pixel_points.shape != (4, 2) or world_points.shape != (4, 2):
            return jsonify({"status": "error", "error": "Need exactly 4 points (each [x,y])"}), 400

        # Compute the 3x3 perspective transform matrix
        matrix = cv2.getPerspectiveTransform(pixel_points, world_points)

        # Compute zone_bounds from the world points (bounding box)
        x_min = float(np.min(world_points[:, 0]))
        y_min = float(np.min(world_points[:, 1]))
        x_max = float(np.max(world_points[:, 0]))
        y_max = float(np.max(world_points[:, 1]))
        zone_bounds = [x_min, y_min, x_max, y_max]

        # Load or create cameras.json
        cameras = []
        if os.path.exists(CAMERAS_JSON_PATH):
            try:
                with open(CAMERAS_JSON_PATH, "r") as f:
                    cameras = json.load(f)
            except Exception:
                cameras = []

        # Upsert entry for this camera index
        entry = {
            "camera_index": camera_index,
            "rtsp_url": rtsp_url,
            "homography_matrix": matrix.tolist(),
            "zone_bounds": zone_bounds
        }

        found = False
        for i, cam in enumerate(cameras):
            if cam.get("camera_index") == camera_index:
                cameras[i] = entry
                found = True
                break
        if not found:
            cameras.append(entry)

        # Save
        with open(CAMERAS_JSON_PATH, "w") as f:
            json.dump(cameras, f, indent=2)

        log(f"Camera {camera_index} calibrated. Matrix saved to {CAMERAS_JSON_PATH}")
        return jsonify({"status": "ok", "matrix": matrix.tolist()})

    except Exception as e:
        log(f"Calibration error: {e}")
        return jsonify({"status": "error", "error": str(e)}), 500


@app.route("/cameras")
def get_cameras():
    """Return current cameras.json contents."""
    if os.path.exists(CAMERAS_JSON_PATH):
        try:
            with open(CAMERAS_JSON_PATH, "r") as f:
                return jsonify(json.load(f))
        except Exception as e:
            return jsonify({"error": str(e)}), 500
    return jsonify([])


if __name__ == "__main__":
    log("Starting Camera Calibration App...")
    log("Open http://<RPi4_IP>:5000 in your browser")
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
