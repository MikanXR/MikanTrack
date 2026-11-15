"""Serve the Kalidokit render page over a take's raw frames and save the images it posts back.

  python serve_render.py <recording_frames_dir> <cameraIndex> <file.vrm> <out_dir> [port]

Open http://localhost:<port>/ in a browser. The page runs MediaPipe Holistic and Kalidokit on each raw
frame in order, poses the VRM as Kalidokit's 3d sample does, and posts one image per frame, which lands
in <out_dir> as frame_<frameIndex>.jpg. /status reports how many are saved.
"""

import glob
import json
import os
import re
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
frames_dir, camera, vrm_path, out_dir = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]
port = int(sys.argv[5]) if len(sys.argv) > 5 else 8765
os.makedirs(out_dir, exist_ok=True)

frame_indices = sorted(
    int(re.search(r"_(\d+)\.jpg$", path).group(1)) for path in glob.glob(os.path.join(frames_dir, f"cam{camera}_*.jpg")))

STATIC = {"/": ("render.html", "text/html"), "/render.js": ("render.js", "application/javascript")}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def reply(self, body, content_type, status=200):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def reply_file(self, path, content_type):
        with open(path, "rb") as f:
            self.reply(f.read(), content_type)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in STATIC:
            name, content_type = STATIC[path]
            self.reply_file(os.path.join(HERE, name), content_type)
        elif path == "/frames.json":
            self.reply(json.dumps(frame_indices).encode(), "application/json")
        elif path == "/status":
            saved = len(glob.glob(os.path.join(out_dir, "frame_*.jpg")))
            self.reply(json.dumps({"frames": len(frame_indices), "saved": saved}).encode(), "application/json")
        elif path == "/vrm":
            self.reply_file(vrm_path, "model/gltf-binary")
        elif re.fullmatch(r"/frame/\d+\.jpg", path):
            index = int(re.search(r"\d+", path).group(0))
            self.reply_file(os.path.join(frames_dir, f"cam{camera}_{index}.jpg"), "image/jpeg")
        else:
            self.reply(b"not found", "text/plain", 404)

    def do_POST(self):
        match = re.fullmatch(r"/save/(\d+)", self.path)
        if not match:
            self.reply(b"not found", "text/plain", 404)
            return
        body = self.rfile.read(int(self.headers["Content-Length"]))
        with open(os.path.join(out_dir, f"frame_{match.group(1)}.jpg"), "wb") as f:
            f.write(body)
        self.reply(b"ok", "text/plain")


print(f"{len(frame_indices)} frames of camera {camera} on http://localhost:{port}/", flush=True)
ThreadingHTTPServer(("127.0.0.1", port), Handler).serve_forever()
