#!/usr/bin/env python3
"""Live RGB and IR in a web browser: open http://<pi-address>:8000

Reads /dev/vd16gz-rgb and /dev/vd16gz-ir and serves them as MJPEG streams
(/rgb.mjpg, /ir.mjpg) plus single snapshots (/rgb.jpg, /ir.jpg).
"""
import argparse
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

PAGE = """<!doctype html><title>VD16GZ RGB-IR</title>
<style>body{background:#111;color:#ddd;font:14px sans-serif;margin:16px}
div{display:flex;flex-wrap:wrap;gap:16px}figure{margin:0}img{max-width:100%;height:auto}</style>
<h3>VD16GZ RGB-IR live</h3>
<div><figure><img src="/rgb.mjpg"><figcaption>RGB</figcaption></figure>
<figure><img src="/ir.mjpg"><figcaption>IR</figcaption></figure></div>"""


class Stream(threading.Thread):
    """Reads one device and keeps the newest frame as JPEG."""

    def __init__(self, path, quality, scale):
        super().__init__(daemon=True)
        self.cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise SystemExit(f"cannot open {path}; is the vd16gz-bridge service running?")
        self.quality, self.scale = quality, scale
        self.jpeg, self.seq = b"", 0
        self.cond = threading.Condition()
        self.start()

    def run(self):
        while True:
            ok, img = self.cap.read()
            if not ok:
                continue
            if self.scale != 1:
                img = cv2.resize(img, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
            jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])[1].tobytes()
            with self.cond:
                self.jpeg, self.seq = jpeg, self.seq + 1
                self.cond.notify_all()


def make_handler(streams):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def send(self, body, ctype):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/":
                return self.send(PAGE.encode(), "text/html")
            name, _, ext = path[1:].partition(".")
            if name not in streams or ext not in ("jpg", "mjpg"):
                return self.send_error(404)
            s = streams[name]
            if ext == "jpg":
                return self.send(s.jpeg, "image/jpeg")
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            seq = -1
            try:
                while True:
                    with s.cond:
                        s.cond.wait_for(lambda: s.seq != seq)
                        seq, jpg = s.seq, s.jpeg
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpg))
                    self.wfile.write(jpg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--quality", type=int, default=80, help="JPEG quality (default 80)")
    ap.add_argument("--scale", type=float, default=0.5, help="resize factor before encoding (default 0.5)")
    args = ap.parse_args()
    streams = {name: Stream(f"/dev/vd16gz-{name}", args.quality, args.scale) for name in ("rgb", "ir")}
    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(streams))
    server.daemon_threads = True
    print(f"Streaming on http://0.0.0.0:{args.port}/", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
