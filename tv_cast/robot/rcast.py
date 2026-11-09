#!/usr/bin/env python3
# rcast - minimal DLNA video caster, Python 3.5 compatible (for the Yanshee / Raspbian Stretch).
# Serves one video file over HTTP and tells the TV to play it; --loop repeats it.
import sys, os, re, socket, threading, time, html, argparse, urllib.request
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

AVT = "urn:schemas-upnp-org:service:AVTransport:1"
FILE = None

class TServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

def local_ip(host):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((host, 9197)); return s.getsockname()[0]
    finally:
        s.close()

class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass
    def do_HEAD(self):
        self._serve(True)
    def do_GET(self):
        self._serve(False)
    def _serve(self, head):
        p = FILE
        if not p or not os.path.isfile(p):
            self.send_response(404); self.end_headers(); return
        sz = os.path.getsize(p)
        rng = self.headers.get("Range"); a = 0; b = sz - 1
        if rng:
            m = re.match(r"bytes=(\d+)-(\d*)", rng)
            if m:
                a = int(m.group(1)); b = int(m.group(2)) if m.group(2) else sz - 1
        self.send_response(206 if rng else 200)
        self.send_header("Content-Type", "video/mp4")
        self.send_header("contentFeatures.dlna.org",
                         "DLNA.ORG_OP=01;DLNA.ORG_CI=0;DLNA.ORG_FLAGS=01700000000000000000000000000000")
        self.send_header("transferMode.dlna.org", "Streaming")
        self.send_header("Accept-Ranges", "bytes")
        if rng:
            self.send_header("Content-Range", "bytes {}-{}/{}".format(a, b, sz))
        self.send_header("Content-Length", str(b - a + 1))
        self.end_headers()
        if head:
            return
        with open(p, "rb") as f:
            f.seek(a); left = b - a + 1
            while left > 0:
                chunk = f.read(min(262144, left))
                if not chunk:
                    break
                try:
                    self.wfile.write(chunk)
                except Exception:
                    return
                left -= len(chunk)

def soap(tv, action, body):
    env = ('<?xml version="1.0"?><s:Envelope xmlns:s="http://schemas.xmlsoap.org/soap/envelope/" '
           's:encodingStyle="http://schemas.xmlsoap.org/soap/encoding/">'
           '<s:Body><u:{a} xmlns:u="{n}">{b}</u:{a}></s:Body></s:Envelope>').format(a=action, n=AVT, b=body).encode()
    req = urllib.request.Request("http://{}/upnp/control/AVTransport1".format(tv), data=env,
          headers={"Content-Type": 'text/xml; charset="utf-8"', "SOAPACTION": '"{}#{}"'.format(AVT, action)})
    try:
        return urllib.request.urlopen(req, timeout=8).read().decode("utf-8", "ignore")
    except Exception as e:
        return "<err>{}</err>".format(e)

def push(tv, url, title):
    feat = "DLNA.ORG_OP=01;DLNA.ORG_CI=0;DLNA.ORG_FLAGS=01700000000000000000000000000000"
    meta = html.escape(
        '<DIDL-Lite xmlns="urn:schemas-upnp-org:metadata-1-0/DIDL-Lite/" '
        'xmlns:dc="http://purl.org/dc/elements/1.1/" xmlns:upnp="urn:schemas-upnp-org:metadata-1-0/upnp/">'
        '<item id="0" parentID="-1" restricted="1"><dc:title>{t}</dc:title>'
        '<upnp:class>object.item.videoItem</upnp:class>'
        '<res protocolInfo="http-get:*:video/mp4:{f}">{u}</res></item></DIDL-Lite>'.format(
            t=html.escape(title), f=feat, u=html.escape(url)))
    soap(tv, "SetAVTransportURI",
         "<InstanceID>0</InstanceID><CurrentURI>{}</CurrentURI><CurrentURIMetaData>{}</CurrentURIMetaData>".format(
             html.escape(url), meta))
    soap(tv, "Play", "<InstanceID>0</InstanceID><Speed>1</Speed>")

def state(tv):
    m = re.search(r"<CurrentTransportState>([^<]*)", soap(tv, "GetTransportInfo", "<InstanceID>0</InstanceID>"))
    return m.group(1) if m else "?"

def daemonize(logpath):
    # double-fork so the process fully detaches from the controlling SSH session
    if os.fork() > 0:
        os._exit(0)
    os.setsid()
    if os.fork() > 0:
        os._exit(0)
    sys.stdout.flush(); sys.stderr.flush()
    dn = open(os.devnull, "r")
    lf = open(logpath, "a")
    os.dup2(dn.fileno(), 0)
    os.dup2(lf.fileno(), 1)
    os.dup2(lf.fileno(), 2)

def main():
    global FILE
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("--tv", default="192.168.89.9")
    ap.add_argument("--port", type=int, default=8808)
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--daemon", action="store_true")
    ap.add_argument("--log", default=os.path.expanduser("~/rcast.log"))
    a = ap.parse_args()
    FILE = os.path.abspath(a.file)
    if not os.path.isfile(FILE):
        sys.exit("no such file: " + FILE)
    if a.daemon:
        daemonize(a.log)
    tv = a.tv + ":9197"; ip = local_ip(a.tv)
    srv = TServer(("0.0.0.0", a.port), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = "http://{}:{}/{}".format(ip, a.port, urllib.request.quote(os.path.basename(FILE)))
    print("serving {} as {} to TV {}".format(FILE, url, tv), flush=True)
    push(tv, url, os.path.basename(FILE))
    print("pushed to TV; state now: {}".format(state(tv)), flush=True)
    if a.loop:
        # repeat forever: re-push whenever playback has ended
        time.sleep(8)
        while True:
            st = state(tv)
            if st in ("STOPPED", "NO_MEDIA_PRESENT", "PAUSED_PLAYBACK"):
                push(tv, url, os.path.basename(FILE))
            time.sleep(5)
    else:
        # play ONCE: wait for playback to begin, then for it to finish, then exit
        began = False
        t0 = time.time()
        while time.time() - t0 < 25:
            if state(tv) in ("PLAYING", "TRANSITIONING", "PAUSED_PLAYBACK"):
                began = True
                break
            time.sleep(2)
        if began:
            while True:
                if state(tv) in ("STOPPED", "NO_MEDIA_PRESENT"):
                    break
                time.sleep(3)
        print("playback finished; exiting (played once)", flush=True)
        time.sleep(1)

if __name__ == "__main__":
    main()
