#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
数字孪生桥 — 网页仿真 ⇄ RM_Begin UserApp 真链路

  * 静态服务器  http://127.0.0.1:8000   （直接服 web_sim 网页，页面零改动）
  * WebSocket   ws://127.0.0.1:8765     （替代 start.py 的参考后端位）
  * 子进程      twin_harness.exe        （C 业务链：rc_cmd→合成→手感→逆解→关节积分）

与 start.py（参考后端）二选一运行：都占 8765，桥不启动时孪生走自带后端；
桥启动时网页推杆的 15 字节帧会经过真车同款 C 代码再回来驱动 3D 模型——
看到的手感曲线、急停闩锁、失联渐停即为固件行为，不是复刻。

上行帧解码同 start.py（15B 定长，<Bhhh5bHB）；C 端协议见 twin_harness.c 头注释。
telemetry 对照：网页"上行模式"切到"两者都发"后，本桥自动比对孪生积分出的
关节角与 C 链 Grab 目标角（每 50 帧打印一行最大偏差），量纲换算 rad→deg+90°，
爪开合（joint[3]，孪生为 mm 行程）跳过。

只用 Python 标准库。控制台： v = 详细日志   q = 退出
"""
import argparse, base64, hashlib, http.server, json, math, os, socket, socketserver
import struct, subprocess, sys, threading, time, webbrowser

GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
FRAME_LEN = 15
RC_FMT_LE = "<Bhhh5bHB"                      # 同 start.py：seq,3×i16,5×i8,keys,flags
assert struct.calcsize(RC_FMT_LE) == FRAME_LEN

# ------------------------------------------------------------ web_sim 目录定位
def find_web_dir():
    # 本脚本在 <repo>/PcTwin/ 下：上跳三级到仓库再上一层（WorkBuddy），
    # 数字孪生工程与 RM_Begin 平级放置；都不在就用 --web-dir 显式指定
    cands = []
    up3 = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    for name in ("2026-10-10-19-37-12/digital_twin_car/web_sim",
                 "digital_twin_car/web_sim"):
        cands.append(os.path.join(up3, *name.split("/")))
    for c in cands:
        if os.path.isfile(os.path.join(c, "index.html")):
            return c
    return None


# ------------------------------------------------------------ HTTP 静态服
class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, fmt, *args):
        if "404" in (fmt % args) or "500" in (fmt % args):
            sys.stderr.write("  [http] " + (fmt % args) + "\n")


def serve_http(web_dir, port):
    os.chdir(web_dir)                        # SimpleHTTPRequestHandler 以 cwd 为根
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.ThreadingTCPServer(("127.0.0.1", port), Handler) as srv:
        srv.daemon_threads = True
        srv.serve_forever()


# ------------------------------------------------------------ WebSocket 基础（同 start.py）
def _recv_exact(conn, n):
    buf = b""
    while len(buf) < n:
        try:
            c = conn.recv(n - len(buf))
        except OSError:
            return None
        if not c:
            return None
        buf += c
    return buf


def _ws_handshake(conn):
    data = b""
    conn.settimeout(10)
    while b"\r\n\r\n" not in data:
        chunk = conn.recv(4096)
        if not chunk:
            return False
        data += chunk
        if len(data) > 65536:
            return False
    key = None
    for line in data.decode("latin-1").split("\r\n")[1:]:
        if ":" in line:
            k, v = line.split(":", 1)
            if k.strip().lower() == "sec-websocket-key":
                key = v.strip()
    if not key:
        return False
    accept = base64.b64encode(hashlib.sha1((key + GUID).encode()).digest()).decode()
    conn.sendall(("HTTP/1.1 101 Switching Protocols\r\n"
                  "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                  "Sec-WebSocket-Accept: %s\r\n\r\n" % accept).encode())
    conn.settimeout(None)
    return True


def _ws_recv(conn):
    hdr = _recv_exact(conn, 2)
    if not hdr:
        return None
    b0, b1 = hdr[0], hdr[1]
    opcode = b0 & 0x0F
    masked = b1 >> 7
    ln = b1 & 0x7F
    if ln == 126:
        ln = struct.unpack("!H", _recv_exact(conn, 2))[0]
    elif ln == 127:
        ln = struct.unpack("!Q", _recv_exact(conn, 8))[0]
    mask = _recv_exact(conn, 4) if masked else None
    payload = _recv_exact(conn, ln) if ln else b""
    if payload is None:
        return None
    if mask:
        payload = bytes(payload[i] ^ mask[i % 4] for i in range(len(payload)))
    return opcode, payload


def _ws_send(conn, lock, text):
    data = text.encode("utf-8")
    n = len(data)
    if n < 126:
        hdr = struct.pack("!BB", 0x81, n)
    elif n < 65536:
        hdr = struct.pack("!BBH", 0x81, 126, n)
    else:
        hdr = struct.pack("!BBQ", 0x81, 127, n)
    with lock:
        try:
            conn.sendall(hdr + data)
        except OSError:
            pass


# ------------------------------------------------------------ C 业务链子进程
class Harness:
    def __init__(self, exe_path):
        self.exe = exe_path
        self.p = None
        self.lock = threading.Lock()
        self.verbose = False
        self.n_cmd = 0
        self._tele_n = 0
        self._last_state = None            # 最近一条 S 行解析出的 C 链状态
        self.last_cfg = None               # C 链报来的满量程配置（robot_config.h 宏直出）

    def start(self):
        if not os.path.isfile(self.exe):
            print("  [harness] 找不到 %s——先构建：cmake -S PcTwin -B PcTwin/build "
                  "&& cmake --build PcTwin/build" % self.exe)
            sys.exit(1)
        self.p = subprocess.Popen([self.exe], stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, text=True, bufsize=1)

    def feed_rc(self, payload_bytes):
        """解码 15B 帧 → 喂 C 链 → 回 (下行 json 文本, C 链状态 dict)"""
        (seq, vx, vy, om, j0, j1, j2, j3, j4, keys, flags) = struct.unpack(
            RC_FMT_LE, payload_bytes[:FRAME_LEN])
        t_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
        line = "F,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n" % (
            t_ms, seq, vx, vy, om, j0, j1, j2, j3, j4, keys, flags)
        with self.lock:
            self.p.stdin.write(line)
            self.p.stdin.flush()
            out_c = self.p.stdout.readline().strip()
            out_s = self.p.stdout.readline().strip()
        if not out_c.startswith("C ") or not out_s.startswith("S "):
            return None, None
        self.n_cmd += 1
        try:
            state = json.loads(out_s[2:])
        except Exception:
            state = None
        self._last_state = state
        if isinstance(state, dict) and isinstance(state.get("cfg"), dict) \
                and state["cfg"] != self.last_cfg:
            self.last_cfg = state["cfg"]
            broadcast(json.dumps({"type": "config",
                                  "chassis": {"vx_max_mps": state["cfg"].get("vx_max_mps"),
                                              "vy_max_mps": state["cfg"].get("vy_max_mps"),
                                              "w_max_radps": state["cfg"].get("w_max_radps")},
                                  "joint": {"rate_dps": state["cfg"].get("joint_rate_dps")}}))
        if self.verbose and (self.n_cmd % 100 == 0):
            print("  [harness] %d 帧 | C链角 %s | 轮速rpm %s" % (
                self.n_cmd,
                state.get("deg") if state else "?",
                state.get("rpm") if state else "?"))
        return out_c[2:], state

    def compare_telemetry(self, msg):
        """孪生积分出的关节角 vs C 链 Grab 目标角（rad→deg+90°，爪[3]跳过）"""
        self._tele_n += 1
        if self._tele_n % 50 != 0:
            return
        state = self._last_state
        jrad = (msg.get("state") or {}).get("joint_rad")
        if not state or not jrad or len(jrad) != 5:
            return
        worst, worst_i = 0.0, -1
        for i in range(5):
            if i == 3:                     # 爪开合：孪生是 mm 行程，不同量纲
                continue
            twin_deg = math.degrees(jrad[i]) + 90.0
            dev = abs(twin_deg - state["deg"][i])
            if dev > worst:
                worst, worst_i = dev, i
        print("  [对表] 孪生vsC链 关节角最大偏差 %.2f° (joint[%d]) | 轮速孪生 %s" % (
            worst, worst_i,
            {k: round(v, 1) for k, v in
             ((msg.get("state") or {}).get("wheel") or {}).items()}))


HARNESS = None
CLIENTS = set()
CLIENTS_LOCK = threading.Lock()


def broadcast(text):
    with CLIENTS_LOCK:
        targets = list(CLIENTS)
    for conn, lock in targets:
        _ws_send(conn, lock, text)


def _client(conn, addr):
    lock = threading.Lock()
    try:
        if not _ws_handshake(conn):
            conn.close(); return
        print("  [ws] 已连接 %s:%s" % addr)
        with CLIENTS_LOCK:
            CLIENTS.add((conn, lock))
        _ws_send(conn, lock, json.dumps({"type": "hello", "robot": "mecanum_car_arm",
                                         "proto": 1, "server": "rm-begin-userapp-bridge",
                                         "uplink": "binary-rc-15B"}))
        if HARNESS is not None and HARNESS.last_cfg:      # 迟连上的网页补发车端配置
            cfg = HARNESS.last_cfg
            _ws_send(conn, lock, json.dumps(
                {"type": "config",
                 "chassis": {"vx_max_mps": cfg.get("vx_max_mps"),
                             "vy_max_mps": cfg.get("vy_max_mps"),
                             "w_max_radps": cfg.get("w_max_radps")},
                 "joint": {"rate_dps": cfg.get("joint_rate_dps")}}))
        while True:
            got = _ws_recv(conn)
            if got is None:
                break
            op, payload = got
            if op == 0x8:
                break
            if op == 0x9:                                     # ping -> pong
                with lock:
                    try: conn.sendall(struct.pack("!BB", 0x8A, len(payload)) + payload)
                    except OSError: pass
                continue

            if op == 0x2 and len(payload) >= FRAME_LEN:       # 二进制遥控帧
                out, state = HARNESS.feed_rc(payload)
                if out is not None:
                    broadcast(out)
            elif op == 0x1:                                   # 文本（hello/telemetry）
                try:
                    msg = json.loads(payload.decode("utf-8"))
                except Exception:
                    continue
                if msg.get("type") == "telemetry":
                    HARNESS.compare_telemetry(msg)
    except Exception as e:
        print("  [ws] 异常:", e)
    finally:
        with CLIENTS_LOCK:
            CLIENTS.discard((conn, lock))
        try: conn.close()
        except OSError: pass
        print("  [ws] 断开 %s:%s" % addr)


def serve_ws(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port)); s.listen(8)
    while True:
        try:
            conn, addr = s.accept()
        except OSError:
            break
        threading.Thread(target=_client, args=(conn, addr), daemon=True).start()


def console():
    if not sys.stdin or not sys.stdin.isatty():
        print("  (无交互终端——后台运行，Ctrl+C 退出)")
        try:
            while True:
                time.sleep(3600)
        except KeyboardInterrupt:
            pass
        return
    print("\n控制台：  v = 详细日志    q = 退出\n")
    for line in sys.stdin:
        c = line.strip().lower()
        if c == "q":
            os._exit(0)
        elif c == "v":
            HARNESS.verbose = not HARNESS.verbose
            print("  [bridge] 详细日志 =", HARNESS.verbose)


def main():
    ap = argparse.ArgumentParser(description="RM_Begin UserApp 数字孪生桥")
    ap.add_argument("--harness", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "build", "twin_harness.exe"))
    ap.add_argument("--web-dir", default=None, help="web_sim 目录（默认自动定位）")
    ap.add_argument("--http-port", type=int, default=8000)
    ap.add_argument("--ws-port", type=int, default=8765)
    ap.add_argument("--no-open", action="store_true")
    args = ap.parse_args()

    global HARNESS
    HARNESS = Harness(args.harness)
    HARNESS.start()
    print("  [harness] C 业务链已启动：%s" % args.harness)

    web_dir = args.web_dir or find_web_dir()
    if web_dir is None:
        print("  [bridge] 定位不到 web_sim 目录——用 --web-dir 显式指定"
              "（数字孪生工程目录下的 web_sim）")
        sys.exit(1)
    print("  [bridge] 网页目录：%s" % web_dir)

    threading.Thread(target=serve_http, args=(web_dir, args.http_port),
                     daemon=True).start()
    threading.Thread(target=serve_ws, args=(args.ws_port,), daemon=True).start()
    time.sleep(0.4)
    print("=" * 64)
    print("  RM_Begin UserApp × 数字孪生 桥接")
    print("=" * 64)
    print("  网页      http://127.0.0.1:%d/   （推杆经真车 C 链回驱模型）" % args.http_port)
    print("  WebSocket ws://127.0.0.1:%d    （与 start.py 参考后端二选一）" % args.ws_port)
    print("  对照      网页上行模式切\"两者都发\"→ 每 50 帧打印关节角偏差")
    print("-" * 64)
    if not args.no_open:
        try:
            webbrowser.open("http://127.0.0.1:%d/" % args.http_port)
        except Exception:
            pass
    console()


if __name__ == "__main__":
    main()
