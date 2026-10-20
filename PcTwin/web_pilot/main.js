/**
 * WebPilot 网页遥控台主逻辑（原生脚本，零构建）
 *
 * 线协议见同目录 PROTOCOL.md（与固件 Hardware/Remoter/UserApp/web_input.h 对表）：
 * 下行 12B 定长帧（0x5A + wseq + vx/vy/omega 三轴逻辑量 ±1000 + keys 7B 镜像
 * + CRC8），20Hz 心跳 + 变化即发（≥15ms 间隔）；上行 [RMT] 文本统计行。
 * v2（2026-10-20）：帧加 omega 轴——整车旋转支持线性控制（回转挡的杆 X）；
 * 网页不再发 K_6 位（旋转统走 omega 轴），与旧固件帧长错配=看门狗安全停发，
 * 页面与板必须同批升级。
 *
 * 挡位（对应实机单杆+SW 切挡的交互模型，网页=实机超集预演）：
 *  - 「前进·左右」：杆 Y=前后，杆 X=左右平移；
 *  - 「前进×回转」：杆 Y=前后，杆 X=线性回转；
 *  - 切换=左杆下压（L3）或屏上按钮，杆回中才许切（防弯中翻语义突跳）；
 *  - 右杆 X 恒=K_6 等位（定速率三态，同 Q/E 与屏上旋转开关），粗调旋转；
 *  - 键盘 WASD+Q/E 不受挡位影响。
 *
 * 安全行为（拍板 2026-10-16）：
 *  - 断连（拔线/浏览器崩/关标签）：停发帧 → 板 500ms 看门狗停发 RF → 车渐停+告警；
 *  - 急停 = 点击闩锁，再点解除（与物理轻触急停的"按住置位"在帧字段同构）；
 *  - 后台标签页被节流 → 心跳掉线 → 车自动渐停：特性非故障，横幅提示。
 */

"use strict";

/* ==================== 线协议（与 PROTOCOL.md / web_input.h 逐字节对表） ==================== */

const HEADER = 0x5A;
const FRAME_LEN = 12;
const BAUD = 115200;

/** CRC8：poly 0x07、初值 0x00、不反射、无异或输出（"123456789" → 0xF4） */
function crc8(bytes) {
  let c = 0x00;
  for (const b of bytes) {
    c ^= b;
    for (let i = 0; i < 8; i++) {
      c = (c & 0x80) ? ((c << 1) ^ 0x07) & 0xFF : (c << 1) & 0xFF;
    }
  }
  return c;
}

/** 打一帧 12B：小端，字段布局见 PROTOCOL.md §2（vx/vy/omega=轴逻辑量 ±1000，i16） */
function packFrame(wseq, vx, vy, omega, keys, estop) {
  const f = new Uint8Array(FRAME_LEN);
  f[0] = HEADER;
  f[1] = wseq & 0xFF;
  const dv = new DataView(f.buffer);
  dv.setInt16(2, clampAxis(vx), true);    // 小端：低字节在前，有符号
  dv.setInt16(4, clampAxis(vy), true);
  dv.setInt16(6, clampAxis(omega), true);
  dv.setUint16(8, keys & 0xFFFF, true);
  f[10] = estop ? 1 : 0;
  f[11] = crc8(f.subarray(1, 11));        // CRC 覆盖 [1..10]
  return f;
}

/* 轴逻辑量钳位 ±1000（发送端纪律，rc_cmd.h v2：vx/vy/omega 由发送端钳位） */
function clampAxis(v) { return Math.max(-1000, Math.min(1000, Math.round(v))); }

/* keys 位图（与 rc_input.h 宏组一致）：每开关 2 位 A/B。网页只用 K_1..K_5
 * （bit0..bit9）——旋转统走 omega 轴，K_6 位（bit10/11）网页不再置位，
 * 该两位语义归还 PHYS 物理开关 */
const KEY_BITS = [];
for (let n = 0; n < 6; n++) KEY_BITS.push({ a: 1 << (2 * n), b: 1 << (2 * n + 1) });

/* ==================== 输入状态（唯一真值，打包函数只读这里） ==================== */

const input = {
  vx: 0,               // 轴逻辑量 ±1000（0=回中），右/前为正
  vy: 0,
  omega: 0,            // 整车旋转轴逻辑量 ±1000（推左=+，同 Q）
  keys: 0,             // 方向刀位图（网页只产 K_1..K_5）
  estop: false,        // 急停闩锁
  wseq: 0,
};

/* keys 只剩屏开关+键盘 K_1..K_5 的位（旋转已出 keys，无需多源合成） */
let swBits = 0;

/* 旋转的三个"K_6 化身"共用一份定速率票：Q/E 键、屏上旋转开关、手柄右杆 X。
 * 正负票同时存在=矛盾，失安全归零（同板端 FoldOmega 语义） */
let rotQ = false, rotE = false, rotSwUp = false, rotSwDn = false;
function rotVote() {
  const pos = rotQ || rotSwUp, neg = rotE || rotSwDn;
  return pos && neg ? 0 : pos ? 1 : neg ? -1 : 0;
}

/* ==================== 挡位（单杆+SW 切挡模型，实机网页同构） ==================== */

const MODE_TRANSLATE = 0;   // 前进·左右：杆 X=平移
const MODE_ROTATE = 1;      // 前进×回转：杆 X=线性回转
let mode = MODE_TRANSLATE;

function modeName() { return mode === MODE_ROTATE ? "前进×回转" : "前进·左右"; }

function renderMode() {
  const btn = $("btn-mode");
  btn.textContent = `挡位：${modeName()}`;
  btn.classList.toggle("mode-rotate", mode === MODE_ROTATE);
}

function toggleMode(source) {
  // 手柄杆还压着不许切：切挡=杆 X 换语义，带弯切会指令突跳抽车
  if (padIndex !== null) {
    const gp = navigator.getGamepads()[padIndex];
    if (gp && gp.connected && !sticksCentered(gp)) {
      logLine("〔挡位〕杆未回中，切挡忽略（回中再切，防指令突跳）", "line-evt");
      return;
    }
  }
  mode = mode === MODE_TRANSLATE ? MODE_ROTATE : MODE_TRANSLATE;
  if (joyDragging && lastDragNy !== null) applyScreenStick(lastDragNx, lastDragNy);
  renderMode();
  logLine(`〔挡位〕切到「${modeName()}」（${source}）——杆 X=${mode === MODE_ROTATE ? "线性回转" : "左右平移"}`, "line-evt");
  dirty = true; maybeSend();
}

/* ==================== 心跳与脏发送 ==================== */

let dirty = true;        // 输入有变 → 立即发帧（受 15ms 最小间隔约束）
let lastSent = 0;
let sentCount = 0;

/* ==================== 串口（Web Serial API） ==================== */

let port = null;
let writer = null;
let keepReading = false;
let serialOpen = false;
let boardWebUp = false;  // 板端 [RMT] w= 状态
let lastRmtMs = 0;
let rmt = null;          // 最新统计行解析结果

const $ = (id) => document.getElementById(id);

function setBanner(text) {
  const el = $("banner");
  if (text) { el.textContent = text; el.classList.remove("hidden"); }
  else { el.classList.add("hidden"); }
}

function logLine(text, cls) {
  const box = $("log");
  const div = document.createElement("div");
  div.className = cls || "";
  div.textContent = text;
  box.appendChild(div);
  while (box.childElementCount > 200) box.removeChild(box.firstChild);
  box.scrollTop = box.scrollHeight;
}

function setLight(id, on, bad) {
  const el = $(id);
  el.classList.toggle("on", !!on);
  el.classList.toggle("bad", !!bad);
}

/* ---- 发送 ---- */

async function sendFrame() {
  if (!writer) return;
  const f = packFrame(input.wseq, input.vx, input.vy, input.omega, input.keys, input.estop);
  try {
    await writer.write(f);
    input.wseq = (input.wseq + 1) & 0xFF;
    sentCount++;
    lastSent = performance.now();
    dirty = false;
    updateTxHud(f);
  } catch (e) {
    logLine(`[web] 发送失败：${e.message}`, "line-evt");
    onSerialLost("写入失败");
  }
}

function maybeSend() {
  if (!writer) return;
  const now = performance.now();
  if (dirty && now - lastSent >= 15) sendFrame();
}

/* 心跳：50ms 定时器。停发不能赌浏览器节流——开着 Web Serial 串口的页面，
 * Chromium 不做后台定时器节流（2026-10-17 实测：切后台 20Hz 照跑，板端
 * 500ms 看门狗永不超窗，车端链路断不掉）。显式停发在 visibilitychange 里做，
 * 这里加 hidden 闸门做双保险 */
setInterval(() => { if (writer && !document.hidden) sendFrame(); }, 50);
/* 切后台 = 显式停发：收尾发一帧当前态再静默（急停位保持原值，不误放闩锁；
 * 指令位清零）——板端 500ms 看门狗超窗停发 RF，车端走已验的渐停+断链声光链；
 * 回前台补一帧立即恢复 */
document.addEventListener("visibilitychange", () => {
  if (!document.hidden) {
    setBanner(null);
    if (writer) { dirty = true; maybeSend(); }
  } else {
    if (writer) {
      const f = packFrame(input.wseq, 0, 0, 0, 0, input.estop);
      try { writer.write(f); input.wseq = (input.wseq + 1) & 0xFF; } catch (e) { /* 尽力而为 */ }
    }
    setBanner("已切到后台：心跳已停发，车端将自动渐停断链。回到前台自动恢复。");
  }
});

/* 页面卸载尽力发一帧全零（礼貌收尾，非安全依赖——真兜底是板上看门狗） */
window.addEventListener("pagehide", () => {
  if (!writer) return;
  const f = packFrame(input.wseq, 0, 0, 0, 0, 0);
  try { writer.write(f); } catch (e) { /* 尽力而为 */ }
});

/* ---- 连接管理 ---- */

async function connect() {
  if (!("serial" in navigator)) {
    setBanner("此浏览器不支持 Web Serial API——请用 Chrome/Edge（需 https、localhost 或 file:// 等安全上下文）。");
    return;
  }
  try {
    port = await navigator.serial.requestPort();
    await port.open({ baudRate: BAUD });
  } catch (e) {
    logLine(`[web] 打开串口失败：${e.message}`, "line-evt");
    return;
  }
  writer = port.writable.getWriter();
  serialOpen = true;
  boardWebUp = false;
  resetInputToZero();
  $("btn-connect").disabled = true;
  $("btn-disconnect").disabled = false;
  $("conn-info").textContent = "已连接 @115200";
  setLight("light-serial", true);
  logLine("[web] 串口已打开，输入已复位为全零（等板端 [RMT] 行对账）", "line-evt");
  readLoop();
  navigator.serial.addEventListener("disconnect", onSerialLost);
}

function resetInputToZero() {
  input.vx = 0; input.vy = 0; input.omega = 0;
  swBits = 0; input.keys = 0; input.estop = false;
  rotQ = false; rotE = false; rotSwUp = false; rotSwDn = false;
  padActive = false; padK6 = 0; screenOmega = 0;
  renderJoystick(); renderSwitches(); renderEstop(); renderInputHud();
  dirty = true; maybeSend();
}

async function disconnect(reason) {
  keepReading = false;
  try { if (writer) { writer.releaseLock(); } } catch (e) { /* 已释放 */ }
  try { if (port) await port.close(); } catch (e) { /* 已关 */ }
  writer = null; port = null; serialOpen = false; boardWebUp = false;
  $("btn-connect").disabled = false;
  $("btn-disconnect").disabled = true;
  $("conn-info").textContent = "未连接（Chrome/Edge，115200）";
  setLight("light-serial", false);
  setLight("light-web", false);
  setLight("light-rf", false);
  if (reason) {
    setBanner(`串口已断开（${reason}）：已停止发帧，板端看门狗会停发 RF，车端自动渐停。`);
    logLine(`[web] 串口断开：${reason}`, "line-evt");
  }
}

function onSerialLost(reason) { if (serialOpen) disconnect(reason || "设备移除"); }

/* ---- 收：[RMT] 文本行 ---- */

async function readLoop() {
  keepReading = true;
  const decoder = new TextDecoder();
  let buf = "";
  const reader = port.readable.getReader();
  try {
    for (;;) {
      const { value, done } = await reader.read();
      if (done || !keepReading) break;
      buf += decoder.decode(value, { stream: true });
      let nl;
      while ((nl = buf.indexOf("\n")) >= 0) {
        const line = buf.slice(0, nl).replace(/\r$/, "");
        buf = buf.slice(nl + 1);
        if (line) handleLine(line);
      }
    }
  } catch (e) {
    if (keepReading) onSerialLost(`读取中断：${e.message}`);
  } finally {
    try { reader.releaseLock(); } catch (e) { /* 已释放 */ }
  }
}

/* WEB 模式统计行（PROTOCOL.md §5）：
 * [RMT] tx=98 fail=0 seq=201 v=(311,-468) o=-500 k=0x0800 e=0 w=u ok=185 err=0 dr=12 */
const RMT_RE = /tx=(\d+) fail=(\d+) seq=(\d+) v=\((-?\d+),(-?\d+)\) o=(-?\d+) k=0x([0-9A-Fa-f]+) e=(\d+) w=(\w) ok=(\d+) err=(\d+) dr=(\d+)/;

function handleLine(line) {
  logLine(line, line.startsWith("[RMT]") ? "line-rmt" : null);
  if (!line.startsWith("[RMT]")) return;
  if (line.includes("web up")) { boardWebUp = true; logLine("〔板〕网页链路恢复", "line-evt"); }
  if (line.includes("web lost")) { boardWebUp = false; logLine("〔板〕网页链路断流（板已停发 RF）", "line-evt"); }
  const m = line.match(RMT_RE);
  if (!m) return;   // init OK 等行不进遥测
  rmt = {
    tx: +m[1], fail: +m[2], seq: +m[3],
    vx: +m[4], vy: +m[5], o: +m[6], k: parseInt(m[7], 16),
    e: +m[8], w: m[9], ok: +m[10], err: +m[11], dr: +m[12],
  };
  lastRmtMs = performance.now();
  renderTelemetry();
}

/* ==================== UI：摇杆 ==================== */

const JOY_R = 240 / 2 - 28;   // 拖拽半径（pad 240、thumb 56）
let joyDragging = false;
let joyPointerId = null;      // 当前拖拽的指针 id（多指/误触过滤）
let lastDragNx = 0, lastDragNy = null;   // 最近一次屏杆位置（null=从未拖过；切挡重映射用）
let screenOmega = 0;          // 回转挡下屏杆 X 产出的线性 omega（拖拽期间有效）

function renderJoystick() {
  // 屏横向：平移挡=vy（左推为正，2026-10-16 修正）；回转挡=omega（左旋为正）
  const nx = mode === MODE_ROTATE ? -input.omega / 1000 : -input.vy / 1000;
  const ny = input.vx / 1000;              // 屏幕纵向=vx（前进为正）
  const px = 120 + nx * JOY_R;
  const py = 120 - ny * JOY_R;             // 屏幕 y 向下，取反
  $("joy-thumb").style.left = `${px}px`;
  $("joy-thumb").style.top = `${py}px`;
  $("val-vx").textContent = input.vx;
  $("val-vy").textContent = input.vy;
  $("val-omega").textContent = input.omega;
}

/* 按当前挡位把屏杆位置映射成指令（指针拖拽与切挡重映射共用） */
function applyScreenStick(nx, ny) {
  input.vx = Math.round(ny * 1000);              // 屏上推=前进=vx+（车端纵向习惯）
  if (mode === MODE_ROTATE) {
    input.vy = 0;
    screenOmega = Math.round(-nx * 1000);        // 屏左推=+omega（同 Q/屏开关正向）
  } else {
    input.vy = Math.round(-nx * 1000);           // 屏左推=vy+（2026-10-16 实车方向二次修正）
    screenOmega = 0;
  }
  renderJoystick(); renderInputHud();
  dirty = true; maybeSend();
}

function setJoyFromPoint(ev) {
  if (padActive) return;   // 手柄接管中屏摇杆让位（杆回中自动交还，按住不松可无缝续控）
  const r = $("joy-pad").getBoundingClientRect();
  let nx = (ev.clientX - r.left - 120) / JOY_R;    // -1..+1
  let ny = -(ev.clientY - r.top - 120) / JOY_R;
  const m = Math.hypot(nx, ny);
  if (m > 1) { nx /= m; ny /= m; }                 // 方形 pad 圆形行程
  lastDragNx = nx; lastDragNy = ny;
  applyScreenStick(nx, ny);
}

const joyPad = $("joy-pad");
joyPad.addEventListener("pointerdown", (ev) => {
  ev.preventDefault();     // 防文本选择/原生拖拽抢事件流（Edge 下拖选会把杆卡住）
  if (joyDragging) return; // 已有指针在拖，第二指不抢
  joyDragging = true;
  joyPointerId = ev.pointerId;
  try { joyPad.setPointerCapture(ev.pointerId); } catch (e) { /* 假 pointerId（自动化/边缘时序）不影响拖拽 */ }
  joyPad.classList.add("active");
  setJoyFromPoint(ev);
});
joyPad.addEventListener("pointermove", (ev) => {
  if (!joyDragging || ev.pointerId !== joyPointerId) return;
  if (ev.pointerType === "mouse" && ev.buttons === 0) {
    joyRelease(ev);        // 键已抬起却没收到 up=异常残留态，就地释放
    return;
  }
  setJoyFromPoint(ev);
});
const joyRelease = (ev) => {
  if (!joyDragging) return;
  if (ev && ev.pointerId !== undefined && joyPointerId !== null
      && ev.pointerId !== joyPointerId) return;   // 只认自己这根指针
  joyDragging = false;
  joyPointerId = null;
  joyPad.classList.remove("active");
  input.vx = 0; input.vy = 0; screenOmega = 0;   // 松手弹回中位（自复位语义）
  renderJoystick(); renderInputHud();
  dirty = true; maybeSend();
};
joyPad.addEventListener("pointerup", joyRelease);
joyPad.addEventListener("pointercancel", joyRelease);
joyPad.addEventListener("lostpointercapture", joyRelease);
/* 兜底：capture 偶发失效时 pad 收不到 up——window 层再收一次
 * （pointerId 过滤，不误伤开关按钮和其他指针） */
addEventListener("pointerup", joyRelease);
addEventListener("pointercancel", joyRelease);
/* 原生拖拽（拖选/拖图）会抢走指针流，全页禁掉 */
addEventListener("dragstart", (ev) => ev.preventDefault());

/* 键盘 WASD：斜按=对角；keydown 持续压杆，keyup 回弹（动画帧内渐动）。
 * 轴习惯与车端一致：vx=纵向（W 前进/S 后退），vy=横移（A 左=+ / D 右=−）。
 * 键盘不受挡位影响——A/D 恒平移，旋转恒 Q/E */
const heldKeys = new Set();
const KEYDIR = { KeyW: [1, 0], KeyS: [-1, 0], KeyA: [0, 1], KeyD: [0, -1] };

/* ==================== 蓝牙/USB 游戏手柄（Xbox 布局） ==================== */

/* 手柄在系统里蓝牙配对（或插 USB）后，浏览器 Gamepad API 统一暴露，网页不分
 * 有线无线。Chrome 要按一下手柄任意键才认（首键前不进列表，规范防窥探行为）。
 * 布局（单杆+SW 切挡模型，与实机 H2 摇杆同构）：
 *  - 左杆 Y=前后恒定；左杆 X 随挡位（L3=左杆下压切换，实机对应 SW 键）；
 *  - 右杆 X 恒=K_6 等位定速率旋转（实机对应 K_6 船型开关）；
 *  - 杆在死区外=手柄接管 vx/vy/omega，回中自动交还 */
const PAD_DZ = 0.12;       // 左摇杆径向清噪死区：吃掉杆的回中偏移，出死区线性放大回 0..1
const PAD_ROT_DZ = 0.35;   // 右杆旋转触发阈值：定速率挡位，推过半才触发，免误碰

let padIndex = null;       // 认领的手柄槽位（多只先到先得，其余忽略）
let padActive = false;     // 任一杆出死区=true：手柄拥有 vx/vy/omega
let padK6 = 0;             // 手柄右杆贡献的定速率旋转（±1000 或 0）
let padSuspended = false;  // 失焦挂起：杆还压着也先不信（同键盘失焦全松纪律）
let padPrevL3 = false;     // 左杆下压上拍态（沿触发切挡）

function padStickRescale(x, y) {
  const m = Math.hypot(x, y);
  if (m < PAD_DZ) return [0, 0];
  const k = ((m - PAD_DZ) / (1 - PAD_DZ)) / m;   // 死区外放大，方向不变
  let ox = x * k, oy = y * k;
  const om = Math.hypot(ox, oy);
  if (om > 1) { ox /= om; oy /= om; }            // 斜推超圆缘钳回 1（屏摇杆同规矩，防轴量越 ±1000）
  return [ox, oy];
}

function sticksCentered(gp) {
  const x = gp.axes[0] || 0, y = gp.axes[1] || 0, rx = gp.axes[2] || 0;
  return Math.hypot(x, y) < PAD_DZ && Math.abs(rx) < PAD_ROT_DZ;
}

function padRelease(reason) {
  const wasActive = padActive;
  padIndex = null; padActive = false; padK6 = 0; padPrevL3 = false;
  setLight("light-pad", false);
  $("pad-info").textContent = "手柄：未连接（配对后按任意键唤醒）";
  if (reason) logLine(`[手柄] ${reason}`, "line-evt");
  if (wasActive) { renderJoystick(); renderSwitches(); dirty = true; maybeSend(); }
}

function padPoll() {
  if (padIndex === null || padSuspended) return;
  const gp = navigator.getGamepads()[padIndex];
  if (!gp || !gp.connected) { padRelease("手柄已断开：控制权交还键盘/屏开关"); return; }

  /* 左杆下压（L3，实机 SW 同位）：按下沿切挡，杆回中才受理（见 toggleMode） */
  const l3 = !!(gp.buttons[10] && gp.buttons[10].pressed);
  if (l3 && !padPrevL3) toggleMode("左杆下压");
  padPrevL3 = l3;

  const [lx, ly] = padStickRescale(gp.axes[0] || 0, gp.axes[1] || 0);
  const rxRaw = gp.axes[2] || 0;
  const rot = Math.abs(rxRaw) >= PAD_ROT_DZ ? (rxRaw < 0 ? 1 : -1) : 0;  // 推左=+omega（同 Q）
  /* 挡位决定左杆 X 身份：平移挡=vy，回转挡=线性 omega（右杆恒定速率） */
  const linOmega = mode === MODE_ROTATE ? Math.round(-lx * 1000) : 0;
  padK6 = rot * 1000;
  const nOmega = linOmega !== 0 ? linOmega : padK6;   // 线性细调优先于定速率粗调
  const nvx = Math.round(-ly * 1000);   // 推上=vx+（axes[1] 向下为正，取反）
  const nvy = mode === MODE_ROTATE ? 0 : Math.round(-lx * 1000);   // 推左=vy+（axes[0] 向右为正）
  const prevKeys = input.keys;
  padActive = (lx !== 0 || ly !== 0 || rot !== 0);
  if (padActive) {
    if (nvx !== input.vx || nvy !== input.vy || nOmega !== input.omega) { dirty = true; maybeSend(); }
    input.vx = nvx; input.vy = nvy; input.omega = nOmega;
    renderJoystick(); renderInputHud();   // 屏摇杆/数值跟着手柄动，所见即所发
  } else {
    padK6 = 0;   // 松杆交还：vx/vy 交给 WASD 动画收尾，omega 交 tick 合成（Q/E/屏开关）
  }
  if (input.keys !== prevKeys) renderSwitches();
}

addEventListener("gamepadconnected", (ev) => {
  const gp = ev.gamepad;
  if (padIndex !== null) { logLine(`[手柄] 忽略第二只：${gp.id}`, "line-evt"); return; }
  padIndex = gp.index;
  setLight("light-pad", true);
  $("pad-info").textContent = `手柄：${gp.id}`;
  logLine(`[手柄] 已连接（${gp.mapping === "standard" ? "标准 Xbox 布局" : "非标准映射，轴向可能不对"}）——左杆=前后+挡位轴（下压切挡），右杆=定速率旋转`, "line-evt");
});
addEventListener("gamepaddisconnected", (ev) => {
  if (ev.gamepad.index !== padIndex) return;   // 断的是没认领的那只，不理
  padRelease("手柄已断开：控制权交还键盘/屏开关");
});

/* 每帧输入拍：手柄先出牌（接管中屏摇杆/键盘让位），手柄空手时屏杆/键盘说话 */
function inputTick() {
  padPoll();
  if (!padActive) {
    if (!joyDragging) {
      let tx = 0, ty = 0;
      for (const c of heldKeys) { if (KEYDIR[c]) { tx += KEYDIR[c][0]; ty += KEYDIR[c][1]; } }
      const m = Math.hypot(tx, ty);
      if (m > 1) { tx /= m; ty /= m; }
      const nx = input.vx / 1000, ny = input.vy / 1000;
      const k = 0.35;                         // 每帧向目标靠拢的比例（手感：~100ms 满行程）
      let gx = nx + (tx - nx) * k, gy = ny + (ty - ny) * k;
      if (tx === 0 && Math.abs(gx) < 0.01) gx = 0;    // 松键回中收尾，不挂尾数
      if (ty === 0 && Math.abs(gy) < 0.01) gy = 0;
      input.vx = Math.round(gx * 1000);
      input.vy = Math.round(gy * 1000);
      if (gx !== nx || gy !== ny) {
        dirty = true; maybeSend();
      }
    }
    /* 旋转合成（手柄空手时）：回转挡屏杆拖拽=线性细调，否则 Q/E/屏开关定速率。
     * 屏杆拖拽的 vx/vy 已由指针事件直写，这里只管 omega */
    const om = (joyDragging && mode === MODE_ROTATE) ? screenOmega : rotVote() * 1000;
    if (om !== input.omega) {
      input.omega = om;
      dirty = true; maybeSend();
      renderJoystick(); renderSwitches(); renderInputHud();
    } else if (input.vx !== 0 || input.vy !== 0 || heldKeys.size > 0 || joyDragging) {
      renderJoystick(); renderInputHud();     // 跟随 WASD 动画/屏杆刷新
    }
  }
  requestAnimationFrame(inputTick);
}
requestAnimationFrame(inputTick);

/* ==================== UI：三态船型开关 ==================== */

/* 下标=开关号 n：K_1..K_5 关节；K_6=整车旋转——网页上它直接驱动 omega
 * （±1000 定速率，同 Q/E），不再写 keys 位（2026-10-20 起 K_6 位归 PHYS） */
const SW_DEFS = [
  { cn: "大臂", kbUp: "Digit1", kbDn: "Digit6" },
  { cn: "小臂", kbUp: "Digit2", kbDn: "Digit7" },
  { cn: "爪子", kbUp: "Digit3", kbDn: "Digit8" },
  { cn: "关节4", kbUp: "Digit4", kbDn: "Digit9" },
  { cn: "手腕", kbUp: "Digit5", kbDn: "Digit0" },
  { cn: "旋转", kbUp: "KeyQ", kbDn: "KeyE" },
];

const swEls = [];

function buildSwitches() {
  const box = $("switches");
  SW_DEFS.forEach((def, n) => {
    const wrap = document.createElement("div");
    wrap.className = "sw";
    wrap.innerHTML = `
      <span class="zone up" title="${def.cn} 正向">＋</span>
      <div class="label">${def.cn}<br><span class="tag">K_${n + 1}${n === 5 ? " 旋转" : ""}</span></div>
      <span class="zone dn" title="${def.cn} 反向">－</span>`;
    const up = wrap.querySelector(".zone.up");
    const dn = wrap.querySelector(".zone.dn");
    const press = (which, on) => {
      if (n === 5) {
        // K_6 旋转：驱动 omega 票（Q/E 同源），不进 keys
        if (which === "up") rotSwUp = on; else rotSwDn = on;
      } else {
        const bit = KEY_BITS[n];
        if (on) swBits |= (which === "up" ? bit.a : bit.b);
        else swBits &= ~(which === "up" ? bit.a : bit.b);
        input.keys = swBits;
      }
      renderSwitches(); renderInputHud();
      dirty = true; maybeSend();
    };
    const bind = (el, which) => {
      el.addEventListener("pointerdown", (ev) => {
        try { el.setPointerCapture(ev.pointerId); } catch (e) { /* 假 pointerId 不拦 press */ }
        press(which, true);
      });
      const rel = () => press(which, false);
      el.addEventListener("pointerup", rel);
      el.addEventListener("pointercancel", rel);
    };
    bind(up, "up"); bind(dn, "dn");
    swEls.push({ up, dn, def });
    box.appendChild(wrap);
  });
}

function renderSwitches() {
  swEls.forEach(({ up, dn }, n) => {
    if (n === 5) {
      // 旋转开关=omega 指示灯：无论来源（Q/E/右杆/回转挡左杆/屏开关），亮实际生效方向
      up.classList.toggle("press-up", input.omega > 0);
      dn.classList.toggle("press-dn", input.omega < 0);
      return;
    }
    const bit = KEY_BITS[n];
    up.classList.toggle("press-up", (input.keys & bit.a) !== 0);
    dn.classList.toggle("press-dn", (input.keys & bit.b) !== 0);
  });
}

/* ==================== UI：急停 ==================== */

function renderEstop() {
  $("btn-estop").classList.toggle("latched", input.estop);
}

$("btn-estop").addEventListener("click", toggleEstop);

function toggleEstop() {
  input.estop = !input.estop;
  renderEstop(); renderInputHud();
  logLine(input.estop ? "〔急停〕闩锁 ON（每帧 estop=1）" : "〔急停〕解除 OFF", "line-evt");
  dirty = true; maybeSend();
}

/* ==================== UI：键盘 ==================== */

addEventListener("keydown", (ev) => {
  if (ev.repeat) return;
  if (ev.code === "Space") { ev.preventDefault(); toggleEstop(); return; }
  if (KEYDIR[ev.code]) { ev.preventDefault(); heldKeys.add(ev.code); return; }
  if (ev.code === "KeyQ") { rotQ = true; renderSwitches(); return; }   // 旋转走 omega，不进 keys
  if (ev.code === "KeyE") { rotE = true; renderSwitches(); return; }
  const n = SW_DEFS.findIndex((d) => d.kbUp === ev.code);
  if (n >= 0) {
    swBits |= KEY_BITS[n].a; input.keys = swBits; renderSwitches(); renderInputHud(); dirty = true; maybeSend(); return;
  }
  const nd = SW_DEFS.findIndex((d) => d.kbDn === ev.code);
  if (nd >= 0) {
    swBits |= KEY_BITS[nd].b; input.keys = swBits; renderSwitches(); renderInputHud(); dirty = true; maybeSend();
  }
});
addEventListener("keyup", (ev) => {
  if (KEYDIR[ev.code]) heldKeys.delete(ev.code);
  if (ev.code === "KeyQ") { rotQ = false; renderSwitches(); return; }
  if (ev.code === "KeyE") { rotE = false; renderSwitches(); return; }
  const n = SW_DEFS.findIndex((d) => d.kbUp === ev.code);
  if (n >= 0) { swBits &= ~KEY_BITS[n].a; input.keys = swBits; renderSwitches(); renderInputHud(); dirty = true; maybeSend(); }
  const nd = SW_DEFS.findIndex((d) => d.kbDn === ev.code);
  if (nd >= 0) { swBits &= ~KEY_BITS[nd].b; input.keys = swBits; renderSwitches(); renderInputHud(); dirty = true; maybeSend(); }
});
addEventListener("blur", () => {           // 失焦=不可信输入，全松（车交看门狗兜底）
  padSuspended = true;                     // 手柄杆可能还压着：挂起，不许它失焦后继续出力
  padActive = false; padK6 = 0;
  heldKeys.clear();
  resetInputToZero();
});
addEventListener("focus", () => { padSuspended = false; });   // 回前台解挂，压着的杆下一拍重新出力

/* ==================== UI：挡位按钮 ==================== */

$("btn-mode").addEventListener("click", () => toggleMode("屏上按钮"));

/* ==================== UI：HUD 刷新 ==================== */

function renderInputHud() {
  $("val-wseq").textContent = writer ? String(input.wseq) : "–";
  $("val-keys").textContent = `0x${input.keys.toString(16).padStart(4, "0")}`;
  $("val-estop").textContent = input.estop ? "1" : "0";
  $("val-sent").textContent = String(sentCount);
}

function updateTxHud(f) {
  $("val-lastframe").textContent = [...f].map((b) => b.toString(16).padStart(2, "0")).join(" ");
  renderInputHud();
}

function renderTelemetry() {
  if (!rmt) return;
  $("tlm-tx").textContent = String(rmt.tx);
  $("tlm-fail").textContent = String(rmt.fail);
  $("tlm-seq").textContent = String(rmt.seq);
  $("tlm-v").textContent = `(${rmt.vx}, ${rmt.vy})`;
  $("tlm-o").textContent = String(rmt.o);
  $("tlm-k").textContent = `0x${rmt.k.toString(16).padStart(4, "0")}`;
  $("tlm-e").textContent = String(rmt.e);
  $("tlm-ctr").textContent = `${rmt.ok} / ${rmt.err} / ${rmt.dr}`;
}

/* 遥测行龄与灯：200ms 快刷 */
setInterval(() => {
  if (!serialOpen) return;
  const age = lastRmtMs ? performance.now() - lastRmtMs : Infinity;
  $("tlm-age").textContent = Number.isFinite(age) ? `${(age / 1000).toFixed(1)}s` : "–";
  const alive = age < 2000;
  setLight("light-web", alive && rmt && rmt.w === "u", alive && rmt && rmt.w === "d");
  setLight("light-rf", alive && rmt && rmt.tx > 0 && rmt.w === "u", alive && rmt && rmt.fail > 0 && rmt.tx === 0);
}, 200);

/* ==================== 启动 ==================== */

$("btn-connect").addEventListener("click", connect);
$("btn-disconnect").addEventListener("click", () => disconnect("手动断开"));
buildSwitches();
renderJoystick();
renderEstop();
renderMode();
renderInputHud();
logLine("WebPilot 就绪。协议 v2（12B 含 omega 轴）见 PROTOCOL.md——页面与 Remoter 板须同批升级，旧板收不了 v2 帧（看门狗安全停发）。", "line-evt");
logLine("未连接时所有控件可空按演练，不发包。", null);
logLine("挡位「前进·左右 / 前进×回转」：左杆下压(L3)或点挡位按钮切换，杆回中才受理；键盘 WASD+Q/E 不受挡位影响。", null);
logLine("蓝牙/USB 手柄：系统配对后在页面按一下任意手柄键接入——左杆=前后+挡位轴，右杆=定速率旋转（K_6 同款）。", null);

/* 调试出口：控制台可 packFrame 对账帧字节、handleLine 回放 [RMT] 行、读 input 真值 */
window.WebPilotDebug = { packFrame, crc8, input, handleLine };
