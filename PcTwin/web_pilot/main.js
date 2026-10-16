/**
 * WebPilot 网页遥控台主逻辑（原生 ES Module，零构建）
 *
 * 线协议见同目录 PROTOCOL.md（与固件 Hardware/Remoter/UserApp/web_input.h 对表）：
 * 下行 10B 定长帧（0x5A + wseq + RcInput_Raw_t 7B 镜像 + CRC8），20Hz 心跳 +
 * 变化即发（≥15ms 间隔）；上行 [RMT] 文本统计行。
 *
 * 安全行为（拍板 2026-10-16）：
 *  - 断连（拔线/浏览器崩/关标签）：停发帧 → 板 500ms 看门狗停发 RF → 车渐停+告警；
 *  - 急停 = 点击闩锁，再点解除（与物理轻触急停的"按住置位"在帧字段同构）；
 *  - 后台标签页被节流 → 心跳掉线 → 车自动渐停：特性非故障，横幅提示。
 */

"use strict";

/* ==================== 线协议（与 PROTOCOL.md / web_input.h 逐字节对表） ==================== */

const HEADER = 0x5A;
const FRAME_LEN = 10;
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

/** 打一帧 10B：小端，字段布局见 PROTOCOL.md §2（vx/vy=轴逻辑量 ±1000，i16） */
function packFrame(wseq, vx, vy, keys, estop) {
  const f = new Uint8Array(FRAME_LEN);
  f[0] = HEADER;
  f[1] = wseq & 0xFF;
  const dv = new DataView(f.buffer);
  dv.setInt16(2, clampAxis(vx), true);    // 小端：低字节在前，有符号
  dv.setInt16(4, clampAxis(vy), true);
  dv.setUint16(6, keys & 0xFFFF, true);
  f[8] = estop ? 1 : 0;
  f[9] = crc8(f.subarray(1, 9));          // CRC 覆盖 [1..8]
  return f;
}

/* 轴逻辑量钳位 ±1000（发送端纪律，rc_cmd.h v2：vx/vy 由发送端钳位） */
function clampAxis(v) { return Math.max(-1000, Math.min(1000, Math.round(v))); }

/* keys 位图（与 rc_input.h 宏组一致）：每开关 2 位 A/B */
const KEY_BITS = [];
for (let n = 0; n < 6; n++) KEY_BITS.push({ a: 1 << (2 * n), b: 1 << (2 * n + 1) });

/* ==================== 输入状态（唯一真值，打包函数只读这里） ==================== */

const input = {
  vx: 0,               // 轴逻辑量 ±1000（0=回中），右/前为正
  vy: 0,
  keys: 0,             // 方向刀位图
  estop: false,        // 急停闩锁
  wseq: 0,
};

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
  const f = packFrame(input.wseq, input.vx, input.vy, input.keys, input.estop);
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
      const f = packFrame(input.wseq, 0, 0, 0, input.estop);
      try { writer.write(f); input.wseq = (input.wseq + 1) & 0xFF; } catch (e) { /* 尽力而为 */ }
    }
    setBanner("已切到后台：心跳已停发，车端将自动渐停断链。回到前台自动恢复。");
  }
});

/* 页面卸载尽力发一帧全零（礼貌收尾，非安全依赖——真兜底是板上看门狗） */
window.addEventListener("pagehide", () => {
  if (!writer) return;
  const f = packFrame(input.wseq, 0, 0, 0, 0);
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
  input.vx = 0; input.vy = 0;
  input.keys = 0; input.estop = false;
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

function renderJoystick() {
  const nx = -input.vy / 1000;             // 屏幕横向=vy（左推为正，2026-10-16 修正）
  const ny = input.vx / 1000;              // 屏幕纵向=vx（前进为正）
  const px = 120 + nx * JOY_R;
  const py = 120 - ny * JOY_R;             // 屏幕 y 向下，取反
  $("joy-thumb").style.left = `${px}px`;
  $("joy-thumb").style.top = `${py}px`;
  $("val-vx").textContent = input.vx;
  $("val-vy").textContent = input.vy;
}

function setJoyFromPoint(ev) {
  const r = $("joy-pad").getBoundingClientRect();
  let nx = (ev.clientX - r.left - 120) / JOY_R;    // -1..+1
  let ny = -(ev.clientY - r.top - 120) / JOY_R;
  const m = Math.hypot(nx, ny);
  if (m > 1) { nx /= m; ny /= m; }                 // 方形 pad 圆形行程
  input.vx = Math.round(ny * 1000);                // 屏上推=前进=vx+（车端纵向习惯）
  input.vy = Math.round(-nx * 1000);               // 屏左推=vy+（2026-10-16 实车方向二次修正）
  renderJoystick(); renderInputHud();
  dirty = true; maybeSend();
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
  input.vx = 0; input.vy = 0;              // 松手弹回中位（自复位语义）
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
 * 轴习惯与车端一致：vx=纵向（W 前进/S 后退），vy=横移（A 左=+ / D 右=−，
 * 2026-10-16 实车方向二次修正） */
const heldKeys = new Set();
const KEYDIR = { KeyW: [1, 0], KeyS: [-1, 0], KeyA: [0, 1], KeyD: [0, -1] };

function keyStickTick() {
  let tx = 0, ty = 0;
  for (const c of heldKeys) { if (KEYDIR[c]) { tx += KEYDIR[c][0]; ty += KEYDIR[c][1]; } }
  const m = Math.hypot(tx, ty);
  if (m > 1) { tx /= m; ty /= m; }
  if (!joyDragging) {
    const nx = input.vx / 1000, ny = input.vy / 1000;
    const k = 0.35;                         // 每帧向目标靠拢的比例（手感：~100ms 满行程）
    let gx = nx + (tx - nx) * k, gy = ny + (ty - ny) * k;
    if (tx === 0 && Math.abs(gx) < 0.01) gx = 0;    // 松键回中收尾，不挂尾数
    if (ty === 0 && Math.abs(gy) < 0.01) gy = 0;
    input.vx = Math.round(gx * 1000);
    input.vy = Math.round(gy * 1000);
    renderJoystick(); renderInputHud();
    if (gx !== nx || gy !== ny) {
      dirty = true; maybeSend();
    }
  }
  requestAnimationFrame(keyStickTick);
}
requestAnimationFrame(keyStickTick);

/* ==================== UI：三态船型开关 ==================== */

/* 下标=开关号 n：K_1..K_5 关节，K_6=整车旋转 omega（2026-10-16 拍板）。
 * Q=左转（+omega）/ E=右转（−omega），车端几何定实际转向 */
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
      const bit = KEY_BITS[n];
      if (which === "up") up.classList.toggle("press-up", on);
      else dn.classList.toggle("press-dn", on);
      if (on) input.keys |= (which === "up" ? bit.a : bit.b);
      else input.keys &= ~(which === "up" ? bit.a : bit.b);
      renderInputHud();
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
  const n = SW_DEFS.findIndex((d) => d.kbUp === ev.code);
  if (n >= 0) {
    input.keys |= KEY_BITS[n].a; renderSwitches(); renderInputHud(); dirty = true; maybeSend(); return;
  }
  const nd = SW_DEFS.findIndex((d) => d.kbDn === ev.code);
  if (nd >= 0) {
    input.keys |= KEY_BITS[nd].b; renderSwitches(); renderInputHud(); dirty = true; maybeSend();
  }
});
addEventListener("keyup", (ev) => {
  if (KEYDIR[ev.code]) heldKeys.delete(ev.code);
  const n = SW_DEFS.findIndex((d) => d.kbUp === ev.code);
  if (n >= 0) { input.keys &= ~KEY_BITS[n].a; renderSwitches(); renderInputHud(); dirty = true; maybeSend(); }
  const nd = SW_DEFS.findIndex((d) => d.kbDn === ev.code);
  if (nd >= 0) { input.keys &= ~KEY_BITS[nd].b; renderSwitches(); renderInputHud(); dirty = true; maybeSend(); }
});
addEventListener("blur", () => {           // 失焦=不可信输入，全松（车交看门狗兜底）
  heldKeys.clear();
  resetInputToZero();
});

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
renderInputHud();
logLine("WebPilot 就绪。协议见 PROTOCOL.md；连接前先插好 USB-TTL（TX/RX 交叉、共地、3.3V）。", "line-evt");
logLine("未连接时所有控件可空按演练，不发包。", null);

/* 调试出口：控制台可 packFrame 对账帧字节、handleLine 回放 [RMT] 行、读 input 真值 */
window.WebPilotDebug = { packFrame, crc8, input, handleLine };
