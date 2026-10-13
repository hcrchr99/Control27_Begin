# -*- coding: utf-8 -*-
"""竖直装配方案：Σ = −150.7°（腕转轴水平） + roll = 90° → 矿石轴线严格竖直
   再求竖直下压的完整 4 关节轨迹，并核验限位与可达性。
"""
import numpy as np, sys
try: sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception: pass
np.set_printoptions(precision=4, suppress=True)

# ---- robot_config.h 常量 ----
SH = np.array([48.900, 46.198])
UP = np.array([111.100, 63.400])
RJ = np.array([58.354, 26.496]); RJY = 46.593
WR = np.array([23.723, 21.742]); WRY = -46.593
TL = np.array([70.6734, -47.8161]); TLY = 56.550
RAX3 = np.array([0.872057, 0.0, 0.489405]); RAX3 /= np.linalg.norm(RAX3)
Y3 = np.array([0.0, 1.0, 0.0]); Z3 = np.array([0.0, 0.0, 1.0])
D2R, R2D = np.pi/180.0, 180.0/np.pi
LIM = 90.0

def rot_y(t):
    c, s = np.cos(t*D2R), np.sin(t*D2R)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rod(a, t):
    t = t*D2R
    K = np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
    return np.eye(3) + np.sin(t)*K + (1-np.cos(t))*(K@K)
def R2(t):
    c, s = np.cos(t*D2R), np.sin(t*D2R)
    return np.array([[c,-s],[s,c]])

def el2servo(el): return 90.0 - el
def servo2el(s):  return 90.0 - s

def fk(el1, el2, el3, roll):
    """返回 爪心(x,z), 爪心y, 开合方向(3), 爪板法向(2), 腕俯仰轴位(2), 腕转轴(3)"""
    sig = el1 + el2 + el3
    a3 = el3*D2R
    ca3, sa3 = np.cos(a3), np.sin(a3)
    wxx = WR[0] + TL[0]*ca3 - TL[1]*sa3
    wzz = WR[1] + TL[0]*sa3 + TL[1]*ca3
    wy  = WRY + TLY
    ax, az = RAX3[0], RAX3[2]
    adotw = ax*wxx + az*wzz
    cr, sr = np.cos(roll*D2R), np.sin(roll*D2R)
    tx = cr*wxx + sr*wy*(-az) + (1-cr)*adotw*ax
    tz = cr*wzz + sr*wy*( ax) + (1-cr)*adotw*az
    bx, bz = RJ[0]+tx, RJ[1]+tz
    tip_y = RJY + wy*cr + (az*wxx - ax*wzz)*sr
    lb = np.hypot(bx, bz); bb = np.arctan2(bz, bx)
    la = np.hypot(*UP);    be = np.arctan2(UP[1], UP[0])
    a1 = be + el1*D2R
    a2 = bb + (el1+el2)*D2R
    x = SH[0] + la*np.cos(a1) + lb*np.cos(a2)
    z = SH[1] + la*np.sin(a1) + lb*np.sin(a2)
    nrm = np.array([np.cos(sig*D2R), np.sin(sig*D2R)])
    axw = rot_y(sig) @ RAX3
    ov = rod(axw, roll) @ Y3
    wp = SH + UP + RJ + WR
    return (np.array([x, z]), tip_y, ov, nrm, wp, axw)

# ---- 求 Σ 使腕转轴水平 ----
best = None
for sig in np.arange(-180, 180.001, 0.001):
    a = rot_y(sig) @ RAX3
    d = abs(np.degrees(np.arccos(np.clip(abs(a @ Z3), -1, 1))) - 90.0)
    if best is None or d < best[0]: best = (d, sig)
SIG = best[1]
print("=" * 96)
print("竖直装配解")
print("=" * 96)
print("  Σ = el1+el2+el3 = %.3f°   ⇒ 腕转轴水平" % SIG)
print("  roll(θ3) = 90.0°          ⇒ 爪开合方向 = 竖直")
print()
_, _, ov, _, _, axw = fk(0, 0, SIG, 90)
print("  核验：腕转轴 = (%.5f, %.5f, %.5f)   与竖直夹角 %.4f°"
      % (*axw, np.degrees(np.arccos(np.clip(abs(axw@Z3), -1, 1)))))
print("        开合方向 = (%.5f, %.5f, %.5f)  与竖直夹角 %.6f°"
      % (*ov, np.degrees(np.arccos(np.clip(abs(ov@Z3), -1, 1)))))
print()

# ---- 数值逆解：固定 Σ、roll=90，求 el1,el2 使爪心到目标 ----
def ik(tgt, el1_0=0.0, el2_0=0.0, iters=300):
    q = np.array([el1_0, el2_0], float)
    for _ in range(iters):
        p, _, _, _, _, _ = fk(q[0], q[1], SIG-q[0]-q[1], 90)
        e = tgt - p
        if np.linalg.norm(e) < 1e-12: break
        J = np.zeros((2,2)); h = 1e-6
        for i in range(2):
            a = q.copy(); a[i] += h
            b = q.copy(); b[i] -= h
            J[:,i] = (fk(a[0],a[1],SIG-a.sum(),90)[0] - fk(b[0],b[1],SIG-b.sum(),90)[0])/(2*h)
        try: dq = np.linalg.solve(J + 1e-14*np.eye(2), e)
        except np.linalg.LinAlgError: break
        q = q + 0.6*dq
        q[0] = np.clip(q[0], -LIM, LIM); q[1] = np.clip(q[1], -LIM, LIM)
    p, _, _, _, _, _ = fk(q[0], q[1], SIG-q[0]-q[1], 90)
    if np.linalg.norm(p - tgt) > 1e-6: return None
    el3 = SIG - q[0] - q[1]
    if abs(el3) > LIM: return None
    return np.array([q[0], q[1], el3])

# ---- 找一个能竖直走 70mm 的起点 ----
print("=" * 96)
print("搜索：能竖直下压 70mm 的起点（要求全程 el 都在 ±90° 内）")
print("=" * 96)
found = []
for x0 in np.arange(60, 340, 2.0):
    for z0 in np.arange(60, 300, 2.0):
        p0 = np.array([x0, z0], float)
        q = ik(p0, 0.0, 0.0)
        if q is None:
            q = ik(p0, -40.0, 20.0)
        if q is None: continue
        # 试走 70mm 向下
        ok = True; qq = q[:2].copy(); traj = []
        for k in range(31):
            s = 70.0*k/30.0
            q2 = ik(p0 + np.array([0, -s]), qq[0], qq[1])
            if q2 is None: ok = False; break
            qq = q2[:2].copy()
            traj.append((s, q2))
        if ok:
            found.append((p0, q, traj))
    if len(found) >= 8: break

print("  可行起点数（首轮扫描）= %d" % len(found))
for p0, q, traj in found[:6]:
    el1s = [t[1][0] for t in traj]; el2s = [t[1][1] for t in traj]; el3s = [t[1][2] for t in traj]
    print("   爪心起点 (%.1f, %.1f)  el=(%+.2f,%+.2f,%+.2f)  舵机=(%.1f,%.1f,%.1f)"
          % (p0[0], p0[1], *q, el2servo(q[0]), el2servo(q[1]), el2servo(q[2])))
    print("        全程 el1 %.2f~%.2f  el2 %.2f~%.2f  el3 %.2f~%.2f"
          % (min(el1s),max(el1s),min(el2s),max(el2s),min(el3s),max(el3s)))
    print("        全程 舵机角: θ1 %.1f~%.1f  θ2 %.1f~%.1f  θ4 %.1f~%.1f"
          % (el2servo(max(el1s)), el2servo(min(el1s)),
             el2servo(max(el2s)), el2servo(min(el2s)),
             el2servo(max(el3s)), el2servo(min(el3s))))
