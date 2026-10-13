# -*- coding: utf-8 -*-
"""在矿物作业高度带 z∈[100,200] 内找竖直下压 70mm 的可行起点，输出完整轨迹表 + 核验"""
import numpy as np, sys, json, io
try: sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception: pass
np.set_printoptions(precision=4, suppress=True)
B = r"C:\Users\Admin\WorkBuddy\RM_Begin"

SH = np.array([48.900, 46.198]); UP = np.array([111.100, 63.400])
RJ = np.array([58.354, 26.496]); RJY = 46.593
WR = np.array([23.723, 21.742]); WRY = -46.593
TL = np.array([70.6734, -47.8161]); TLY = 56.550
RAX3 = np.array([0.872057, 0.0, 0.489405]); RAX3 /= np.linalg.norm(RAX3)
Y3 = np.array([0.,1.,0.]); Z3 = np.array([0.,0.,1.])
D2R, R2D = np.pi/180.0, 180.0/np.pi
LIM = 90.0
RAX_TILT = np.degrees(np.arctan2(RAX3[2], RAX3[0]))
SIG = 90.0 - RAX_TILT          # 使腕转轴水平
ROLL = 90.0                    # 使开合方向竖直

def rot_y(t):
    c, s = np.cos(t*D2R), np.sin(t*D2R); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rod(a, t):
    t = t*D2R; K = np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
    return np.eye(3) + np.sin(t)*K + (1-np.cos(t))*(K@K)

def fk(el1, el2, el3, roll=ROLL):
    sig = el1+el2+el3
    a3 = el3*D2R; ca3, sa3 = np.cos(a3), np.sin(a3)
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
    lb = np.hypot(bx,bz); bb = np.arctan2(bz,bx)
    la = np.hypot(*UP);   be = np.arctan2(UP[1], UP[0])
    x = SH[0] + la*np.cos(be+el1*D2R) + lb*np.cos(bb+(el1+el2)*D2R)
    z = SH[1] + la*np.sin(be+el1*D2R) + lb*np.sin(bb+(el1+el2)*D2R)
    nrm = np.array([np.cos(sig*D2R), np.sin(sig*D2R)])
    return np.array([x,z]), tip_y, rod(rot_y(sig)@RAX3, roll)@Y3, nrm, rot_y(sig)@RAX3

def ik(tgt, e0=(0.,0.), iters=400):
    q = np.array(e0, float)
    for _ in range(iters):
        p = fk(q[0], q[1], SIG-q[0]-q[1])[0]
        e = tgt - p
        if np.linalg.norm(e) < 1e-12: break
        J = np.zeros((2,2)); h = 1e-6
        for i in range(2):
            a=q.copy(); a[i]+=h; b=q.copy(); b[i]-=h
            J[:,i] = (fk(a[0],a[1],SIG-a.sum())[0]-fk(b[0],b[1],SIG-b.sum())[0])/(2*h)
        try: dq = np.linalg.solve(J+1e-14*np.eye(2), e)
        except np.linalg.LinAlgError: return None
        q = q + 0.6*dq
        q[0]=np.clip(q[0],-LIM,LIM); q[1]=np.clip(q[1],-LIM,LIM)
    p = fk(q[0],q[1],SIG-q[0]-q[1])[0]
    if np.linalg.norm(p-tgt) > 1e-6: return None
    e3 = SIG-q[0]-q[1]
    if abs(e3) > LIM: return None
    return np.array([q[0],q[1],e3])

def servo(el): return 90.0-el

print("=" * 100)
print("竖直装配解（用矿物作业高度带 z ∈ [100, 200] mm）")
print("=" * 100)
print("  Σ = el1+el2+el3 = %.3f°   (舵机 90−el ⇒ 三关节基准角 %.1f°)" % (SIG, servo(SIG)))
print("  θ3 (roll) = %.1f°" % ROLL)
x0, z0 = 0., 0.
p0, ty, ov, nrm, axw = fk(0, 0, SIG)
print("  零位偏置校验: 开合方向 = (%.5f,%.5f,%.5f) 距竖直 %.5f°"
      % (*ov, np.degrees(np.arccos(np.clip(abs(ov@Z3),-1,1)))))
print()

# 搜索起点：优先 z 起点在 170~200（下压 70 后落在 100~130）
sol = []
for x in np.arange(60, 340, 1.0):
    for z in np.arange(100, 200, 1.0):
        p = np.array([x, z], float)
        q = ik(p, (0., 0.)) or ik(p, (-20., 20.)) or ik(p, (30., -30.))
        if q is None: continue
        # 全程 70mm 向下
        ok = True; qq = q[:2].copy(); tr = []
        for k in range(36):
            s = 70.0*k/35.0
            q2 = ik(p + np.array([0.0, -s]), qq)
            if q2 is None: ok = False; break
            qq = q2[:2].copy(); tr.append((s, q2))
        if ok: sol.append((p, q, tr))

print("  可行起点数 = %d" % len(sol))
if not sol:
    print("  ** 该高度带内无解 **"); sys.exit(0)

# 选最"正"的一个（x 靠近中位，z 起点最高）
sol.sort(key=lambda t: (abs(t[0][0]-200.0), -t[0][1]))
p, q, tr = sol[0]
print("  选定起点: 爪心 = (%.1f, %.1f) mm" % (p[0], p[1]))
print()
print("   #   Δz(mm)   θ1(°)   θ2(°)   θ3(°)   θ4(°)   爪心(x,z) mm      Σ(°)    轴线偏竖(°)")
mx_lat = 0; mx_ang = 0
for i, (s, qq) in enumerate(tr):
    pc, tyy, ovv, nn, aa = fk(qq[0], qq[1], qq[2])
    ang = np.degrees(np.arccos(np.clip(abs(ovv@Z3),-1,1)))
    lat = abs(pc[0]-p[0])
    mx_lat = max(mx_lat, lat); mx_ang = max(mx_ang, ang)
    if i % 5 == 0 or i == len(tr)-1:
        print("  %2d  %7.2f  %7.3f %7.3f %7.1f %7.3f   (%7.3f,%7.3f)  %+8.3f   %.5f"
              % (i, s, servo(qq[0]), servo(qq[1]), ROLL, servo(qq[2]), pc[0], pc[1],
                 qq.sum(), ang))
print()
print("  横向偏差 ≤ %.6f mm      轴线偏离竖直 ≤ %.6f°" % (mx_lat, mx_ang))
e1 = [t[1][0] for t in tr]; e2 = [t[1][1] for t in tr]; e3 = [t[1][2] for t in tr]
print()
print("  关节范围（舵机角，限位 [0,180]）：")
for nm, arr in (("θ1 大臂", e1), ("θ2 小臂", e2), ("θ4 手腕", e3)):
    lo, hi = servo(max(arr)), servo(min(arr))
    print("    %s  %.2f° ~ %.2f°   行程 %.2f°   %s"
          % (nm, lo, hi, hi-lo, "OK" if lo>=0 and hi<=180 else "**越限**"))

# 导出
out = dict(sigma_deg=SIG, roll_deg=ROLL, start_xy=[p[0], p[1]],
           rows=[dict(s_mm=float(s), servo=[float(servo(t[1][0])), float(servo(t[1][1])),
                                            ROLL, float(servo(t[1][2]))],
                      el=[float(t[1][0]), float(t[1][1]), float(t[1][2])],
                      cc=fk(t[1][0],t[1][1],t[1][2])[0].tolist()) for s, t in tr])
json.dump(out, io.open(B+r"\work_kin_probe\vertical_assembly_traj.json", 'w', encoding='utf-8'),
          ensure_ascii=False, indent=1)
print("\n  轨迹已写入 work_kin_probe/vertical_assembly_traj.json")
