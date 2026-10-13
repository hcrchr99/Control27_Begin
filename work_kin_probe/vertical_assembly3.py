# -*- coding: utf-8 -*-
"""快速版：竖直装配轨迹（Σ=90−腕转轴倾角，roll=90°）
   爪心 x 只需沿一条竖直线扫；对每个 x 用解析化搜索 z。
"""
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
SIG = 90.0 - np.degrees(np.arctan2(RAX3[2], RAX3[0]))
ROLL = 90.0
LA = np.hypot(*UP); BEA = np.degrees(np.arctan2(UP[1], UP[0]))
LB = np.hypot(*(RJ+WR)); BEB = np.degrees(np.arctan2(*(RJ+WR)[::-1]))

def rot_y(t):
    c, s = np.cos(t*D2R), np.sin(t*D2R); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rod(a, t):
    t = t*D2R; K = np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
    return np.eye(3)+np.sin(t)*K+(1-np.cos(t))*(K@K)
def R2(t):
    c,s = np.cos(t*D2R), np.sin(t*D2R); return np.array([[c,-s],[s,c]])

# 预计算：给定 el3，腕+爪的等效连杆（roll=90 时的面内部分）
def eff(el3):
    a3 = el3*D2R; ca3, sa3 = np.cos(a3), np.sin(a3)
    wxx = WR[0] + TL[0]*ca3 - TL[1]*sa3
    wzz = WR[1] + TL[0]*sa3 + TL[1]*ca3
    wy  = WRY + TLY
    ax, az = RAX3[0], RAX3[2]
    cr, sr = 0.0, 1.0            # roll = 90
    adotw = ax*wxx + az*wzz
    tx = cr*wxx + sr*wy*(-az) + (1-cr)*adotw*ax
    tz = cr*wzz + sr*wy*( ax) + (1-cr)*adotw*az
    bx, bz = RJ[0]+tx, RJ[1]+tz
    return bx, bz, np.hypot(bx,bz), np.degrees(np.arctan2(bz,bx))

def fk(el1, el2, el3):
    bx,bz,lb,bb = eff(el3)
    x = SH[0] + LA*np.cos((BEA+el1)*D2R) + lb*np.cos((bb+el1+el2)*D2R)
    z = SH[1] + LA*np.sin((BEA+el1)*D2R) + lb*np.sin((bb+el1+el2)*D2R)
    return np.array([x,z])

def ik(tgt, e0):
    q = np.array(e0, float)
    for _ in range(200):
        e = tgt - fk(q[0], q[1], SIG-q[0]-q[1])
        if np.linalg.norm(e) < 1e-11: break
        J = np.zeros((2,2)); h = 1e-6
        for i in range(2):
            a=q.copy(); a[i]+=h; b=q.copy(); b[i]-=h
            J[:,i] = (fk(a[0],a[1],SIG-a.sum())-fk(b[0],b[1],SIG-b.sum()))/(2*h)
        try: dq = np.linalg.solve(J+1e-14*np.eye(2), e)
        except np.linalg.LinAlgError: return None
        q = q + 0.6*dq
        q[0]=np.clip(q[0],-LIM,LIM); q[1]=np.clip(q[1],-LIM,LIM)
    if np.linalg.norm(tgt-fk(q[0],q[1],SIG-q[0]-q[1])) > 1e-6: return None
    e3 = SIG-q[0]-q[1]
    if abs(e3) > LIM: return None
    return np.array([q[0],q[1],e3])

def servo(el): return 90.0-el

print("Σ = %.4f°   roll = %.0f°   （腕转轴倾角 %.4f°）" % (SIG, ROLL, 90-SIG))
print()

found = []
for x in np.arange(60, 320, 2.0):
    for z in np.arange(100, 200, 2.0):
        p = np.array([x, z], float)
        q = None
        for seed in ((0.,0.), (-20.,20.), (30.,-30.), (50.,-60.)):
            q = ik(p, seed)
            if q is not None: break
        if q is None: continue
        ok = True; qq = q[:2].copy(); tr = []
        for k in range(31):
            s = 70.0*k/30.0
            q2 = ik(p+np.array([0.,-s]), qq)
            if q2 is None: ok = False; break
            qq = q2[:2].copy(); tr.append((s, q2))
        if ok: found.append((p, q, tr))
    if len(found) >= 30: break

print("可行起点数 = %d" % len(found))
if not found:
    print("该高度带无解"); sys.exit(0)

found.sort(key=lambda t: (abs(t[0][0]-180.0), -t[0][1]))
for p, q, tr in found[:5]:
    print("  起点(%.0f,%.0f)  舵机 θ1=%.1f θ2=%.1f θ4=%.1f"
          % (p[0], p[1], servo(q[0]), servo(q[1]), servo(q[2])))

p, q, tr = found[0]
print()
print("=" * 100)
print("选定：爪心起点 (%.1f, %.1f) mm，竖直下压 70 mm" % (p[0], p[1]))
print("=" * 100)
print("   #   Δz(mm)    θ1(°)    θ2(°)    θ3(°)    θ4(°)    爪心 x(mm)   爪心 z(mm)")
for i,(s,qq) in enumerate(tr):
    pc = fk(qq[0],qq[1],qq[2])
    if i % 5 == 0 or i == len(tr)-1:
        print("  %2d  %7.2f  %7.3f  %7.3f  %7.1f  %7.3f   %9.4f   %9.4f"
              % (i, s, servo(qq[0]), servo(qq[1]), ROLL, servo(qq[2]), pc[0], pc[1]))

lat = max(abs(fk(t[1][0],t[1][1],t[1][2])[0]-p[0]) for t in tr)
print("\n  轨迹横向（x）偏差 ≤ %.6f mm" % lat)
print("  θ3 全程 %.1f°（锁死）    Σ 全程 %.4f°（恒定 ⇒ 爪板朝向不变）" % (ROLL, SIG))

print("\n  关节范围（舵机角）：")
for nm, idx in (("θ1 大臂",0),("θ2 小臂",1),("θ4 手腕",2)):
    arr = [servo(t[1][idx]) for t in tr]
    print("    %s  %.2f° ~ %.2f°   行程 %.2f°   %s"
          % (nm, min(arr), max(arr), max(arr)-min(arr),
             "OK" if min(arr)>=0 and max(arr)<=180 else "**越限**"))

json.dump(dict(sigma_deg=SIG, roll_deg=ROLL, start_xy=p.tolist(),
               rows=[dict(s_mm=float(s),
                          servo=[float(servo(qq[0])), float(servo(qq[1])), float(ROLL),
                                 float(servo(qq[2]))],
                          el=[float(qq[0]), float(qq[1]), float(qq[2])],
                          cc=fk(qq[0],qq[1],qq[2]).tolist()) for s, qq in tr]),
          io.open(B+r"\work_kin_probe\vertical_assembly_traj.json",'w',encoding='utf-8'),
          ensure_ascii=False, indent=1)
print("\n  写入 work_kin_probe/vertical_assembly_traj.json")
