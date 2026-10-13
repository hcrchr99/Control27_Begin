# -*- coding: utf-8 -*-
"""权威版 ③：用正确的 Σ = +29.3015° 生成竖直下压 70mm 轨迹并全面核验"""
import numpy as np, sys, json, io
try: sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception: pass
np.set_printoptions(precision=5, suppress=True)
B = r"C:\Users\Admin\WorkBuddy\RM_Begin"

SH = np.array([48.900, 46.198]); UP = np.array([111.100, 63.400])
RJ = np.array([58.354, 26.496]); RJY = 46.593
WR = np.array([23.723, 21.742]); WRY = -46.593
TL = np.array([70.6734, -47.8161]); TLY = 56.550
RAX3 = np.array([0.872057, 0.0, 0.489405]); RAX3 /= np.linalg.norm(RAX3)
Y3 = np.array([0.,1.,0.]); Z3 = np.array([0.,0.,1.])
D2R, R2D = np.pi/180.0, 180.0/np.pi
LIM = 90.0
ROLL = 90.0
SIG = 90.0 - np.degrees(np.arctan2(RAX3[2], RAX3[0]))     # = 29.3015
LA = np.hypot(*UP); BEA = np.degrees(np.arctan2(UP[1], UP[0]))

def xz_rot(t):   # 面内 2D 旋转（仰角域逆时针）
    c, s = np.cos(t*D2R), np.sin(t*D2R)
    return np.array([[c,-s],[s,c]])
def Ry3(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c,0.,s],[0.,1.,0.],[-s,0.,c]])
def rod(a, ang):
    t = ang*D2R
    K = np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
    return np.eye(3)*np.cos(t) + (1-np.cos(t))*np.outer(a,a) + np.sin(t)*K

def eff(el3):
    """roll=90 时 腕+爪 的等效连杆（面内）"""
    a3 = el3*D2R; ca3, sa3 = np.cos(a3), np.sin(a3)
    wxx = WR[0] + TL[0]*ca3 - TL[1]*sa3
    wzz = WR[1] + TL[0]*sa3 + TL[1]*ca3
    wy  = WRY + TLY
    ax, az = RAX3[0], RAX3[2]
    tx = wxx*0 + wy*(-az)                    # cr=0, sr=1
    tz = wzz*0 + wy*( ax)
    bx, bz = RJ[0]+tx, RJ[1]+tz
    return np.hypot(bx,bz), np.degrees(np.arctan2(bz,bx))

def clawpos(el1, el2, el3):
    lb, bb = eff(el3)
    x = SH[0] + LA*np.cos((BEA+el1)*D2R) + lb*np.cos((bb+el1+el2)*D2R)
    z = SH[1] + LA*np.sin((BEA+el1)*D2R) + lb*np.sin((bb+el1+el2)*D2R)
    return np.array([x,z])

def opening(el1, el2, el3):
    s = el1+el2+el3
    return rod(Ry3(s*D2R) @ RAX3, ROLL) @ Y3, Ry3(s*D2R) @ RAX3

def ik(tgt, e0):
    q = np.array(e0, float)
    for _ in range(250):
        e = tgt - clawpos(q[0], q[1], SIG-q[0]-q[1])
        if np.linalg.norm(e) < 1e-11: break
        J = np.zeros((2,2)); h = 1e-6
        for i in range(2):
            a=q.copy(); a[i]+=h; b=q.copy(); b[i]-=h
            J[:,i]=(clawpos(a[0],a[1],SIG-a.sum())-clawpos(b[0],b[1],SIG-b.sum()))/(2*h)
        try: dq = np.linalg.solve(J+1e-14*np.eye(2), e)
        except np.linalg.LinAlgError: return None
        q = q + 0.6*dq
        q[0]=np.clip(q[0],-LIM,LIM); q[1]=np.clip(q[1],-LIM,LIM)
    if np.linalg.norm(tgt-clawpos(q[0],q[1],SIG-q[0]-q[1])) > 1e-6: return None
    e3 = SIG-q[0]-q[1]
    if abs(e3) > LIM: return None
    return np.array([q[0],q[1],e3])

def sv(el): return 90.0-el

print("Σ = %.5f°    θ3(roll) = %.0f°" % (SIG, ROLL))
o, a = opening(0,0,SIG)
print("校验：开合方向 = %s  距竖直 %.6f°" % (np.round(o,6), np.degrees(np.arccos(np.clip(abs(o@Z3),-1,1)))))
print()

# 在 z∈[100,200] 找可行起点
found = []
for x in np.arange(60, 320, 2.0):
    for z in np.arange(100, 200, 2.0):
        p = np.array([x,z], float)
        q = None
        for sd in ((0.,0.),(30.,-30.),(-30.,30.),(60.,-60.)):
            q = ik(p, sd)
            if q is not None: break
        if q is None: continue
        ok=True; qq=q[:2].copy(); tr=[]
        for k in range(31):
            s = 70.0*k/30.0
            q2 = ik(p+np.array([0.,-s]), qq)
            if q2 is None: ok=False; break
            qq=q2[:2].copy(); tr.append((s,q2))
        if ok: found.append((p,q,tr))

print("可行起点数 = %d" % len(found))
found.sort(key=lambda t:(abs(t[0][0]-220.0), -t[0][1]))
p, q, tr = found[0]
print("选定起点：爪心 (%.1f, %.1f) mm   舵机 θ=(%.2f, %.2f, %.1f, %.2f)"
      % (p[0], p[1], sv(q[0]), sv(q[1]), ROLL, sv(q[2])))
print()
print("   #   Δz(mm)    θ1(°)    θ2(°)   θ3(°)    θ4(°)   爪心(x,z) mm         轴线偏竖(°)  法向仰角(°)")
lat=0; vang=[]
for i,(s,qq) in enumerate(tr):
    pc = clawpos(*qq); oo,aa = opening(*qq)
    ang = np.degrees(np.arccos(np.clip(abs(oo@Z3),-1,1))); vang.append(ang)
    nrm = np.degrees(np.arctan2(np.sin(sum(qq)*D2R), np.cos(sum(qq)*D2R)))
    lat = max(lat, abs(pc[0]-p[0]))
    if i%5==0 or i==len(tr)-1:
        print("  %2d  %7.2f  %7.2f  %7.2f  %6.1f  %7.2f   (%8.4f,%8.4f)   %9.5f   %+8.3f"
              % (i, s, sv(qq[0]), sv(qq[1]), ROLL, sv(qq[2]), pc[0], pc[1], ang, nrm))
print()
print("  横向(x)偏差 ≤ %.6f mm        轴线偏离竖直 ≤ %.6f°" % (lat, max(vang)))
print("  Σ 全程恒定 = %.5f°  ⇒  爪板法向恒定 ⇒ 爪板朝向严格不变" % SIG)
print()
print("  关节范围（舵机角，限位 [0,180]）：")
for nm, j in (("θ1 大臂",0),("θ2 小臂",1),("θ3 爪旋转",2),("θ4 手腕",3)):
    arr=[sv(t[1][j]) if j!=2 else ROLL for t in tr]
    print("    %-9s %7.2f° ~ %7.2f°   行程 %6.2f°   %s"
          % (nm, min(arr), max(arr), max(arr)-min(arr),
             "OK" if min(arr)>=0 and max(arr)<=180 else "**越限**"))
print()
print("  从零位到起点需要动：")
print("    θ1 90 → %.1f (Δ%+.1f)   θ2 90 → %.1f (Δ%+.1f)   θ3 0→90 (Δ+90)   θ4 90 → %.1f (Δ%+.1f)"
      % (sv(q[0]), sv(q[0])-90, sv(q[1]), sv(q[1])-90, sv(q[2]), sv(q[2])-90))

json.dump(dict(sigma_deg=float(SIG), roll_deg=float(ROLL), start_xy=p.tolist(),
               rows=[dict(s_mm=float(s),
                          servo=[float(sv(qq[0])), float(sv(qq[1])), float(ROLL), float(sv(qq[2]))],
                          cc=clawpos(*qq).tolist(),
                          opening=opening(*qq)[0].tolist()) for s,qq in tr]),
          io.open(B+r"\work_kin_probe\vertical_assembly_traj.json",'w',encoding='utf-8'),
          ensure_ascii=False, indent=1)
print("\n  写入 work_kin_probe/vertical_assembly_traj.json")
