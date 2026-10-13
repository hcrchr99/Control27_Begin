# -*- coding: utf-8 -*-
"""权威版（已修正 Ry 符号）：爪开合方向 = 矿石轴线，求竖直解 + 轨迹"""
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
LIM = 90.0; ROLL = 90.0
LA = np.hypot(*UP); BEA = np.degrees(np.arctan2(UP[1], UP[0]))

def Ry3(t):                       # ★ 修正：绕 y 轴的标准右手旋转
    c, s = np.cos(t), np.sin(t)
    return np.array([[ c, 0., s],
                     [ 0.,1., 0.],
                     [-s, 0., c]])
def rod(a, ang):
    t = ang*D2R
    K = np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
    return np.eye(3)*np.cos(t) + (1-np.cos(t))*np.outer(a,a) + np.sin(t)*K

def orientation(el1, el2, el3, roll_deg):
    s = el1+el2+el3
    axis = Ry3(s*D2R) @ RAX3
    return axis, rod(axis, roll_deg) @ Y3, np.array([np.cos(s*D2R), np.sin(s*D2R)])

def eff(el3):
    a3 = el3*D2R; ca3, sa3 = np.cos(a3), np.sin(a3)
    wxx = WR[0] + TL[0]*ca3 - TL[1]*sa3
    wzz = WR[1] + TL[0]*sa3 + TL[1]*ca3
    wy  = WRY + TLY
    ax, az = RAX3[0], RAX3[2]
    tx = wxx*0.0 + wy*(-az)                    # cr=0, sr=1  (roll=90)
    tz = wzz*0.0 + wy*( ax)
    bx, bz = RJ[0]+tx, RJ[1]+tz
    return np.hypot(bx,bz), np.degrees(np.arctan2(bz,bx))

def clawpos(el1, el2, el3):
    lb, bb = eff(el3)
    x = SH[0] + LA*np.cos((BEA+el1)*D2R) + lb*np.cos((bb+el1+el2)*D2R)
    z = SH[1] + LA*np.sin((BEA+el1)*D2R) + lb*np.sin((bb+el1+el2)*D2R)
    return np.array([x,z])

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
        q = q+0.6*dq
        q[0]=np.clip(q[0],-LIM,LIM); q[1]=np.clip(q[1],-LIM,LIM)
    if np.linalg.norm(tgt-clawpos(q[0],q[1],SIG-q[0]-q[1])) > 1e-6: return None
    e3 = SIG-q[0]-q[1]
    if abs(e3) > LIM: return None
    return np.array([q[0],q[1],e3])

def sv(el): return 90.0-el
def av(v): return np.degrees(np.arccos(np.clip(abs(v@Z3),-1,1)))

print("=" * 100)
print("① 零位自检")
print("=" * 100)
ax_, op_, n_ = orientation(0,0,0,0)
print("  腕转轴 = %s  与竖直 %.4f°  与水平 %.4f°" % (np.round(ax_,5), av(ax_), 90-av(ax_)))
print("  开合方向(roll=0) = %s  与竖直 %.4f°（纯横向，符合）" % (np.round(op_,5), av(op_)))
print()

print("=" * 100)
print("② 扫 Σ（roll=90°）求开合方向竖直")
print("=" * 100)
print("   Σ(°)     腕转轴/竖直(°)   开合方向(x,y,z)               开合方向/竖直(°)")
best=None
for s in np.arange(-180,180.001,0.0005):
    a,o,nn = orientation(s,0,0,90.0)
    d = av(o)
    if best is None or d<best[0]: best=(d,s,o.copy(),a.copy())
for s in (-90,-60,-30,0,29.3015,60,90,120,150.7):
    a,o,nn = orientation(s,0,0,90.0)
    print("  %+8.3f    %10.4f    (%+.5f,%+.5f,%+.5f)    %10.5f" % (s,av(a),*o,av(o)))
print()
SIG = best[1]
print("  ★ 最优 Σ = %.4f°" % SIG)
print("    腕转轴 = %s   与竖直 %.5f°  ← 水平" % (np.round(best[3],5), av(best[3])))
print("    开合方向 = %s   与竖直 %.6f°" % (np.round(best[2],6), best[0]))
print("    （对应舵机角：三俯仰之和 = 270 − %.4f = %.4f°）" % (SIG, 270-SIG))
print()

found=[]
for x in np.arange(40, 360, 2.0):
    for z in np.arange(60, 260, 2.0):
        p=np.array([x,z],float)
        q=None
        for sd in ((0.,0.),(30.,-30.),(-30.,30.),(60.,-60.),(-60.,60.)):
            q=ik(p,sd)
            if q is not None: break
        if q is None: continue
        ok=True; qq=q[:2].copy(); tr=[]
        for k in range(31):
            s=70.0*k/30.0
            q2=ik(p+np.array([0.,-s]),qq)
            if q2 is None: ok=False; break
            qq=q2[:2].copy(); tr.append((s,q2))
        if ok: found.append((p,q,tr))

print("=" * 100)
print("③ 竖直下压 70mm 轨迹（可行起点 %d 个）" % len(found))
print("=" * 100)
if not found:
    print("  无解（该 Σ 下 70mm 竖直行程不可达）"); sys.exit(0)
found.sort(key=lambda t:(abs(t[0][0]-220.0), -t[0][1]))
p,q,tr = found[0]
print("  选定起点：爪心 (%.1f, %.1f) mm   舵机 θ=(%.2f, %.2f, %.1f, %.2f)"
      % (p[0],p[1], sv(q[0]),sv(q[1]),ROLL,sv(q[2])))
print()
print("   #   Δz(mm)    θ1(°)    θ2(°)   θ3(°)    θ4(°)   爪心(x,z) mm          轴线偏竖(°)  法向仰角(°)")
lat=0; va=[]
for i,(s,qq) in enumerate(tr):
    pc=clawpos(*qq); _a,_o,_n = orientation(*qq, ROLL)
    ang=av(_o); va.append(ang)
    lat=max(lat, abs(pc[0]-p[0]))
    if i%5==0 or i==len(tr)-1:
        print("  %2d  %7.2f  %7.2f  %7.2f  %6.1f  %7.2f   (%8.4f,%8.4f)   %9.5f   %+8.3f"
              % (i,s,sv(qq[0]),sv(qq[1]),ROLL,sv(qq[2]),pc[0],pc[1],ang,
                 np.degrees(np.arctan2(np.sin(sum(qq)*D2R),np.cos(sum(qq)*D2R)))))
print()
print("  横向(x)偏差 ≤ %.6f mm     轴线偏离竖直 ≤ %.6f°（全程恒定）" % (lat, max(va)))
print("  Σ 全程 = %.4f°（恒定 ⇒ 爪板朝向不变）" % SIG)
print()
print("  关节范围（舵机角 [0,180]）：")
for nm,idx in (("θ1 大臂",0),("θ2 小臂",1),("θ4 手腕",2)):
    arr=[sv(t[1][idx]) for t in tr]
    print("    %-9s %7.2f° ~ %7.2f°   行程 %6.2f°   %s"
          % (nm,min(arr),max(arr),max(arr)-min(arr),
             "OK" if min(arr)>=0 and max(arr)<=180 else "**越限**"))
print("    %-9s %7.2f°（锁死）" % ("θ3 爪旋转", ROLL))
print()
print("  零位 → 起点：θ1 90→%.1f   θ2 90→%.1f   θ3 0→%.0f   θ4 90→%.1f"
      % (sv(q[0]),sv(q[1]),ROLL,sv(q[2])))

json.dump(dict(sigma_deg=float(SIG), roll_deg=float(ROLL), start_xy=p.tolist(),
               rows=[dict(s_mm=float(s),
                          servo=[float(sv(qq[0])),float(sv(qq[1])),float(ROLL),float(sv(qq[2]))],
                          cc=clawpos(*qq).tolist(),
                          opening=orientation(*qq,ROLL)[1].tolist()) for s,qq in tr]),
          io.open(B+r"\work_kin_probe\vertical_assembly_traj.json",'w',encoding='utf-8'),
          ensure_ascii=False, indent=1)
print("\n  写入 work_kin_probe/vertical_assembly_traj.json")
