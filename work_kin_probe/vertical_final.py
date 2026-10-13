# -*- coding: utf-8 -*-
"""快速权威版：先解析求 Σ，再粗网格找轨迹，最后精化"""
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

def Ry3(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c,0.,s],[0.,1.,0.],[-s,0.,c]])
def rod(a, ang):
    t = ang*D2R
    K = np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
    return np.eye(3)*np.cos(t) + (1-np.cos(t))*np.outer(a,a) + np.sin(t)*K
def av(v): return np.degrees(np.arccos(np.clip(abs(v@Z3),-1,1)))

print("=" * 100); print("① 零位自检"); print("=" * 100)
ax_ = Ry3(0) @ RAX3; op_ = rod(ax_, 0) @ Y3
print("  腕转轴 = %s  与竖直 %.4f°" % (np.round(ax_,5), av(ax_)))
print("  开合方向(roll=0) = %s  与竖直 %.4f°" % (np.round(op_,5), av(op_)))
print()
print("=" * 100); print("② 求 Σ 使开合方向竖直（roll=90°）"); print("=" * 100)
# 解析：开合方向 = rod(Ry(Σ)RAX3, 90)·ŷ。要它 = ±ẑ
best=None
for s in np.arange(-180,180.001,0.002):
    a = Ry3(s*D2R) @ RAX3
    o = rod(a, ROLL) @ Y3
    d = av(o)
    if best is None or d<best[0]: best=(d,s,o.copy(),a.copy())
for s in (-90,-60,-30,0,29.3015,60,90,120,150.7):
    a = Ry3(s*D2R)@RAX3; o = rod(a,ROLL)@Y3
    print("  Σ=%+8.3f  腕转轴/竖直 %10.4f°   开合方向 (%+.5f,%+.5f,%+.5f)  /竖直 %10.5f°"
          % (s, av(a), *o, av(o)))
print()
SIG = best[1]
print("  ★ Σ = %.4f°   腕转轴 = %s (与竖直 %.5f°)   开合方向 = %s (与竖直 %.6f°)"
      % (SIG, np.round(best[3],5), av(best[3]), np.round(best[2],6), best[0]))
print("    舵机角：三俯仰之和 = 270 − %.4f = %.4f°" % (SIG, 270-SIG))
print()

# ---- 位置 FK / IK ----
def eff(el3):
    a3=el3*D2R; ca3,sa3=np.cos(a3),np.sin(a3)
    wxx = WR[0]+TL[0]*ca3-TL[1]*sa3
    wzz = WR[1]+TL[0]*sa3+TL[1]*ca3
    wy  = WRY+TLY; ax,az = RAX3[0],RAX3[2]
    bx,bz = RJ[0]+wy*(-az), RJ[1]+wy*(ax)
    return np.hypot(bx,bz), np.degrees(np.arctan2(bz,bx))
def cpos(e1,e2,e3):
    lb,bb = eff(e3)
    return np.array([SH[0]+LA*np.cos((BEA+e1)*D2R)+lb*np.cos((bb+e1+e2)*D2R),
                     SH[1]+LA*np.sin((BEA+e1)*D2R)+lb*np.sin((bb+e1+e2)*D2R)])
def ik(tgt, e0):
    q=np.array(e0,float)
    for _ in range(120):
        e=tgt-cpos(q[0],q[1],SIG-q[0]-q[1])
        if np.linalg.norm(e)<1e-11: break
        J=np.zeros((2,2)); h=1e-6
        for i in range(2):
            a=q.copy(); a[i]+=h; b=q.copy(); b[i]-=h
            J[:,i]=(cpos(a[0],a[1],SIG-a.sum())-cpos(b[0],b[1],SIG-b.sum()))/(2*h)
        try: dq=np.linalg.solve(J+1e-14*np.eye(2),e)
        except np.linalg.LinAlgError: return None
        q=q+0.6*dq
        q[0]=np.clip(q[0],-LIM,LIM); q[1]=np.clip(q[1],-LIM,LIM)
    if np.linalg.norm(tgt-cpos(q[0],q[1],SIG-q[0]-q[1]))>1e-6: return None
    e3=SIG-q[0]-q[1]
    return None if abs(e3)>LIM else np.array([q[0],q[1],e3])

def sv(el): return 90.0-el
found=[]
for x in np.arange(40,360,4.0):
    for z in np.arange(60,260,4.0):
        p=np.array([x,z],float); q=None
        for sd in ((0.,0.),(40.,-40.),(-40.,40.)):
            q=ik(p,sd)
            if q is not None: break
        if q is None: continue
        ok=True; qq=q[:2].copy(); tr=[]
        for k in range(15):
            s=70.0*k/14.0
            q2=ik(p+np.array([0.,-s]),qq)
            if q2 is None: ok=False; break
            qq=q2[:2].copy(); tr.append((s,q2))
        if ok: found.append((p,q,tr))
print("=" * 100); print("③ 竖直下压 70mm（粗网格可行起点 %d 个）" % len(found)); print("=" * 100)
if not found: print("  无解"); sys.exit(0)
found.sort(key=lambda t:(abs(t[0][0]-230.0), -t[0][1]))
p,q,tr = found[0]
# 精化到 31 点
qq=q[:2].copy(); tr=[]
for k in range(31):
    s=70.0*k/30.0
    q2=ik(p+np.array([0.,-s]),qq)
    if q2 is None: print("  精化失败"); sys.exit(1)
    qq=q2[:2].copy(); tr.append((s,q2))
print("  起点爪心 (%.1f, %.1f) mm   舵机 θ=(%.2f, %.2f, %.1f, %.2f)"
      % (p[0],p[1],sv(q[0]),sv(q[1]),ROLL,sv(q[2])))
print()
print("   #   Δz(mm)    θ1(°)    θ2(°)   θ3(°)    θ4(°)   爪心(x,z) mm           轴线偏竖(°)")
lat=0; va=[]
for i,(s,qq2) in enumerate(tr):
    pc=cpos(*qq2)
    o = rod(Ry3(sum(qq2)*D2R)@RAX3, ROLL)@Y3
    ang=av(o); va.append(ang); lat=max(lat,abs(pc[0]-p[0]))
    if i%5==0 or i==len(tr)-1:
        print("  %2d  %7.2f  %7.2f  %7.2f  %6.1f  %7.2f   (%9.4f,%9.4f)   %9.5f"
              % (i,s,sv(qq2[0]),sv(qq2[1]),ROLL,sv(qq2[2]),pc[0],pc[1],ang))
print()
print("  横向(x)偏差 ≤ %.6f mm      轴线偏离竖直 ≤ %.6f°" % (lat, max(va)))
print("  Σ 恒定 = %.4f°  ⇒ 爪板法向不变" % SIG)
print()
print("  关节范围（舵机角 [0,180]）：")
for nm,idx in (("θ1 大臂",0),("θ2 小臂",1),("θ4 手腕",2)):
    arr=[sv(t[1][idx]) for t in tr]
    print("    %-9s %7.2f° ~ %7.2f°  行程 %6.2f°  %s" % (nm,min(arr),max(arr),max(arr)-min(arr),
          "OK" if min(arr)>=0 and max(arr)<=180 else "**越限**"))
print("    %-9s %7.2f°（锁死）" % ("θ3 爪旋转", ROLL))
print()
print("  零位 → 起点：θ1 90→%.1f  θ2 90→%.1f  θ3 0→%.0f  θ4 90→%.1f"
      % (sv(q[0]),sv(q[1]),ROLL,sv(q[2])))
json.dump(dict(sigma_deg=float(SIG), roll_deg=float(ROLL), start_xy=p.tolist(),
               rows=[dict(s_mm=float(s),
                          servo=[float(sv(qq2[0])),float(sv(qq2[1])),float(ROLL),float(sv(qq2[2]))],
                          cc=cpos(*qq2).tolist(),
                          opening=(rod(Ry3(sum(qq2)*D2R)@RAX3,ROLL)@Y3).tolist()) for s,qq2 in tr]),
          io.open(B+r"\work_kin_probe\vertical_assembly_traj.json",'w',encoding='utf-8'),
          ensure_ascii=False, indent=1)
print("\n  写入 work_kin_probe/vertical_assembly_traj.json")
