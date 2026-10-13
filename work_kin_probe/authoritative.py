# -*- coding: utf-8 -*-
"""权威版：爪开合方向（=矿石轴线）的完整运动学 + 竖直解 + 轨迹
   唯一实现，不做任何口径假设。
"""
import numpy as np, sys, json, io
try: sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception: pass
np.set_printoptions(precision=5, suppress=True)
B = r"C:\Users\Admin\WorkBuddy\RM_Begin"

# ---- robot_config.h 常量 ----
SH = np.array([48.900, 46.198]); UP = np.array([111.100, 63.400])
RJ = np.array([58.354, 26.496]); RJY = 46.593
WR = np.array([23.723, 21.742]); WRY = -46.593
TL = np.array([70.6734, -47.8161]); TLY = 56.550
RAX3 = np.array([0.872057, 0.0, 0.489405]); RAX3 /= np.linalg.norm(RAX3)
Y3 = np.array([0.0, 1.0, 0.0]); Z3 = np.array([0.0, 0.0, 1.0])
D2R, R2D = np.pi/180.0, 180.0/np.pi
LIM = 90.0

def Ry(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
def rodrigues(a, ang):
    """绕单位轴 a 转 ang（度）"""
    t = ang*D2R
    return Ry(0)*0 + np.eye(3)*np.cos(t) + (1-np.cos(t))*np.outer(a,a) \
           + np.sin(t)*np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])

def orientation(el1, el2, el3, roll_deg):
    """返回 (腕转轴世界方向, 开合方向世界方向, 爪板法向(面内2D))"""
    sigma = el1 + el2 + el3
    axis = Ry(sigma*D2R) @ RAX3              # 腕转轴随臂面三关节转过 Σ
    opening = rodrigues(axis, roll_deg) @ Y3 # 开合方向 = 绕腕转轴转 roll 后的 ŷ
    normal = np.array([np.cos(sigma*D2R), np.sin(sigma*D2R)])
    return axis, opening, normal

def pos(el1, el2, el3, roll_deg):
    """爪心 (x, z) 与横向 y"""
    sigma = el1+el2+el3
    a3 = el3*D2R; ca3, sa3 = np.cos(a3), np.sin(a3)
    wxx = WR[0] + TL[0]*ca3 - TL[1]*sa3
    wzz = WR[1] + TL[0]*sa3 + TL[1]*ca3
    wy  = WRY + TLY
    ax, az = RAX3[0], RAX3[2]
    adotw = ax*wxx + az*wzz
    cr, sr = np.cos(roll_deg*D2R), np.sin(roll_deg*D2R)
    tx = cr*wxx + sr*wy*(-az) + (1-cr)*adotw*ax
    tz = cr*wzz + sr*wy*( ax) + (1-cr)*adotw*az
    bx, bz = RJ[0]+tx, RJ[1]+tz
    tip_y = RJY + wy*cr + (az*wxx - ax*wzz)*sr
    lb = np.hypot(bx,bz); bb = np.arctan2(bz,bx)
    la = np.hypot(*UP);   be = np.arctan2(UP[1],UP[0])
    x = SH[0] + la*np.cos(be+el1*D2R) + lb*np.cos(bb+(el1+el2)*D2R)
    z = SH[1] + la*np.sin(be+el1*D2R) + lb*np.sin(bb+(el1+el2)*D2R)
    return np.array([x,z]), tip_y

def ang_to_vert(v):
    return np.degrees(np.arccos(np.clip(abs(v @ Z3), -1.0, 1.0)))

print("=" * 96)
print("① 零位自检（三关节舵机 90° → el=0，roll=0）")
print("=" * 96)
a, o, n = orientation(0,0,0, 0)
print("  腕转轴 = %s   与竖直 %.4f°   与水平 %.4f°"
      % (np.round(a,5), ang_to_vert(a), 90-ang_to_vert(a)))
print("  开合方向 = %s  与竖直 %.4f°" % (np.round(o,5), ang_to_vert(o)))
print("  （roll=0 时开合方向纯横向 y，与竖直差 90°，符合预期）")
print()

print("=" * 96)
print("② 让开合方向竖直：扫 Σ（roll 固定在 90°）")
print("=" * 96)
print("   Σ(°)     腕转轴/竖直(°)   开合方向(x,y,z)              开合方向/竖直(°)")
best = None
for s in np.arange(-180, 180.001, 0.001):
    a, o, n = orientation(s, 0, 0, 90.0)
    d = ang_to_vert(o)
    if best is None or d < best[0]:
        best = (d, s, o.copy(), a.copy())
for s in (-90, -60.7, -29.3015, 0, 29.3015, 60.7, 90):
    a, o, n = orientation(s,0,0,90.0)
    print("  %+8.3f    %10.4f    (%+.5f,%+.5f,%+.5f)   %10.5f"
          % (s, ang_to_vert(a), *o, ang_to_vert(o)))
print()
print("  ★ 最优 Σ = %.4f°" % best[1])
print("    腕转轴 = %s   与竖直 %.4f°  ← 水平" % (np.round(best[3],5), ang_to_vert(best[3])))
print("    开合方向 = %s   与竖直 %.6f°" % (np.round(best[2],5), best[0]))
print()
SIG = best[1]
