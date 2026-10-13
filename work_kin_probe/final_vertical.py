# -*- coding: utf-8 -*-
"""最终核验：竖直装配解的关键性质"""
import numpy as np, sys, json, io
try: sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception: pass
np.set_printoptions(precision=5, suppress=True)
SH = np.array([48.900, 46.198]); UP = np.array([111.100, 63.400])
RJ = np.array([58.354, 26.496]); RJY = 46.593
WR = np.array([23.723, 21.742]); WRY = -46.593
TL = np.array([70.6734, -47.8161]); TLY = 56.550
RAX3 = np.array([0.872057, 0.0, 0.489405]); RAX3 /= np.linalg.norm(RAX3)
Y3 = np.array([0.,1.,0.]); Z3 = np.array([0.,0.,1.])
D2R, R2D = np.pi/180.0, 180.0/np.pi
SIG = 60.6985; ROLL = 90.0

def rot_y(t):
    c,s = np.cos(t*D2R), np.sin(t*D2R); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rod(a,t):
    t=t*D2R; K=np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
    return np.eye(3)+np.sin(t)*K+(1-np.cos(t))*(K@K)

print("=" * 92)
print("竖直装配解核验")
print("=" * 92)
print("  Σ = el1+el2+el3 = %.4f°    θ3(roll) = %.1f°" % (SIG, ROLL))
print()
print("  随 Σ 变化：腕转轴朝向与爪轴线（roll=90）")
print("    Σ(°)    腕转轴(x,y,z)                 轴/竖直夹角(°)   爪轴线/竖直夹角(°)")
for s in (-90, -60, -30, 0, 29.3, SIG, 90, 120, 150.7):
    a = rot_y(s) @ RAX3
    v = rod(a, ROLL) @ Y3
    aa = np.degrees(np.arccos(np.clip(abs(a@Z3),-1,1)))
    vv = np.degrees(np.arccos(np.clip(abs(v@Z3),-1,1)))
    mark = "  <<< 本方案" if abs(s-SIG) < 0.01 else ("  (零位)" if abs(s) < 0.01 else "")
    print("   %+7.2f  (%+.4f,%+.4f,%+.4f)   %10.4f      %10.4f%s" % (s,*a,aa,vv,mark))
print()
print("  规律：爪轴线与竖直的夹角 = |Σ − 29.30°| 在 roll=90° 时")
print("        Σ = 0      → 29.30°（原注释说的几何极限，其实只是零位）")
print("        Σ = 60.70° →  0.00°（腕转轴水平）  ← 本方案")
print()

d = json.load(io.open(r"vertical_assembly_traj.json", encoding='utf-8'))
rows = d['rows']
print("=" * 92)
print("轨迹（爪心 x = %.1f mm 竖直下压 70 mm）" % d['start_xy'][0])
print("=" * 92)
print("   #   Δz(mm)    θ1(°)    θ2(°)   θ3(°)    θ4(°)    Σ_el(°)   爪轴线/竖直(°)   爪板法向仰角(°)")
lat = 0.0; angs = []
for i, r in enumerate(rows):
    e = r['el']; sig = sum(e)
    a = rot_y(sig) @ RAX3
    v = rod(a, ROLL) @ Y3
    vang = np.degrees(np.arccos(np.clip(abs(v@Z3),-1,1)))
    nrm = np.degrees(np.arctan2(np.sin(sig*D2R), np.cos(sig*D2R)))
    angs.append(vang)
    lat = max(lat, abs(r['cc'][0] - d['start_xy'][0]))
    if i % 5 == 0 or i == len(rows)-1:
        print("  %2d  %7.2f  %7.3f  %7.3f  %6.1f  %7.3f  %+8.4f   %10.5f      %+8.3f"
              % (i, r['s_mm'], r['servo'][0], r['servo'][1], r['servo'][2], r['servo'][3],
                 sig, vang, nrm))
print()
print("  横向偏差 ≤ %.6f mm        爪轴线偏离竖直 ≤ %.5f°（恒定）" % (lat, max(angs)))
print("  爪板法向仰角 = Σ = %.4f°（恒定 ⇒ 爪板朝向严格不变）" % sum(rows[0]['el']))
print()
print("  关节限位（[0,180]）：")
for nm, j in (("\u03b81 \u5927\u81c2",0),("\u03b82 \u5c0f\u81c2",1),("\u03b83 roll",2),("\u03b84 \u624b\u8155",3)):
    arr = [r['servo'][j] for r in rows]
    ok = min(arr) >= 0 and max(arr) <= 180
    print("    %-10s %7.2f\u00b0 ~ %7.2f\u00b0   \u884c\u7a0b %6.2f\u00b0   %s"
          % (nm, min(arr), max(arr), max(arr)-min(arr), "OK" if ok else "**\u8d8a\u9650**"))
print()
print("  零位→本方案起始姿态的变化：")
print("    θ1 90.0 → %.1f    θ2 90.0 → %.1f    θ3 0.0(or 180) → %.1f    θ4 90.0 → %.1f"
      % (rows[0]['servo'][0], rows[0]['servo'][1], rows[0]['servo'][2], rows[0]['servo'][3]))
