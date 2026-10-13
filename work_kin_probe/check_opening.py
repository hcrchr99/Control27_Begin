# -*- coding: utf-8 -*-
"""数值核验：爪开合方向随 roll 变化（用三维旋转直接算，不依赖任何口径假设）"""
import numpy as np, sys
try: sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception: pass

RAX = np.array([0.872057, 0.0, 0.489405])      # 腕转轴单位向量（臂面内，斜置）
RAX = RAX / np.linalg.norm(RAX)
Y = np.array([0.0, 1.0, 0.0])                  # 建模位开合方向
UP = np.array([0.0, 0.0, 1.0])

def rodrigues(a, t):
    t = np.radians(t)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(t)*K + (1-np.cos(t))*(K @ K)

print("腕转轴 = (%.6f, %.6f, %.6f)   与水平面夹角 = %.4f°"
      % (*RAX, np.degrees(np.arctan2(RAX[2], RAX[0]))))
print("                             与竖直夹角 = %.4f°"
      % np.degrees(np.arccos(RAX[2])))
print()
print("开合方向 = R(腕转轴, roll) · ŷ    （ŷ = 建模位开合方向，横向）")
print("  roll(°)    开合方向(x,y,z)                与竖直(±z)夹角(°)")
best = None
for R in range(0, 181, 10):
    v = rodrigues(RAX, R) @ Y
    ang = np.degrees(np.arccos(np.clip(abs(v @ UP), -1, 1)))
    if best is None or ang < best[0]:
        best = (ang, R, v.copy())
    print("   %4d    (%+.5f, %+.5f, %+.5f)      %7.3f" % (R, *v, ang))
print()
for R in np.arange(0, 180.01, 0.01):
    v = rodrigues(RAX, R) @ Y
    ang = np.degrees(np.arccos(np.clip(abs(v @ UP), -1, 1)))
    if ang < best[0]:
        best = (ang, R, v.copy())
print("  最接近竖直：roll = %.2f°，开合方向 = (%.5f, %.5f, %.5f)，夹角 %.3f°"
      % (best[1], *best[2], best[0]))
print()
print("  几何解释：开合方向扫出的是以腕转轴为轴、半顶角 90° 的【大圆】")
print("            （因为 ŷ ⊥ 腕转轴，半顶角恰好 90°）")
print("  腕转轴与竖直成 %.2f°  →  大圆上离竖直最近的点也差 %.2f°"
      % (np.degrees(np.arccos(RAX[2])), np.degrees(np.arccos(RAX[2]))))
print()
print("  ⇒ 爪子夹持的圆柱轴线【永远无法竖直】，最少差 %.1f°" % best[0])
