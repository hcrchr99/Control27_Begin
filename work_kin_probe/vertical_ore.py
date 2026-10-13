# -*- coding: utf-8 -*-
"""核验：通过臂面三关节把【腕转轴】摆成水平，则爪开合方向(=矿石轴线)完全竖直

坐标系：臂面 (x,z)，x 前 / z 上；关节用仰角域 el（+=抬头），el = 90 − 舵机角。
"""
import numpy as np, sys
try: sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception: pass
np.set_printoptions(precision=5, suppress=True)

RAX = np.array([0.872057, 0.0, 0.489405]); RAX /= np.linalg.norm(RAX)
Y = np.array([0.0, 1.0, 0.0]); UP = np.array([0.0, 0.0, 1.0])
RAX_TILT = np.degrees(np.arctan2(RAX[2], RAX[0]))     # 腕转轴在臂面内的仰角
print("腕转轴在臂面内的安装仰角 = %.4f°" % RAX_TILT)
print()

def rot_y(t):
    c, s = np.cos(np.radians(t)), np.sin(np.radians(t))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def rod(a, t):
    t = np.radians(t)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(t)*K + (1-np.cos(t))*(K@K)

def opening_dir(el1, el2, el3, roll_deg):
    """开合方向（世界系）。臂面内总转角 = el1+el2+el3（仰角域）"""
    sig = el1 + el2 + el3
    a_world = rot_y(sig) @ RAX
    return rod(a_world, roll_deg) @ Y, a_world

print("=" * 96)
print("① 腕转轴与竖直的夹角 随 Σ=el1+el2+el3 变化（roll 不改变它）")
print("=" * 96)
print("   Σ(°)     腕转轴(x,y,z)                 与竖直夹角(°)   roll=90时的爪轴线与竖直夹角(°)")
for sig in range(-90, 91, 10):
    v, a = opening_dir(sig, 0, 0, 90)
    ang_axis = np.degrees(np.arccos(np.clip(abs(a @ UP), -1, 1)))
    ang_open = np.degrees(np.arccos(np.clip(abs(v @ UP), -1, 1)))
    print("   %+5d   (%+.4f, %+.4f, %+.4f)      %8.3f          %8.3f"
          % (sig, *a, ang_axis, ang_open))
print()

# 求 Σ 使腕转轴水平（与竖直夹角 = 90°）
best = None
for sig in np.arange(-180, 180.01, 0.005):
    _, a = opening_dir(sig, 0, 0, 0)
    d = abs(np.degrees(np.arccos(np.clip(abs(a @ UP), -1, 1))) - 90.0)
    if best is None or d < best[0]:
        best = (d, sig, a.copy())
print("  腕转轴水平 ⇔ Σ = %.4f°   此时轴 = (%.5f, %.5f, %.5f)"
      % (best[1], *best[2]))
print("  理论值 = %.4f° − 90° = %.4f°" % (RAX_TILT, RAX_TILT - 90.0))
print()

SIG_VERT = best[1]
print("=" * 96)
print("② Σ = %.4f°，roll = 90° 时爪轴线是否完全竖直" % SIG_VERT)
print("=" * 96)
for el1, el2 in [(-30, -9.465), (0, -39.465), (20, -59.465), (-40, 0.535)]:
    el3 = SIG_VERT - el1 - el2
    if abs(el3) > 90: continue
    v, a = opening_dir(el1, el2, el3, 90)
    ang = np.degrees(np.arccos(np.clip(abs(v @ UP), -1, 1)))
    print("   el=(%+7.3f, %+7.3f, %+7.3f)  开合方向=(%+.5f,%+.5f,%+.5f)  与竖直夹角 = %.6f°"
          % (el1, el2, el3, *v, ang))
print()
print("  ⇒ 与竖直夹角 = 0.000000°：**完全竖直**（不是 29.3°）")
print("     roll=90° 的 29.3° 残角只在 Σ=0（零位）时成立。")
print()

print("=" * 96)
print("③ 修正后的结论")
print("=" * 96)
print("  old 结论（错）: roll=90° 时开合方向距竖直 29.3°，是几何极限")
print("  正确结论     : 开合方向 = 绕【腕转轴】转 roll 后的 ŷ；")
print("                 而腕转轴的空间朝向 = R_y(Σ)·RAX，由 Σ = el1+el2+el3 决定。")
print("                 Σ = %.2f° ⇒ 腕转轴水平 ⇒ roll=90° ⇒ 爪轴线严格竖直。" % SIG_VERT)
print()
print("  自由度账：")
print("    · 竖直约束      →  Σ 定死一个值")
print("    · θ3(roll)=90°  →  锁死")
print("    · 剩 θ1, θ2     →  两个自由度，正好定爪心 (x, z) 平面位置")
print("    · θ4 = Σ − θ1 − θ2 收尾")
print("  ⇒ 竖直直线插补【完全可行】，且是唯一解，无需迭代")
