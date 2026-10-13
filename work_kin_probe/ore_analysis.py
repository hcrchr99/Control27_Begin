# -*- coding: utf-8 -*-
"""分析：爪开合方向（=矿石圆柱轴线）能否竖直；以及装配动作该怎么走

坐标系：臂面 (x, z)，x 前 / z 上；关节角用「仰角」域（+ = 抬头），
        el = 90 − 舵机角；与 robot_config.h / kinematics.c 同口径。
"""
import numpy as np, sys
try: sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception: pass
np.set_printoptions(precision=4, suppress=True)

# ---- robot_config.h 常量（改后） ----
SH = np.array([48.900, 46.198])
UP = np.array([111.100, 63.400])
RJ = np.array([58.354, 26.496]); RJY = 46.593
WR = np.array([23.723, 21.742]); WRY = -46.593
TL = np.array([70.6734, -47.8161]); TLY = 56.550
RAX = np.array([0.872057, 0.489405])
R4 = np.array([70.673, -47.818])

D2R = np.pi / 180.0
R2D = 180.0 / np.pi


def fk_full(servo, roll_deg):
    """完整 FK（含 roll 旋转），返回 爪心(x,z,y)、开合方向、爪板法向、腕俯仰轴位"""
    el = 90.0 - np.asarray(servo, float)
    el1, el2, el3 = el
    # roll 旋转作用在 (WRIST_YZ + TOOL) 上
    gx, gz = TL
    a3 = el3 * D2R
    ca3, sa3 = np.cos(a3), np.sin(a3)
    wxx = WR[0] + gx * ca3 - gz * sa3
    wzz = WR[1] + gx * sa3 + gz * ca3
    wy = WRY + TLY
    ax, az = RAX
    px, pz = -az, ax
    adotw = ax * wxx + az * wzz
    cr, sr = np.cos(roll_deg * D2R), np.sin(roll_deg * D2R)
    tx = cr * wxx + sr * wy * px + (1 - cr) * adotw * ax
    tz = cr * wzz + sr * wy * pz + (1 - cr) * adotw * az
    bx = RJ[0] + tx
    bz = RJ[1] + tz
    tip_y = RJY + wy * cr + (az * wxx - ax * wzz) * sr
    # 平面 2R
    lb = np.hypot(bx, bz)
    bb = np.arctan2(bz, bx)
    la = np.hypot(*UP)
    be = np.arctan2(UP[1], UP[0])
    a1 = be + el1 * D2R
    a2 = bb + (el1 + el2) * D2R
    x = SH[0] + la * np.cos(a1) + lb * np.cos(a2)
    z = SH[1] + la * np.sin(a1) + lb * np.sin(a2)
    # 开合方向 = Ra(roll)·ŷ
    op = np.array([sr * (-az), sr * ax])          # 面内投影
    oy = cr
    # 爪板法向：Σ = el1+el2+el3（仰角代数和），法向在面内
    sigma = el1 + el2 + el3
    nrm = np.array([np.cos(sigma * D2R), np.sin(sigma * D2R)])
    # 腕俯仰轴位
    wp = SH + UP + RJ + WR
    return dict(x=x, z=z, y=tip_y, open_inplane=op, open_y=oy,
                normal=nrm, sigma=sigma, wp=wp)


print("=" * 92)
print("① 爪开合方向能指到多接近竖直？")
print("=" * 92)
best = None
for R in np.arange(0, 180.01, 0.05):
    r = fk_full([90, 90, 90], R)
    op = r['open_inplane']
    v = np.array([op[0], r['open_y'], op[1]])
    v = v / np.linalg.norm(v)
    # 与竖直向下 (0,0,-1) 的夹角
    ang = np.degrees(np.arccos(np.clip(v @ np.array([0, 0, -1.0]), -1, 1)))
    if best is None or ang < best[0]:
        best = (ang, R, v)
ang, Rbest, vbest = best
print("  开合方向 = (%.5f, %.5f, %.5f)   （面内 x, 横向 y, 竖直 z）" % tuple(vbest))
print("  roll = %.2f° 时距竖直最近：%.2f°" % (Rbest, ang))
print("  开合方向单位向量 =", np.round(vbest, 5))
print()

print("  各 roll 下的开合方向（看看像不像锥面）：")
print("    roll(°)   面内投影(a,b)      横向 y     距竖直(°)")
for R in (0, 20, 40, 60, 80, 100, 120, Rbest, 150, 180):
    r = fk_full([90, 90, 90], R)
    op = r['open_inplane']
    v = np.array([op[0], r['open_y'], op[1]])
    v /= np.linalg.norm(v)
    a = np.degrees(np.arccos(np.clip(v @ np.array([0, 0, -1.0]), -1, 1)))
    print("   %7.1f   (%+.4f, %+.4f)   %+.4f    %7.2f"
          % (R, op[0], op[1], r['open_y'], a))
print()

print("=" * 92)
print("② 在 roll = %.1f°（开合方向最接近竖直）时，各关节角的影响" % Rbest)
print("=" * 92)
print("  爪板法向 Σ 与开合方向是两个独立的朝向自由度：")
print("    · 开合方向 由 roll 唯一决定（矿石圆柱轴线 = 它）")
print("    · 爪板法向 由 Σ = el1+el2+el3 决定（只影响爪板朝向）")
print("    · 让爪板法向 ⊥ 圆柱轴线 时，爪板才能贴住矿石侧面")
print()
r = fk_full([90, 90, 90], Rbest)
print("  roll=%.1f 时 面内投影 = (%.4f, %.4f) → 面内仰角 %.2f°"
      % (Rbest, r['open_inplane'][0], r['open_inplane'][1],
         np.degrees(np.arctan2(r['open_inplane'][1], r['open_inplane'][0]))))
print("  横向 y 分量 = %.4f" % r['open_y'])
print("  ⇒ 开合方向在臂面内的投影仰角 %.2f°，横向占 %.2f%%"
      % (np.degrees(np.arctan2(r['open_inplane'][1], r['open_inplane'][0])),
         abs(r['open_y']) * 100))
