# -*- coding: utf-8 -*-
import io
p = r"C:\Users\Admin\WorkBuddy\RM_Begin\work_kin_probe\verify2.c"
s = io.open(p, encoding='utf-8').read()
s = s.replace('float dir = k==0 ? -90.0f : nd;', 'float dir = k==0 ? 50.535f : nd;')
s = s.replace('printf("  竖直向下(-90°) 可用 s', 'printf("  dir=+50.5 可用 s')
io.open(p, 'w', encoding='utf-8').write(s)
print("patched")
