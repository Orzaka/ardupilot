#!/usr/bin/env python3
"""檢查 ArduPilot I2C 和 RangeFinder 配置"""

import os
import re

print("=" * 70)
print("  檢查 SpeedyBeeF405WING 的 I2C 和 RangeFinder 配置")
print("=" * 70)
print()

# 檢查 hwdef.h
hwdef_path = "build/SpeedyBeeF405WING/hwdef.h"
if os.path.exists(hwdef_path):
    print("✓ 找到 hwdef.h")
    with open(hwdef_path, 'r') as f:
        content = f.read()
        
    # 檢查 I2C 配置
    print("\n【I2C 配置】")
    i2c_matches = re.findall(r'#define HAL_I2C.*', content)
    if i2c_matches:
        for match in i2c_matches[:10]:
            print(f"  {match}")
    else:
        print("  ❌ 未找到 I2C 配置")
    
    # 檢查 MN68820
    print("\n【MN68820 配置】")
    mn68820_matches = re.findall(r'.*MN68820.*', content)
    if mn68820_matches:
        for match in mn68820_matches[:10]:
            print(f"  {match}")
    else:
        print("  ❌ 未找到 MN68820 配置")
        
    # 檢查 RangeFinder 相關
    print("\n【RangeFinder 配置】")
    rf_matches = re.findall(r'.*RANGEFINDER.*', content)
    if rf_matches:
        for match in rf_matches[:10]:
            print(f"  {match}")
    else:
        print("  ❌ 未找到 RANGEFINDER 配置")
else:
    print("❌ 未找到 hwdef.h，請先編譯韌體")

# 檢查 hwdef.dat
print("\n" + "=" * 70)
print("【hwdef.dat 配置】")
print("=" * 70)
hwdef_dat = "libraries/AP_HAL_ChibiOS/hwdef/SpeedyBeeF405WING/hwdef.dat"
if os.path.exists(hwdef_dat):
    with open(hwdef_dat, 'r') as f:
        lines = f.readlines()
    
    print("\nI2C 相關配置：")
    for i, line in enumerate(lines, 1):
        if 'I2C' in line.upper() and not line.strip().startswith('#'):
            print(f"  Line {i:3d}: {line.rstrip()}")
    
    print("\nRANGEFINDER 相關配置：")
    for i, line in enumerate(lines, 1):
        if 'RANGEFINDER' in line.upper() or 'MN68820' in line.upper():
            print(f"  Line {i:3d}: {line.rstrip()}")
else:
    print("❌ 未找到 hwdef.dat")

print("\n" + "=" * 70)
print("【需要確認的事項】")
print("=" * 70)
print("""
1. hwdef.dat 中應該有：
   - I2C_ORDER I2C1
   - PB8 I2C1_SCL I2C1
   - PB9 I2C1_SDA I2C1
   - RANGEFINDER MN68820 I2C:1:0x41

2. 參數應該設置為（在 Mission Planner 中）：
   - RNGFND1_TYPE = 45
   - RNGFND1_ADDR = 65 (0x41 的十進制)

3. 如果以上都正確但仍無輸出，可能是：
   - 參數沒有儲存/重啟
   - hwdef 配置在 minimize_fpv_osd.inc 之後被覆蓋
""")

