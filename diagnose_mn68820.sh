#!/bin/bash

echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║       MN68820 RangeFinder 診斷工具                                ║"
echo "╚════════════════════════════════════════════════════════════════════╝"
echo ""

# 1. 檢查編譯配置
echo "【1】檢查編譯配置..."
if [ -f "build/SpeedyBeeF405WING/hwdef.h" ]; then
    echo "✓ 韌體已編譯"
    
    if grep -q "AP_RANGEFINDER_MN68820_ENABLED 1" build/SpeedyBeeF405WING/hwdef.h; then
        echo "✓ MN68820 驅動已啟用"
    else
        echo "✗ MN68820 驅動未啟用！"
    fi
    
    if grep -q "HAL_I2C1_CONFIG" build/SpeedyBeeF405WING/hwdef.h; then
        echo "✓ I2C1 已配置"
    else
        echo "✗ I2C1 未配置！"
    fi
else
    echo "✗ 韌體尚未編譯，請先執行："
    echo "  ./waf configure --board=SpeedyBeeF405WING"
    echo "  ./waf copter"
    exit 1
fi

echo ""
echo "【2】檢查 hwdef.dat 配置..."
if grep -q "RANGEFINDER MN68820" libraries/AP_HAL_ChibiOS/hwdef/SpeedyBeeF405WING/hwdef.dat; then
    echo "✓ hwdef.dat 中有 RANGEFINDER MN68820 定義"
    grep "RANGEFINDER MN68820" libraries/AP_HAL_ChibiOS/hwdef/SpeedyBeeF405WING/hwdef.dat
else
    echo "✗ hwdef.dat 中缺少 RANGEFINDER MN68820 定義"
fi

echo ""
echo "【3】檢查 Type 枚舉值..."
if grep -q "MN68820 = 45" libraries/AP_RangeFinder/AP_RangeFinder.h; then
    echo "✓ MN68820 的 Type = 45"
else
    echo "⚠ 找不到 MN68820 Type 定義"
fi

echo ""
echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║  診斷結果                                                          ║"
echo "╚════════════════════════════════════════════════════════════════════╝"
echo ""
echo "如果上述檢查都通過，但仍然看到 'rangefinder1 not detected'，"
echo "最可能的原因是："
echo ""
echo "❌ 參數 RNGFND1_TYPE 沒有設置為 45"
echo ""
echo "請在 Mission Planner 中："
echo "1. Config → Full Parameter List"
echo "2. 搜索 RNGFND1_TYPE"
echo "3. 設置為 45"
echo "4. 點擊 'Write Params'"
echo "5. 重啟飛控"
echo ""
echo "然後使用 MAVProxy 查看啟動訊息："
echo "  mavproxy.py --master=/dev/ttyACM0 --console"
echo "或"
echo "  mavproxy.py --master=COM3 --console"
echo ""
echo "重啟飛控後，應該會看到："
echo "  MN68820: detect() called"
echo "  ..."
echo ""

