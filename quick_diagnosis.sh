#!/bin/bash

echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║           MN68820 快速診斷工具                                    ║"
echo "╚════════════════════════════════════════════════════════════════════╝"
echo ""

# 檢查 1：編譯配置
echo "檢查 1/3: 確認 MN68820 是否包含在編譯中"
echo "───────────────────────────────────────────────────────────────"
if [ -f "build/SpeedyBeeF405WING/ap_config.h" ]; then
    if grep -q "AP_RANGEFINDER_MN68820_ENABLED.*1" build/SpeedyBeeF405WING/ap_config.h 2>/dev/null; then
        echo "✅ MN68820 驅動已包含在編譯中"
    else
        echo "❌ MN68820 驅動未包含在編譯中"
        echo ""
        echo "解決方案："
        echo "  ./quick_rebuild.sh"
        exit 1
    fi
else
    echo "❌ 找不到編譯文件，需要先編譯"
    echo ""
    echo "解決方案："
    echo "  ./quick_rebuild.sh"
    exit 1
fi
echo ""

# 檢查 2：hwdef.dat 配置
echo "檢查 2/3: 確認 hwdef.dat 中的 I2C 配置"
echo "───────────────────────────────────────────────────────────────"
if grep -q "RANGEFINDER MN68820 I2C:0:0x41" libraries/AP_HAL_ChibiOS/hwdef/SpeedyBeeF405WING/hwdef.dat; then
    echo "✅ I2C 配置正確：I2C:0:0x41"
elif grep -q "RANGEFINDER MN68820 I2C:1:0x41" libraries/AP_HAL_ChibiOS/hwdef/SpeedyBeeF405WING/hwdef.dat; then
    echo "❌ I2C 配置錯誤：應該是 I2C:0 而不是 I2C:1"
    echo ""
    echo "這個問題已經修正，需要重新編譯"
    echo "解決方案："
    echo "  ./quick_rebuild.sh"
    exit 1
else
    echo "⚠️  找不到 RANGEFINDER MN68820 定義"
fi
echo ""

# 檢查 3：韌體文件
echo "檢查 3/3: 確認韌體是否已編譯"
echo "───────────────────────────────────────────────────────────────"
if [ -f "build/SpeedyBeeF405WING/bin/arducopter.apj" ]; then
    SIZE=$(ls -lh build/SpeedyBeeF405WING/bin/arducopter.apj | awk '{print $5}')
    DATE=$(ls -l build/SpeedyBeeF405WING/bin/arducopter.apj | awk '{print $6, $7, $8}')
    echo "✅ 韌體已編譯"
    echo "   大小：$SIZE"
    echo "   日期：$DATE"
else
    echo "❌ 找不到韌體文件"
    echo ""
    echo "解決方案："
    echo "  ./quick_rebuild.sh"
    exit 1
fi
echo ""

echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║                  編譯配置檢查通過！                                ║"
echo "╚════════════════════════════════════════════════════════════════════╝"
echo ""
echo "⚠️  如果您仍然看不到 MN68820 訊息，問題很可能是："
echo ""
echo "1️⃣  參數 RNGFND1_TYPE 不是 45"
echo "   請在 Mission Planner 或 MAVProxy 中確認："
echo "   param show RNGFND1_TYPE"
echo "   如果不是 45，請執行："
echo "   param set RNGFND1_TYPE 45"
echo "   param set RNGFND1_ADDR 65"
echo "   然後重啟飛控"
echo ""
echo "2️⃣  韌體沒有上傳或上傳的是舊版本"
echo "   請重新上傳："
echo "   ./waf copter --upload"
echo ""
echo "3️⃣  在錯誤的地方看 LOG"
echo "   ❌ 不要在 Mission Planner Messages 看"
echo "   ✅ 必須使用 MAVProxy："
echo "   mavproxy.py --master=COM3 --console"
echo ""
echo "4️⃣  設置參數後沒有重啟飛控"
echo "   在 MAVProxy 中執行："
echo "   reboot"
echo ""
echo "════════════════════════════════════════════════════════════════════"
echo ""
echo "📖 詳細診斷步驟請參考："
echo "   cat URGENT_CHECKLIST.md"
echo ""

