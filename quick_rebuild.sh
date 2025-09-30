#!/bin/bash

echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║        MN68820 快速重新編譯（I2C 索引已修正）                     ║"
echo "╚════════════════════════════════════════════════════════════════════╝"
echo ""

echo "🔧 已修正的問題："
echo "   I2C:1:0x41 → I2C:0:0x41"
echo ""

# 清理並編譯
echo "步驟 1/3: 清理..."
./waf clean > /dev/null 2>&1

echo "步驟 2/3: 配置編譯環境..."
./waf configure --board=SpeedyBeeF405WING

if [ $? -ne 0 ]; then
    echo "❌ 配置失敗！"
    exit 1
fi

echo "步驟 3/3: 編譯韌體..."
./waf copter

if [ $? -ne 0 ]; then
    echo ""
    echo "❌ 編譯失敗！請檢查錯誤訊息。"
    exit 1
fi

echo ""
echo "✅ 編譯成功！"
echo ""

# 檢查配置
echo "驗證編譯配置："
echo "────────────────────────────────────────────────────────────────────"

if grep -q "AP_RANGEFINDER_MN68820_ENABLED.*1" build/SpeedyBeeF405WING/ap_config.h 2>/dev/null; then
    echo "✅ MN68820 驅動已包含"
else
    echo "❌ MN68820 驅動未包含"
    exit 1
fi

if grep -q "RANGEFINDER MN68820 I2C:0:0x41" build/SpeedyBeeF405WING/hwdef.log 2>/dev/null; then
    echo "✅ I2C 配置正確：I2C:0:0x41"
else
    echo "⚠️  無法驗證 I2C 配置（可能是正常的）"
fi

SIZE=$(ls -lh build/SpeedyBeeF405WING/bin/arducopter.apj 2>/dev/null | awk '{print $5}')
if [ ! -z "$SIZE" ]; then
    echo "✅ 韌體已生成：$SIZE"
else
    echo "❌ 找不到韌體文件"
    exit 1
fi

echo ""
echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║                       下一步操作                                   ║"
echo "╚════════════════════════════════════════════════════════════════════╝"
echo ""
echo "1️⃣  上傳韌體："
echo "    ./waf copter --upload"
echo ""
echo "2️⃣  確認參數（在 Mission Planner 中）："
echo "    RNGFND1_TYPE = 45"
echo "    RNGFND1_ADDR = 65"
echo "    重啟飛控！"
echo ""
echo "3️⃣  使用 MAVProxy 查看 debug 輸出："
echo "    mavproxy.py --master=COM3 --console"
echo ""
echo "4️⃣  預期看到："
echo "    MN68820: detect() called"
echo "    MN68820: I2C device OK, creating sensor object"
echo "    ..."
echo ""
echo "📖 如果有電平轉換板相關問題，參考："
echo "    cat I2C_LEVEL_SHIFTER_GUIDE.md"
echo ""

