#!/bin/bash

echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║            MN68820 編譯、上傳和診斷流程                            ║"
echo "╚════════════════════════════════════════════════════════════════════╝"
echo ""

# 步驟 1: 清理並配置
echo "步驟 1/4: 清理舊的編譯文件..."
./waf clean

echo ""
echo "步驟 2/4: 配置編譯環境..."
./waf configure --board=SpeedyBeeF405WING

if [ $? -ne 0 ]; then
    echo "❌ 配置失敗！"
    exit 1
fi

echo ""
echo "步驟 3/4: 編譯 Copter 韌體..."
./waf copter

if [ $? -ne 0 ]; then
    echo "❌ 編譯失敗！請檢查錯誤訊息。"
    exit 1
fi

echo ""
echo "步驟 4/4: 檢查編譯結果..."

# 檢查 MN68820 是否被包含在編譯中
if grep -q "AP_RANGEFINDER_MN68820_ENABLED.*1" build/SpeedyBeeF405WING/ap_config.h; then
    echo "   ✅ MN68820 驅動已包含在韌體中"
else
    echo "   ❌ MN68820 驅動未包含在韌體中"
    echo "   請檢查 hwdef.dat 中的設定"
    exit 1
fi

# 檢查韌體大小
if [ -f build/SpeedyBeeF405WING/bin/arducopter.apj ]; then
    SIZE=$(ls -lh build/SpeedyBeeF405WING/bin/arducopter.apj | awk '{print $5}')
    echo "   ✅ 韌體已成功編譯"
    echo "   📦 韌體大小: $SIZE"
    echo "   📂 位置: build/SpeedyBeeF405WING/bin/arducopter.apj"
else
    echo "   ❌ 找不到編譯的韌體文件"
    exit 1
fi

echo ""
echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║                    編譯成功！                                      ║"
echo "╚════════════════════════════════════════════════════════════════════╝"
echo ""
echo "下一步："
echo ""
echo "方式 1 - 自動上傳（推薦）："
echo "  ./waf copter --upload"
echo ""
echo "方式 2 - 手動上傳："
echo "  在 Mission Planner:"
echo "  1. Initial Setup → Install Firmware"
echo "  2. 點擊 'Load custom firmware'"  
echo "  3. 選擇: build/SpeedyBeeF405WING/bin/arducopter.apj"
echo ""
echo "════════════════════════════════════════════════════════════════════"
echo ""
echo "⚠️  上傳後務必設置參數："
echo ""
echo "  RNGFND1_TYPE   = 45    ← 最關鍵！MN68820 的類型編號"
echo "  RNGFND1_ADDR   = 65    ← 0x41 的十進制"
echo "  RNGFND1_ORIENT = 25    ← Down（朝下）"
echo "  RNGFND1_MIN_CM = 30"
echo "  RNGFND1_MAX_CM = 1500"
echo ""
echo "設置完成後重啟飛控，然後使用以下命令查看 debug 輸出："
echo ""
echo "  mavproxy.py --master=/dev/ttyACM0 --console"
echo "或"
echo "  mavproxy.py --master=COM3 --console"
echo ""
echo "════════════════════════════════════════════════════════════════════"

