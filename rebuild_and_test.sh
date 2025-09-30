#!/bin/bash

echo "=========================================="
echo "  MN68820 RangeFinder 重新編譯和測試"
echo "=========================================="
echo ""

# 清理並重新編譯
echo "1. 清理舊的編譯文件..."
./waf clean

echo ""
echo "2. 配置編譯（SpeedyBeeF405WING）..."
./waf configure --board=SpeedyBeeF405WING

echo ""
echo "3. 編譯 Copter 韌體..."
./waf copter

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ 編譯成功！"
    echo ""
    echo "=========================================="
    echo "  下一步：上傳韌體和設置參數"
    echo "=========================================="
    echo ""
    echo "方式1：命令行上傳"
    echo "  ./waf copter --upload"
    echo ""
    echo "方式2：Mission Planner 上傳"
    echo "  1. 在 Mission Planner 中選擇 'Load custom firmware'"
    echo "  2. 選擇文件：build/SpeedyBeeF405WING/bin/arducopter.apj"
    echo ""
    echo "上傳後，務必設置以下參數："
    echo "  RNGFND1_TYPE  = 45"
    echo "  RNGFND1_ADDR  = 65"
    echo "  RNGFND1_ORIENT = 25"
    echo "  RNGFND1_MIN_CM = 30"
    echo "  RNGFND1_MAX_CM = 1500"
    echo ""
    echo "然後使用 MAVProxy 查看 debug 輸出："
    echo "  mavproxy.py --master=/dev/ttyACM0 --console"
    echo "或"
    echo "  mavproxy.py --master=COM3 --console"
    echo ""
else
    echo ""
    echo "❌ 編譯失敗！請檢查錯誤訊息。"
    echo ""
fi

