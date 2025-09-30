# MN68820 RangeFinder 診斷檢查清單

## 📋 當您只看到 "RangeFinder not detected" 時的診斷步驟

### ✅ 第一步：確認韌體編譯

```bash
# 執行編譯腳本
./compile_and_check.sh

# 或手動編譯
./waf clean
./waf configure --board=SpeedyBeeF405WING
./waf copter

# 檢查是否包含 MN68820
grep "AP_RANGEFINDER_MN68820_ENABLED" build/SpeedyBeeF405WING/ap_config.h
# 應該看到：#define AP_RANGEFINDER_MN68820_ENABLED 1
```

**預期結果：** ✅ 看到 `AP_RANGEFINDER_MN68820_ENABLED 1`

---

### ✅ 第二步：上傳韌體

```bash
# 方式 1: 命令行上傳
./waf copter --upload

# 方式 2: Mission Planner
# Initial Setup → Install Firmware → Load custom firmware
# 選擇：build/SpeedyBeeF405WING/bin/arducopter.apj
```

**預期結果：** ✅ 韌體上傳成功，飛控重啟

---

### ✅ 第三步：設置參數（最關鍵！）

**在 Mission Planner 中：**

```
Config → Full Parameter List

設置以下參數：
RNGFND1_TYPE = 45     ← 如果這個不是 45，detect() 根本不會被調用！
RNGFND1_ADDR = 65     ← 0x41 的十進制
RNGFND1_ORIENT = 25   ← Down
RNGFND1_MIN_CM = 30
RNGFND1_MAX_CM = 1500
```

**或載入參數文件：**
```
Load from file → MN68820_Parameters.param
Write Params
```

**預期結果：** ✅ 參數寫入成功

⚠️ **務必重啟飛控！**

---

### ✅ 第四步：使用 MAVProxy 查看 Console 輸出

```bash
# 安裝 MAVProxy（如果還沒有）
pip3 install MAVProxy

# 連接飛控
mavproxy.py --master=/dev/ttyACM0 --console    # Linux/WSL
# 或
mavproxy.py --master=COM3 --console            # Windows

# 重啟飛控，觀察輸出
```

**預期看到（如果一切正常）：**
```
MN68820: detect() called
MN68820: I2C device OK, creating sensor object
MN68820: Sensor object created, starting probe and init
MN68820: probe() returned 1
MN68820: init() returned 1
MN68820: Detection successful!
```

**如果看到錯誤：**

| 錯誤訊息 | 問題 | 解決方案 |
|---------|------|---------|
| `MN68820: I2C device is null in detect()` | I2C 設備創建失敗 | 檢查硬體連接和 I2C 配置 |
| `MN68820: I2C device not responding` | 硬體沒回應 | 檢查接線、電源、上拉電阻 |
| `MN68820: probe() returned 0` | Probe 失敗 | MN68820 沒回應 0x41 狀態 |
| `MN68820: init() returned 0` | Init 失敗 | 韌體下載失敗 |
| 完全沒有 MN68820 訊息 | detect() 未被調用 | **檢查 RNGFND1_TYPE 是否 = 45** |

---

### ✅ 第五步：檢查當前參數值

在 Mission Planner 中查看：

```
DATA → Messages 或 Flight Data → Actions → MAVLink
```

使用 MAVProxy 查看：
```
param show RNGFND1_*
```

**確認：**
- RNGFND1_TYPE = 45 ✓
- RNGFND1_ADDR = 65 ✓

---

### ✅ 第六步：檢查硬體連接

```
□ MN68820 VCC → 飛控 3.3V 或 5V（確認感測器規格！）
□ MN68820 GND → 飛控 GND
□ MN68820 SCL → 飛控 PB8 (I2C1_SCL)
□ MN68820 SDA → 飛控 PB9 (I2C1_SDA)
□ I2C 總線上有 4.7K 上拉電阻（到 VCC）
□ 接線長度 < 20cm（太長會有問題）
□ 沒有短路或接觸不良
```

---

### ✅ 第七步：使用 I2C 掃描工具（進階）

如果您有 I2C 掃描工具（如 Arduino + I2C Scanner），確認：
- MN68820 在 I2C 總線上可見
- 地址確實是 0x41

---

## 🔍 常見問題 FAQ

**Q: 為什麼只看到 "RangeFinder not detected" 而沒有其他訊息？**

A: 最可能的原因是 `RNGFND1_TYPE` 沒有設置為 45。如果 TYPE 參數不對，
   ArduPilot 根本不會調用 MN68820 的 detect() 函數。

**Q: 我已經設置了 RNGFND1_TYPE = 45，但還是沒有訊息**

A: 
1. 確認參數已經寫入（Write Params）
2. 確認飛控已重啟
3. 使用 MAVProxy 查看 console 輸出（不是 Mission Planner）
4. 檢查編譯的韌體是否包含 MN68820 驅動

**Q: 如何確認 I2C1 已啟用？**

A: 執行：
```bash
bash /tmp/check_i2c_config.sh
```
應該看到所有 ✅ 標記。

**Q: DEV_PRINTF 輸出到哪裡？**

A: DEV_PRINTF 輸出到 console，必須使用 MAVProxy 才能看到：
```bash
mavproxy.py --master=/dev/ttyACM0 --console
```

**Q: GCS_SEND_TEXT 輸出到哪裡？**

A: GCS_SEND_TEXT 輸出到地面站（Mission Planner/QGC）的 Messages 面板。

---

## 📞 回報問題時請提供

1. MAVProxy console 的完整輸出
2. Mission Planner Messages 的截圖
3. `param show RNGFND1_*` 的輸出
4. 硬體連接照片（如果可能）

