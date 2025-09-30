# MN68820 + 電平轉換板診斷指南

## 🔧 已修正的問題

✅ **I2C 總線索引已修正**
- 從 `I2C:1:0x41` → `I2C:0:0x41`
- 現在會正確使用 I2C1 總線

## ⚡ 電平轉換板相關問題

### 常見的電平轉換板問題：

1. **雙重上拉電阻**
   - 飛控的 I2C 總線通常有上拉電阻（到 5V）
   - 電平轉換板兩側通常也有上拉電阻
   - MN68820 可能也有板載上拉電阻
   - **問題**：太多上拉電阻會降低電阻值，導致通訊失敗

2. **時序問題**
   - 電平轉換板會增加信號延遲
   - 可能需要降低 I2C 速度

3. **電源問題**
   - 確認電平轉換板的 3.3V 側有穩定電源
   - 確認共地（GND）連接良好

## 🔍 I2C 總線共享情況

在同一個 I2C1 總線上：
```
飛控 I2C1 (5V) ───[電平轉換板]─── MN68820 (3.3V, 地址 0x41)
        │
        └─── SPL06 氣壓計 (地址 0x76) ← 雖然被禁用，但可能還在總線上
```

**潛在問題：**
- SPL06 (0x76) 雖然被軟體禁用，但硬體上可能還在總線上
- 如果 SPL06 有問題，可能會拉低整個 I2C 總線

## 📋 診斷步驟

### 步驟 1：重新編譯韌體

```bash
cd /home/charles0326/Projects/ardupilot
./waf clean
./waf configure --board=SpeedyBeeF405WING
./waf copter
```

檢查編譯輸出，確認：
```bash
grep "RANGEFINDER MN68820 I2C:0:0x41" build/SpeedyBeeF405WING/hwdef.log
```

### 步驟 2：檢查電平轉換板連接

```
飛控側（5V）：
  □ VCC(5V) → 電平轉換板 HV
  □ GND → 電平轉換板 GND（必須共地！）
  □ PB8 (SCL) → 電平轉換板 HV_SCL
  □ PB9 (SDA) → 電平轉換板 HV_SDA

MN68820側（3.3V）：
  □ 3.3V → 電平轉換板 LV 和 MN68820 VCC
  □ GND → 電平轉換板 GND 和 MN68820 GND（共地！）
  □ 電平轉換板 LV_SCL → MN68820 SCL
  □ 電平轉換板 LV_SDA → MN68820 SDA
```

### 步驟 3：檢查上拉電阻

**可能的配置：**

方案 A：使用電平轉換板的上拉電阻（推薦）
- 移除或禁用飛控側的上拉（如果可能）
- 電平轉換板通常有 10K 上拉

方案 B：只在 3.3V 側使用上拉
- 3.3V 側：4.7K 上拉到 3.3V
- 5V 側：移除上拉

⚠️ **不要在兩側同時使用強上拉！**

### 步驟 4：降低 I2C 速度（如果需要）

如果仍有通訊問題，在 hwdef.dat 中添加：

```
# 降低 I2C 速度以改善電平轉換板的相容性
define HAL_I2C_INTERNAL_MASK 0
define HAL_I2C_HIGHSPEED 0
```

### 步驟 5：測試基本 I2C 通訊

上傳韌體後，使用 MAVProxy：

```bash
mavproxy.py --master=COM3 --console
```

重啟飛控，應該看到：
```
MN68820: detect() called
MN68820: I2C device OK, creating sensor object
...
```

如果看到：
```
MN68820: I2C device not responding
```

表示電平轉換板或連接有問題。

## 🛠️ 故障排除

### 問題 1：完全沒有 MN68820 訊息

**可能原因：**
- 韌體未正確編譯或上傳
- RNGFND1_TYPE 不是 45

**解決：**
```bash
# 確認編譯包含 MN68820
grep "AP_RANGEFINDER_MN68820_ENABLED.*1" build/SpeedyBeeF405WING/ap_config.h

# 重新上傳
./waf copter --upload

# 確認參數
param show RNGFND1_TYPE    # 應該是 45
param show RNGFND1_ADDR    # 應該是 65
```

### 問題 2：看到 "I2C device not responding"

**可能原因：**
- 電平轉換板連接問題
- 沒有共地
- 上拉電阻問題
- I2C 速度太快

**解決：**
1. 用萬用表檢查連接
2. 確認所有 GND 連在一起
3. 檢查電平轉換板的上拉電阻配置
4. 嘗試降低 I2C 速度

### 問題 3："I2C device is null in detect()"

**可能原因：**
- I2C 總線索引錯誤（應該已修正）
- I2C1 未正確初始化

**解決：**
```bash
# 檢查 hwdef
grep "I2C" libraries/AP_HAL_ChibiOS/hwdef/SpeedyBeeF405WING/hwdef.dat
# 應該看到 I2C_ORDER I2C1
```

### 問題 4：間歇性通訊失敗

**可能原因：**
- 電平轉換板時序問題
- 電源不穩定
- 線路太長或干擾

**解決：**
1. 縮短線路長度（< 10cm）
2. 使用遮蔽線
3. 檢查 3.3V 電源是否穩定（用示波器）
4. 降低 I2C 速度

## 📊 預期行為

**正常情況下（MAVProxy 輸出）：**
```
MN68820: detect() called
MN68820: I2C device OK, creating sensor object
MN68820: Sensor object created, starting probe and init
MN68820: Probing device...
MN68820: I2C device OK, starting probe...
MN68820: I2C device responding, continuing probe...
MN68820: Writing reset command 0x81 to register 0xE0
MN68820: Reset command written successfully
MN68820: Waiting 100ms for device to process reset...
MN68820: Starting CPU ready check loop...
MN68820: Read register 0xE0 = 0x41 (attempt X)
MN68820: Probe successful, CPU ready (0x41) after X attempts
MN68820: probe() returned 1
MN68820: Starting initialization
...
MN68820: Initialization completed successfully
MN68820: init() returned 1
MN68820: Detection successful!
```

## 🔬 進階診斷：使用 Arduino I2C Scanner

如果還是有問題，建議使用 Arduino + I2C Scanner：

1. 直接連接飛控的 I2C（5V 側）→ 掃描是否看到 SPL06 (0x76)
2. 連接電平轉換板的 3.3V 側 → 掃描是否看到 MN68820 (0x41)

這樣可以隔離是硬體問題還是軟體問題。

