# 緊急診斷清單 - 為什麼看不到 MN68820 訊息

## ⚠️ 請逐項確認以下步驟

### 第 1 步：確認您是用 MAVProxy 還是 Mission Planner？

**問題：您在哪裡看 LOG？**

❌ **如果您在 Mission Planner 中看：**
   - Mission Planner 的 Messages 或 Flight Data
   - 這裡**看不到** `DEV_PRINTF()` 的輸出！
   - 只能看到 `GCS_SEND_TEXT()` 的部分訊息

✅ **正確做法：使用 MAVProxy**
```bash
# 在 Windows PowerShell 或 WSL 中執行：
pip3 install MAVProxy
mavproxy.py --master=COM3 --console

# 或 Linux：
mavproxy.py --master=/dev/ttyACM0 --console

# 然後重啟飛控，觀察 MAVProxy 視窗的輸出
```

---

### 第 2 步：確認參數設置

在 Mission Planner 或 MAVProxy 中執行：

**MAVProxy 中：**
```
param show RNGFND1_TYPE
param show RNGFND1_ADDR
```

**Mission Planner 中：**
```
Config → Full Parameter List
搜索 RNGFND1_TYPE
```

**必須是：**
- `RNGFND1_TYPE = 45`  ← 如果不是 45，這就是問題！
- `RNGFND1_ADDR = 65`

**如果不是 45，請設置：**

在 MAVProxy 中：
```
param set RNGFND1_TYPE 45
param set RNGFND1_ADDR 65
```

在 Mission Planner 中：
```
Config → Full Parameter List
RNGFND1_TYPE = 45
RNGFND1_ADDR = 65
點擊 "Write Params"
```

**然後務必重啟飛控！**

---

### 第 3 步：確認韌體版本

檢查飛控是否真的上傳了新韌體：

在 MAVProxy 或 Mission Planner 中查看：
- 韌體版本
- 編譯日期

**應該是今天的日期！** 如果不是，表示韌體沒有正確上傳。

重新上傳：
```bash
cd /home/charles0326/Projects/ardupilot
./waf copter --upload
```

---

### 第 4 步：確認編譯配置

```bash
cd /home/charles0326/Projects/ardupilot

# 檢查是否包含 MN68820
grep "AP_RANGEFINDER_MN68820_ENABLED" build/SpeedyBeeF405WING/ap_config.h

# 應該看到：
# #define AP_RANGEFINDER_MN68820_ENABLED 1
```

如果看不到或是 0，需要重新編譯：
```bash
./quick_rebuild.sh
./waf copter --upload
```

---

### 第 5 步：完整測試流程

**請嚴格按照這個順序執行：**

```bash
# 1. 重新編譯（確保最新代碼）
cd /home/charles0326/Projects/ardupilot
./waf clean
./waf configure --board=SpeedyBeeF405WING
./waf copter

# 2. 確認編譯包含 MN68820
grep "AP_RANGEFINDER_MN68820_ENABLED.*1" build/SpeedyBeeF405WING/ap_config.h
# 如果沒有輸出，表示有問題！

# 3. 上傳韌體
./waf copter --upload
# 等待上傳完成

# 4. 啟動 MAVProxy
mavproxy.py --master=COM3 --console
# 或
mavproxy.py --master=/dev/ttyACM0 --console

# 5. 在 MAVProxy 中設置參數
param set RNGFND1_TYPE 45
param set RNGFND1_ADDR 65
param set RNGFND1_ORIENT 25
param set RNGFND1_MIN_CM 30
param set RNGFND1_MAX_CM 1500

# 6. 重啟飛控（在 MAVProxy 中）
reboot

# 7. 觀察 MAVProxy 輸出
# 應該看到：
# MN68820: detect() called
# MN68820: I2C device OK, creating sensor object
# ...
```

---

## 🔍 預期輸出對照

### ✅ 如果一切正常（在 MAVProxy 中）：

```
... (飛控啟動訊息) ...
MN68820: detect() called
MN68820: I2C device OK, creating sensor object
MN68820: Sensor object created, starting probe and init
MN68820: Probing device...
MN68820: I2C device OK, starting probe...
...
```

### ❌ 如果看到 "RangeFinder not detected" 但沒有 MN68820 訊息：

**表示 detect() 沒被調用，原因是：**

1. **RNGFND1_TYPE 不是 45**  ← 最可能！
   ```
   解決：param set RNGFND1_TYPE 45
        然後 reboot
   ```

2. **韌體沒有包含 MN68820 驅動**
   ```
   解決：重新編譯並上傳
   ```

3. **參數設置後沒有重啟**
   ```
   解決：在 MAVProxy 中執行 reboot
   ```

---

## 📊 Debug 檢查表

請回答以下問題：

□ 您是用 MAVProxy 還是 Mission Planner 看 LOG？
  → 如果是 Mission Planner，請改用 MAVProxy

□ RNGFND1_TYPE 的值是多少？
  → 必須是 45

□ 設置參數後有重啟飛控嗎？
  → 必須重啟

□ 韌體的編譯日期是今天嗎？
  → 如果不是，需要重新上傳

□ 執行 grep 後有看到 AP_RANGEFINDER_MN68820_ENABLED 1 嗎？
  → 如果沒有，需要重新編譯

---

## 💡 常見錯誤

### 錯誤 1：在 Mission Planner 看 LOG
❌ Mission Planner Messages 面板
✅ MAVProxy console 視窗

### 錯誤 2：RNGFND1_TYPE 設錯
❌ RNGFND1_TYPE = 32 (這是 VL53L0X)
❌ RNGFND1_TYPE = 0 (禁用)
✅ RNGFND1_TYPE = 45 (MN68820)

### 錯誤 3：沒有重啟
設置參數後必須：
- 在 MAVProxy 中執行 `reboot`
- 或在 Mission Planner 中點擊 "Write Params" 然後拔插電源

### 錯誤 4：沒有真的上傳新韌體
檢查韌體日期和版本號

---

## 📞 如果還是沒有訊息

請提供以下資訊：

1. MAVProxy 的完整輸出（或螢幕截圖）
2. 執行以下命令的結果：
   ```bash
   param show RNGFND1_TYPE
   param show RNGFND1_ADDR
   grep "AP_RANGEFINDER_MN68820_ENABLED" build/SpeedyBeeF405WING/ap_config.h
   ```
3. 韌體版本和編譯日期
4. 您是如何查看 LOG 的？（MAVProxy 還是 Mission Planner）

