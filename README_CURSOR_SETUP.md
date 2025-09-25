# 🚁 ArduPilot Cursor 開發環境設置指南

## 📋 目錄
- [在新電腦上設置](#在新電腦上設置)
- [Cursor 配置](#cursor-配置)
- [日常使用流程](#日常使用流程)
- [故障排除](#故障排除)

## 🖥️ 在新電腦上設置

### 1. 克隆您的私人倉庫
```bash
git clone https://github.com/orzaka/ardupilot.git
cd ardupilot
```

### 2. 運行快速設置腳本
```bash
./quick_setup.sh
```

### 3. 安裝開發依賴（Ubuntu/Debian）
```bash
# 安裝系統依賴
sudo apt update
sudo apt install python3 python3-pip python3-venv git

# 安裝 ArduPilot 依賴
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

### 4. 在 Cursor 中打開項目
1. 打開 Cursor
2. `File` → `Open Folder`
3. 選擇 `ardupilot` 文件夾
4. 等待 Cursor 索引完成

## ⚙️ Cursor 配置

### 推薦擴展
- **C/C++ Extension Pack** - C/C++ 開發支持
- **GitLens** - Git 增強功能
- **Arduino** - Arduino 開發支持
- **Python** - Python 腳本支持

### 工作區設置
項目已包含 `.vscode/settings.json` 和 `.vscode/tasks.json` 配置文件。

### 快捷任務
按 `Ctrl+Shift+P` 打開命令面板，輸入 "Tasks: Run Task" 可以看到：
- **ArduPilot: 配置 SpeedyBeeF405WING** - 配置構建環境
- **ArduPilot: 編譯 ArduCopter** - 編譯韌體
- **ArduPilot: 清理構建** - 清理構建文件
- **Git: 同步官方更新** - 同步官方代碼
- **Git: 提交修改** - 提交您的修改

## 🔄 日常使用流程

### 開始工作前
```bash
# 同步最新更新
git sync
```

### 開發過程中
1. 修改代碼
2. 測試編譯：`Ctrl+Shift+P` → "Tasks: Run Task" → "ArduPilot: 編譯 ArduCopter"
3. 定期提交：`Ctrl+Shift+P` → "Tasks: Run Task" → "Git: 提交修改"

### 結束工作時
```bash
# 提交並推送修改
git save
```

## 🛠️ 手動命令

### Git 操作
```bash
# 查看狀態
git status

# 添加修改
git add .

# 提交修改
git commit -m "您的提交訊息"

# 推送到私人倉庫
git push origin master

# 同步官方更新
git sync

# 提交修改
git save
```

### 構建操作
```bash
# 配置構建環境
./waf configure --board=SpeedyBeeF405WING

# 編譯韌體
./waf build --target=bin/arducopter

# 清理構建
./waf clean

# 查看幫助
./waf --help
```

## 🔧 故障排除

### 權限問題
```bash
# 設置 Git 憑證
git config --global credential.helper store
```

### 編譯問題
```bash
# 清理並重新配置
./waf clean
./waf configure --board=SpeedyBeeF405WING
./waf build --target=bin/arducopter
```

### 同步衝突
```bash
# 查看衝突
git status

# 解決衝突後
git add .
git commit -m "Resolve merge conflicts"
git push origin master
```

### Cursor 智能感知問題
1. 按 `Ctrl+Shift+P`
2. 輸入 "C/C++: Reset IntelliSense Database"
3. 重新索引項目

## 📁 項目結構

```
ardupilot/
├── .vscode/                 # Cursor 配置文件
│   ├── settings.json       # 工作區設置
│   └── tasks.json          # 任務配置
├── libraries/              # 核心庫
│   └── AP_RangeFinder/     # 距離感測器庫
│       └── AP_RangeFinder_MN68820.*  # MN68820 驅動
├── ArduCopter/             # 四軸飛行器代碼
├── Tools/                  # 開發工具
├── waf                     # 構建系統
├── sync_with_upstream.sh   # 同步腳本
├── commit_changes.sh       # 提交腳本
└── quick_setup.sh          # 快速設置腳本
```

## 🎯 您的自定義修改

### MN68820 距離感測器驅動
- **文件位置**: `libraries/AP_RangeFinder/AP_RangeFinder_MN68820.*`
- **主要修改**:
  - 修正 I2C 地址為 `0x41`
  - 添加詳細調試日誌
  - 基於 STM32F103 實現的邏輯

### 如何保持修改
1. 定期運行 `git sync` 同步官方更新
2. 如果有衝突，手動解決後提交
3. 使用 `git save` 提交您的修改
4. 推送到您的私人倉庫

## 📞 需要幫助？

如果遇到問題：
1. 檢查 [ArduPilot 官方文檔](https://ardupilot.org/dev/)
2. 查看 [GitHub Issues](https://github.com/ArduPilot/ardupilot/issues)
3. 運行 `./waf --help` 查看構建幫助

---

**祝您開發愉快！** 🚁✨
