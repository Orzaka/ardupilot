# 在新電腦上設置 ArduPilot 開發環境

## 1. 克隆您的私人倉庫

```bash
# 克隆您的私人倉庫
git clone https://github.com/orzaka/ardupilot.git
cd ardupilot

# 添加官方倉庫作為上游
git remote add upstream https://github.com/ArduPilot/ardupilot.git

# 驗證遠程倉庫設定
git remote -v
```

## 2. 安裝開發環境

### Ubuntu/Debian:
```bash
# 安裝依賴
sudo apt update
sudo apt install python3 python3-pip python3-venv git

# 安裝 ArduPilot 依賴
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

### Windows (WSL):
```bash
# 在 WSL 中執行上述 Ubuntu 命令
```

### macOS:
```bash
# 安裝 Homebrew (如果沒有)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# 安裝依賴
brew install python3 git
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-macos.sh
```

## 3. 配置工具鏈

```bash
# 配置 ArduPilot
cd ardupilot
./waf configure --board=SpeedyBeeF405WING

# 或者配置其他板子
./waf configure --board=CubeOrange
```

## 4. 在 Cursor 中打開項目

1. 打開 Cursor
2. File → Open Folder
3. 選擇 ardupilot 文件夾
4. 等待 Cursor 索引完成

## 5. 設置同步腳本

將以下腳本複製到新電腦的 ardupilot 目錄：

### sync_with_upstream.sh
```bash
#!/bin/bash
echo "🔄 開始同步 ArduPilot..."
CURRENT_BRANCH=$(git branch --show-current)
echo "📍 當前分支: $CURRENT_BRANCH"

echo "📥 從官方 ArduPilot 倉庫獲取更新..."
git fetch upstream

if [ "$CURRENT_BRANCH" != "master" ]; then
    echo "🔄 切換到 master 分支..."
    git checkout master
fi

echo "🔀 合併官方更新..."
git merge upstream/master

echo "📤 推送到您的私人倉庫..."
git push origin master

echo "📤 推送所有分支到私人倉庫..."
git push origin --all

echo "✅ 同步完成！"
```

### commit_changes.sh
```bash
#!/bin/bash
echo "💾 提交 ArduPilot 修改..."

if [ -z "$(git status --porcelain)" ]; then
    echo "ℹ️  沒有修改需要提交"
    exit 0
fi

echo "📝 修改的文件："
git status --short

echo "➕ 添加修改到暫存區..."
git add .

echo "💬 提交修改..."
read -p "請輸入提交訊息 (或按 Enter 使用默認): " commit_msg

if [ -z "$commit_msg" ]; then
    commit_msg="Update ArduPilot with custom modifications"
fi

git commit -m "$commit_msg"

echo "📤 推送到私人倉庫..."
git push origin master

echo "✅ 提交完成！"
```

## 6. 設置 Git 別名

```bash
chmod +x sync_with_upstream.sh commit_changes.sh
git config alias.sync '!./sync_with_upstream.sh'
git config alias.save '!./commit_changes.sh'
```

## 7. 驗證設置

```bash
# 測試編譯
./waf build --target=bin/arducopter

# 測試同步
git sync

# 測試提交
git save
```

## 8. Cursor 設置建議

### 推薦的 Cursor 擴展：
- C/C++ Extension Pack
- GitLens
- Arduino
- Python

### 工作區設置 (.vscode/settings.json):
```json
{
    "C_Cpp.default.configurationProvider": "ms-vscode.cpptools",
    "C_Cpp.default.intelliSenseMode": "gcc-x64",
    "files.associations": {
        "*.h": "c",
        "*.cpp": "cpp"
    },
    "terminal.integrated.defaultProfile.linux": "bash"
}
```

## 9. 日常使用流程

1. **開始工作前**：
   ```bash
   git sync  # 同步最新更新
   ```

2. **開發過程中**：
   - 修改代碼
   - 測試編譯：`./waf build --target=bin/arducopter`
   - 定期提交：`git save`

3. **結束工作時**：
   ```bash
   git save  # 提交並推送修改
   ```

## 10. 故障排除

### 如果遇到權限問題：
```bash
# 設置 Git 憑證
git config --global credential.helper store
```

### 如果遇到編譯問題：
```bash
# 清理並重新配置
./waf clean
./waf configure --board=SpeedyBeeF405WING
./waf build --target=bin/arducopter
```

### 如果遇到同步衝突：
```bash
# 查看衝突
git status

# 解決衝突後
git add .
git commit -m "Resolve merge conflicts"
git push origin master
```
