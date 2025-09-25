#!/bin/bash

# ArduPilot 快速設置腳本
# 用於在新電腦上快速設置開發環境

echo "🚀 ArduPilot 快速設置腳本"
echo "================================"

# 檢查是否在 ardupilot 目錄中
if [ ! -f "waf" ]; then
    echo "❌ 錯誤：請在 ardupilot 根目錄中運行此腳本"
    exit 1
fi

# 1. 設置 Git 遠程倉庫
echo "📡 設置 Git 遠程倉庫..."
if ! git remote get-url upstream >/dev/null 2>&1; then
    git remote add upstream https://github.com/ArduPilot/ardupilot.git
    echo "✅ 已添加官方倉庫作為上游"
else
    echo "ℹ️  上游倉庫已存在"
fi

# 2. 創建同步腳本
echo "📝 創建同步腳本..."

cat > sync_with_upstream.sh << 'EOF'
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
EOF

cat > commit_changes.sh << 'EOF'
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
EOF

# 3. 設置腳本權限
chmod +x sync_with_upstream.sh commit_changes.sh
echo "✅ 已創建同步腳本"

# 4. 設置 Git 別名
echo "🔧 設置 Git 別名..."
git config alias.sync '!./sync_with_upstream.sh'
git config alias.save '!./commit_changes.sh'
echo "✅ 已設置 Git 別名"

# 5. 檢查依賴
echo "🔍 檢查開發環境..."

# 檢查 Python
if command -v python3 &> /dev/null; then
    echo "✅ Python3: $(python3 --version)"
else
    echo "❌ Python3 未安裝"
fi

# 檢查 Git
if command -v git &> /dev/null; then
    echo "✅ Git: $(git --version)"
else
    echo "❌ Git 未安裝"
fi

# 檢查 waf
if [ -f "waf" ]; then
    echo "✅ Waf 構建系統已就緒"
else
    echo "❌ Waf 構建系統未找到"
fi

# 6. 顯示使用說明
echo ""
echo "🎉 設置完成！"
echo "================================"
echo ""
echo "📋 使用方法："
echo "  git sync    - 同步官方更新"
echo "  git save    - 提交您的修改"
echo "  ./waf build --target=bin/arducopter - 編譯韌體"
echo ""
echo "📁 您的倉庫："
echo "  私人倉庫: $(git remote get-url origin)"
echo "  官方倉庫: $(git remote get-url upstream)"
echo ""
echo "🔧 下一步："
echo "  1. 在 Cursor 中打開此文件夾"
echo "  2. 運行 'git sync' 同步最新更新"
echo "  3. 開始開發！"
