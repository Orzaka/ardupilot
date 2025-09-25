#!/bin/bash

# ArduPilot 同步腳本
# 用於從官方 ArduPilot 倉庫同步更新到您的私人倉庫

echo "🔄 開始同步 ArduPilot..."

# 檢查當前分支
CURRENT_BRANCH=$(git branch --show-current)
echo "📍 當前分支: $CURRENT_BRANCH"

# 1. 從官方倉庫獲取最新更新
echo "📥 從官方 ArduPilot 倉庫獲取更新..."
git fetch upstream

# 2. 切換到主分支
if [ "$CURRENT_BRANCH" != "master" ]; then
    echo "🔄 切換到 master 分支..."
    git checkout master
fi

# 3. 合併官方更新
echo "🔀 合併官方更新..."
git merge upstream/master

# 4. 推送到您的私人倉庫
echo "📤 推送到您的私人倉庫..."
git push origin master

# 5. 如果有其他分支，也推送
echo "📤 推送所有分支到私人倉庫..."
git push origin --all

echo "✅ 同步完成！"
echo ""
echo "📋 同步摘要："
echo "   - 官方倉庫: https://github.com/ArduPilot/ardupilot"
echo "   - 您的倉庫: https://github.com/orzaka/ardupilot"
echo "   - 當前分支: $(git branch --show-current)"
echo "   - 最新提交: $(git log --oneline -1)"
