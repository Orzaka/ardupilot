#!/bin/bash

# ArduPilot 提交腳本
# 用於提交您的修改並推送到私人倉庫

echo "💾 提交 ArduPilot 修改..."

# 檢查是否有修改
if [ -z "$(git status --porcelain)" ]; then
    echo "ℹ️  沒有修改需要提交"
    exit 0
fi

# 顯示修改的文件
echo "📝 修改的文件："
git status --short

# 添加所有修改
echo "➕ 添加修改到暫存區..."
git add .

# 提交修改
echo "💬 提交修改..."
read -p "請輸入提交訊息 (或按 Enter 使用默認): " commit_msg

if [ -z "$commit_msg" ]; then
    commit_msg="Update ArduPilot with custom modifications"
fi

git commit -m "$commit_msg"

# 推送到私人倉庫
echo "📤 推送到私人倉庫..."
git push origin master

echo "✅ 提交完成！"
echo ""
echo "📋 提交摘要："
echo "   - 提交訊息: $commit_msg"
echo "   - 提交哈希: $(git rev-parse --short HEAD)"
echo "   - 倉庫地址: https://github.com/orzaka/ardupilot"
