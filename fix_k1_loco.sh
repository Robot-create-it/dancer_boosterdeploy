#!/bin/bash

# K1_LOCO 快速修复脚本
# 修复准备模式的vyaw=0.2问题

set -e

REPO_DIR="/home/booster/Workspace/dancer_boosterdeploy"
TARGET_FILE="$REPO_DIR/tasks/locomotion/nested_locomotion.py"
BACKUP_FILE="${TARGET_FILE}.backup_$(date +%Y%m%d_%H%M%S)"

cd "$REPO_DIR"

echo "================================================"
echo "  K1_LOCO 问题修复脚本"
echo "================================================"
echo ""

# 检查文件是否存在
if [ ! -f "$TARGET_FILE" ]; then
    echo "❌ 错误：找不到文件 $TARGET_FILE"
    exit 1
fi

# 备份原文件
echo "📦 备份原文件..."
cp "$TARGET_FILE" "$BACKUP_FILE"
echo "   备份保存到: $BACKUP_FILE"
echo ""

# 显示修改前的内容
echo "🔍 修改前（第42-45行）："
sed -n '42,45p' "$TARGET_FILE" | cat -n
echo ""

# 执行修复
echo "🔧 执行修复..."
sed -i '44s/self\.processed = \[0\.0, 0\.0, 0\.2\]/self.processed = [0.0, 0.0, 0.0]/' "$TARGET_FILE"

# 显示修改后的内容
echo "✅ 修改后（第42-45行）："
sed -n '42,45p' "$TARGET_FILE" | cat -n
echo ""

# 验证修改
if grep -q "self.processed = \[0.0, 0.0, 0.0\]" "$TARGET_FILE"; then
    echo "✓ 修复成功！"
else
    echo "❌ 修复失败，正在恢复原文件..."
    cp "$BACKUP_FILE" "$TARGET_FILE"
    exit 1
fi

echo ""
echo "================================================"
echo "  修复完成！"
echo "================================================"
echo ""
echo "📝 修改内容："
echo "   第44行：vyaw 从 0.2 改为 0.0"
echo ""
echo "🚀 下一步操作："
echo ""
echo "1. 启动k1_loco任务："
echo "   cd $REPO_DIR"
echo "   python scripts/deploy.py --task k1_loco"
echo ""
echo "2. 测试步骤："
echo "   ① 按 X → 观察机器人是否保持静止站立（不应旋转）"
echo "   ② 按 A/r → 启用RL速度控制"
echo "   ③ 按 w → 增加vx=0.1，等待10秒"
echo "   ④ 观察机器人是否缓慢前进（加速需要~1秒）"
echo "   ⑤ 按 空格 → 停止所有运动"
echo ""
echo "📊 预期行为："
echo "   - 按X后：静止站立，无旋转"
echo "   - 按w后：0.5-1秒内开始缓慢前进"
echo "   - 速度增长：每0.02秒增加约0.026 m/s"
echo ""
echo "⚠️  注意事项："
echo "   - K1_LOCO的加速比K1_WALK慢（设计行为）"
echo "   - 需要持续给命令或等待速度积累"
echo "   - 按一次w可能看不到明显效果，需多按几次或等待"
echo ""
echo "🔙 如需恢复原文件："
echo "   cp $BACKUP_FILE $TARGET_FILE"
echo ""
