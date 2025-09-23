# 本地CI测试指南

在本地使用Docker容器测试您的代码，避免在GitHub上等待CI结果。

## 🚀 快速开始

### 1. 确保依赖已安装
```bash
# 检查Docker和Docker Compose
docker --version
docker compose --version
```

### 2. 运行快速测试
```bash
# 仅检查编译是否通过（推荐日常开发使用）
./scripts/local-ci.sh test-quick
```

### 3. 运行完整测试
```bash
# 包含所有测试（推荐提交前使用）
./scripts/local-ci.sh test-full
```

## 📋 所有命令

```bash
# 构建CI环境
./scripts/local-ci.sh build

# 快速测试（仅编译）
./scripts/local-ci.sh test-quick

# 完整测试（包含测试运行）
./scripts/local-ci.sh test-full

# 自定义测试选项
./scripts/local-ci.sh test-custom --no-trac-ik --debug

# 进入容器进行调试
./scripts/local-ci.sh shell # 出问题再使用调试

# 查看测试日志
./scripts/local-ci.sh logs

# 清理环境
./scripts/local-ci.sh clean # 出问题再使用调试
```

## 🛠️ 自定义选项

### TracIK配置
```bash
# 启用TracIK（默认）
./scripts/local-ci.sh test-custom --trac-ik

# 禁用TracIK
./scripts/local-ci.sh test-custom --no-trac-ik
```

### 构建模式
```bash
# Debug模式
./scripts/local-ci.sh test-custom --debug

# Release模式（默认）
./scripts/local-ci.sh test-custom
```

### 跳过测试
```bash
# 只编译，不运行测试
./scripts/local-ci.sh test-custom --no-tests
```

## 🔍 调试技巧

### 进入容器调试
```bash
# 启动并进入容器
./scripts/local-ci.sh shell

# 在容器内手动运行测试
cd /workspace
source /opt/ros/humble/setup.bash
/usr/local/bin/ci-test ON Release true
```

### 查看构建产物
```bash
# 进入容器后
ls install/trajectory_planning_v3/lib/trajectory_planning_v3/

# 测试运行节点
source install/setup.bash
ros2 run trajectory_planning_v3 trajectory_planning_node --help
```

### 查看详细日志
```bash
# 查看colcon构建日志
cat log/latest_build/events.log

# 查看测试结果
cat log/latest_test/test_results.xml
```

## 📊 测试环境

本地CI环境完全复现GitHub Actions环境：

- **操作系统**: Ubuntu 22.04
- **ROS版本**: Humble
- **MoveIt2**: 预编译版本（快速测试）
- **工具链**: GCC 11, CMake 3.22, Python 3.10

## 🔧 故障排除

### 常见问题

**1. Docker权限问题**
```bash
# 将用户添加到docker组
sudo usermod -aG docker $USER
# 重新登录后生效
```

**2. 构建缓存问题**
```bash
# 清理并重新构建
./scripts/local-ci.sh clean
./scripts/local-ci.sh build --no-cache
```

**3. 内存不足**
```bash
# 限制并行编译数量
export COLCON_BUILD_ARGS="--parallel-workers 2"
./scripts/local-ci.sh test-quick
```

### 查看系统资源
```bash
# 在容器内查看资源使用
docker stats trajectory_planning_ci

# 清理Docker系统
docker system prune -a
```

## 🚦 测试策略

### 日常开发流程
```bash
# 1. 修改代码
vim src/trajectory_planning_v3/src/...

# 2. 快速检查编译
./scripts/local-ci.sh test-quick

# 3. 修复问题后继续开发
```

### 提交前检查
```bash
# 1. 完整测试
./scripts/local-ci.sh test-full

# 2. 检查不同配置
./scripts/local-ci.sh test-custom --no-trac-ik

# 3. 确认通过后提交
git add .
git commit -m "..."
git push
```

### 调试测试失败
```bash
# 1. 进入容器
./scripts/local-ci.sh shell

# 2. 手动重现问题
cd /workspace/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 3. 逐步调试
colcon build --packages-select trajectory_planning_v3
colcon test --packages-select trajectory_planning_v3
```

## 📈 性能优化

### 加速构建
- 使用Docker构建缓存
- 使用colcon编译缓存
- 并行编译（根据系统配置调整）

### 节省磁盘空间
```bash
# 定期清理
./scripts/local-ci.sh clean
docker system prune -a --volumes
```

## 🤝 与GitHub CI对比

| 功能 | 本地CI | GitHub CI |
|------|--------|-----------|
| **速度** | 快（本地资源） | 慢（网络+排队） |
| **环境** | 完全一致 | 官方环境 |
| **调试** | 容易 | 困难 |
| **成本** | 免费 | 有限额度 |
| **反馈** | 立即 | 需要推送 |

建议的工作流：
1. 🔄 **本地CI** - 日常开发和快速验证
2. 🚀 **GitHub CI** - 正式验证和部署

这样可以大大提高开发效率！