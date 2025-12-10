# Webots CrazyFlie 仿真设置指南

本文档指导如何在 Windows 上安装和使用 Webots 来运行 CrazyFlie 仿真。

## 环境信息

- **操作系统**: Windows (通过 WSL2 Ubuntu 24.04)
- **仿真器**: Webots (推荐 R2023b 或更新版本)
- **项目位置**:
  - WSL 路径: `/home/hanzel/24774_project/24774_ACSI_F25_ZephyFlyer/crazyflie-simulation`
  - Windows 路径: `\\wsl.localhost\Ubuntu\home\hanzel\24774_project\24774_ACSI_F25_ZephyFlyer\crazyflie-simulation`

## 安装步骤

### 1. 下载并安装 Webots

1. 访问 https://cyberbotics.com/
2. 点击 "Download" 下载最新版本
3. 选择 **Windows 64-bit** 版本
4. 运行安装程序，使用默认安装选项
5. 安装路径通常为：`C:\Program Files\Webots`

### 2. 验证安装

打开 Windows 开始菜单，搜索 "Webots" 并启动应用程序。如果能正常启动，说明安装成功。

## 运行 CrazyFlie 仿真

### 方法 1：通过 Webots GUI（推荐用于初次测试）

1. **启动 Webots**
   - 从开始菜单打开 Webots

2. **打开世界文件**
   - 点击菜单：`File` → `Open World...`
   - 在文件浏览器中，将路径栏粘贴：
     ```
     \\wsl.localhost\Ubuntu\home\hanzel\24774_project\24774_ACSI_F25_ZephyFlyer\crazyflie-simulation\simulator_files\webots\worlds
     ```
   - 选择 `crazyflie_world.wbt`

3. **运行仿真**
   - Webots 加载世界后，点击工具栏的"播放"按钮（或按 `Alt + R`）
   - CrazyFlie 应该会自动起飞并悬停

4. **键盘控制**
   - **点击 3D 视图窗口**使其获得焦点
   - 使用**方向键**控制无人机：
     - `↑` - 前进
     - `↓` - 后退
     - `←` - 向左
     - `→` - 向右
   - 查看控制台输出获取更多控制指令

### 方法 2：通过命令行（高级用户）

在 Windows PowerShell 或 CMD 中执行：

```powershell
# 基础键盘控制
webots "\\wsl.localhost\Ubuntu\home\hanzel\24774_project\24774_ACSI_F25_ZephyFlyer\crazyflie-simulation\simulator_files\webots\worlds\crazyflie_world.wbt"

# 墙壁跟随示例（高级）
webots "\\wsl.localhost\Ubuntu\home\hanzel\24774_project\24774_ACSI_F25_ZephyFlyer\crazyflie-simulation\simulator_files\webots\worlds\crazyflie_apartement.wbt"
```

**注意**: 确保 Webots 已添加到系统 PATH，或使用完整路径：
```powershell
"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" "\\wsl.localhost\Ubuntu\..."
```

## 可用的世界和控制器

### 世界文件

项目包含两个预配置的世界：

| 文件 | 描述 | 难度 |
|------|------|------|
| `crazyflie_world.wbt` | 基础键盘控制，空旷环境 | 简单 ⭐ |
| `crazyflie_apartement.wbt` | 室内公寓，墙壁跟随 | 中等 ⭐⭐ |

### 控制器

可用的 Python 控制器位于 `simulator_files/webots/controllers/`：

| 控制器 | 描述 |
|--------|------|
| `crazyflie_controller_py` | 基础 Python 控制器（键盘控制） |
| `crazyflie_controller_py_firmware_pid` | 使用固件 PID 参数的控制器 |
| `crazyflie_controller_py_wallfollowing` | 墙壁跟随控制器 |
| `crazyflie_controller_c` | C 语言实现的控制器 |
| `crazyflie_controller_py_socket` | Socket 通信控制器 |

## Python 依赖（可选）

### 基础键盘控制

**不需要额外依赖**，Webots 自带的 Python API 已包含所需模块：
- `controller.Robot`
- `controller.Motor`
- `controller.GPS`
- `controller.InertialUnit`
- `controller.Keyboard`

### 墙壁跟随（高级功能）

如果要使用 `crazyflie_apartement.wbt` 的墙壁跟随功能，需要配置 cflib：

1. 确认 cflib 已在 WSL2 环境中安装（已完成✓）
2. 在 Windows 上安装 Python 3 和 cflib（如需要）：
   ```powershell
   pip install cflib
   ```

## 故障排查

### 问题 1: 无法访问 WSL 文件

**症状**: 在文件浏览器中找不到 `\\wsl.localhost\` 路径

**解决方案**:
1. 确保 WSL2 正在运行：
   ```bash
   # 在 WSL 终端中
   wsl --status
   ```
2. 尝试替代路径格式：
   ```
   \\wsl$\Ubuntu-24.04\home\hanzel\24774_project\...
   ```
3. 或者将项目复制到 Windows 本地磁盘（如 `C:\Projects\`）

### 问题 2: CrazyFlie 不起飞

**可能原因**:
- 仿真未开始运行（点击播放按钮）
- 控制器脚本有错误（查看 Webots 控制台）

**解决方案**:
1. 检查 Webots 控制台是否有错误信息
2. 重启仿真：`Simulation` → `Revert`
3. 检查控制器是否正确加载

### 问题 3: 键盘控制无响应

**可能原因**: 3D 视图窗口未获得焦点

**解决方案**:
- 点击 3D 仿真窗口使其激活
- 控制台会显示控制说明

## 与 MuJoCo 仿真对比

您现在有两个仿真环境可以使用：

| 特性 | MuJoCo | Webots |
|------|--------|--------|
| **物理精度** | 高 ⭐⭐⭐⭐ | 中等 ⭐⭐⭐ |
| **官方支持** | 通用物理引擎 | CrazyFlie 官方 |
| **维护状态** | 活跃 | 不再维护 |
| **适用场景** | 控制算法开发 | 快速原型和演示 |
| **已有工作** | PID 控制器 | 键盘控制 |
| **学习曲线** | 陡峭 | 平缓 |
| **可视化** | 基础 | 丰富（室内环境） |

**建议**:
- 用 **Webots** 进行初步测试和演示
- 用 **MuJoCo** 进行精确的控制算法开发和调优
- 两者可以并行使用，互相验证

## 下一步

### 快速开始（5分钟）

1. 安装 Webots（如果还没有）
2. 打开 `crazyflie_world.wbt`
3. 点击播放按钮
4. 使用方向键控制无人机

### 进阶学习

1. **研究控制器代码**：
   ```bash
   # 在 WSL 中查看
   cd crazyflie-simulation/simulator_files/webots/controllers
   cat crazyflie_controller_py/crazyflie_controller_py.py
   ```

2. **尝试墙壁跟随**：
   - 打开 `crazyflie_apartement.wbt`
   - 按 `A` 键启用/禁用墙壁跟随

3. **修改控制器**：
   - 编辑控制器 Python 文件
   - 重新加载世界测试更改

4. **集成您的 PID 控制器**：
   - 将 MuJoCo 的 cascade_pid_improved.py 逻辑移植到 Webots 控制器
   - 对比两个仿真器的表现

## 相关文档

- 官方文档: https://www.bitcraze.io/documentation/repository/crazyflie-simulation/main/
- Webots 用户指南: https://cyberbotics.com/doc/guide/index
- CrazyFlie 固件: https://github.com/bitcraze/crazyflie-firmware

## 联系与支持

如遇问题，可参考：
- Webots 论坛: https://github.com/cyberbotics/webots/discussions
- Bitcraze 论坛: https://forum.bitcraze.io/

---

**最后更新**: 2025-11-03
**创建者**: Claude Code
