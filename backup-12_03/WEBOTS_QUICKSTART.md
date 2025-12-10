# Webots CrazyFlie 快速开始指南

## 🚀 5分钟快速启动

### 第一步：下载并安装 Webots

1. **在 Windows 浏览器中打开**：https://cyberbotics.com/

2. **点击 "Download"**，选择：
   - **Webots R2023b** 或更新版本
   - **Windows 64-bit** 版本

3. **运行安装程序**，使用默认选项
   - 安装路径通常：`C:\Program Files\Webots`
   - 等待安装完成（约 5-10 分钟）

### 第二步：打开 CrazyFlie 仿真

1. **启动 Webots**
   - Windows 开始菜单 → 搜索 "Webots" → 点击打开

2. **打开世界文件**
   - 在 Webots 中：`File` → `Open World...`
   - 在文件路径栏粘贴以下路径（复制整行）：
   ```
   \\wsl.localhost\Ubuntu\home\hanzel\24774_project\24774_ACSI_F25_ZephyFlyer\crazyflie-simulation\simulator_files\webots\worlds
   ```
   - 选择 `crazyflie_world.wbt`
   - 点击 "打开"

   **如果路径无法访问**，尝试这个替代路径：
   ```
   \\wsl$\Ubuntu-24.04\home\hanzel\24774_project\24774_ACSI_F25_ZephyFlyer\crazyflie-simulation\simulator_files\webots\worlds
   ```

3. **运行仿真**
   - 点击工具栏的 **▶️ 播放按钮**（或按 `Alt+R`）
   - CrazyFlie 会自动起飞并悬停在空中

### 第三步：键盘控制

1. **激活 3D 窗口**
   - 用鼠标 **点击 3D 仿真窗口** 使其获得焦点
   - 窗口边框会高亮显示

2. **使用键盘控制 CrazyFlie**：

| 按键 | 功能 |
|------|------|
| **↑** | 向前飞 |
| **↓** | 向后飞 |
| **←** | 向左飞 |
| **→** | 向右飞 |
| **W** | 上升 |
| **S** | 下降 |
| **Q** | 逆时针旋转（yaw left） |
| **E** | 顺时针旋转（yaw right） |

3. **观察控制台输出**
   - Webots 窗口底部有控制台
   - 会显示控制说明和实时状态

### 第四步：尝试更复杂的场景（可选）

1. **停止当前仿真**：点击 ⏸️ 暂停按钮

2. **加载公寓场景**：
   - `File` → `Open World...`
   - 选择 `crazyflie_apartement.wbt`（注意拼写：apartement）

3. **运行墙壁跟随**：
   - 点击播放
   - 按 **A** 键启用/禁用墙壁跟随功能
   - CrazyFlie 会使用距离传感器沿墙飞行

---

## 🎯 期望效果

### 基础场景（crazyflie_world.wbt）
- ✓ CrazyFlie 自动起飞
- ✓ 悬停在约 1 米高度
- ✓ 方向键控制流畅响应
- ✓ 没有控制台错误

### 公寓场景（crazyflie_apartement.wbt）
- ✓ CrazyFlie 在室内环境起飞
- ✓ 按 A 键激活墙壁跟随
- ✓ 自主沿墙飞行，避障

---

## ❗ 常见问题

### 问题 1: 找不到 WSL 路径

**症状**：文件浏览器显示 "网络路径无法访问"

**解决方案**：
1. 确保 WSL 正在运行（打开任意 WSL 终端窗口）
2. 尝试替代路径：`\\wsl$\Ubuntu-24.04\...`
3. 或者在 Windows 中复制整个项目文件夹：
   ```powershell
   # 在 PowerShell 中
   cp -r \\wsl.localhost\Ubuntu\home\hanzel\24774_project\24774_ACSI_F25_ZephyFlyer\crazyflie-simulation C:\Users\YourUsername\Desktop\
   ```
   然后从 `C:\Users\...\crazyflie-simulation\simulator_files\webots\worlds` 打开

### 问题 2: CrazyFlie 不起飞

**可能原因**：
- 仿真未启动（播放按钮未点击）
- 控制器加载失败

**解决方案**：
1. 检查控制台是否有红色错误信息
2. 点击 ⏹️ 停止，然后重新点击 ▶️ 播放
3. 或者：`Simulation` → `Revert` 重新加载世界

### 问题 3: 键盘控制无响应

**原因**：3D 窗口未激活

**解决方案**：
- 用鼠标点击 3D 仿真窗口
- 确保窗口边框高亮
- 再次尝试按键

### 问题 4: 控制台显示 Python 错误

**错误示例**：`ModuleNotFoundError: No module named 'controller'`

**这是正常的！**
- Webots 使用自己的 Python 环境
- 不需要安装额外的 Python 包
- 如果看到此错误，说明 Webots 配置有问题，重新安装 Webots

---

## 📊 与 MuJoCo 对比测试

测试完 Webots 后，您可以对比两个仿真器：

| 观察项 | MuJoCo | Webots |
|--------|--------|--------|
| **起飞稳定性** | ? | ? |
| **悬停精度** | ? | ? |
| **响应速度** | ? | ? |
| **控制难度** | ? | ? |

### 重要发现：Webots 的 PID 参数

我发现 Webots 控制器使用的 PID 参数（来自 Bitcraze 官方）：

```python
# 速度控制（外环）
kp_vel_xy: 2.0
kd_vel_xy: 0.5

# 姿态控制（内环）
kp_att_rp: 0.5  # roll/pitch
kd_att_rp: 0.1
kp_att_y: 1.0   # yaw
kd_att_y: 0.5

# 高度控制
kp_z: 10.0
ki_z: 5.0
kd_z: 5.0
```

**对比您的 MuJoCo 参数**：
- 您的 X/Y 位置 Kp=1.5，Webots 速度 Kp=2.0（相近）
- 您的姿态 Kp=0.02，Webots Kp=0.5（**差异大！**）
- 您的 Z Kp=8.0，Webots Kp=10.0（相近）

**这可能是您 MuJoCo PID 不稳定的线索！**
- Webots 的姿态控制增益更高
- 可能需要调整您的内环 PID 参数

---

## 📁 项目文件位置

**Windows 路径**：
```
\\wsl.localhost\Ubuntu\home\hanzel\24774_project\24774_ACSI_F25_ZephyFlyer\crazyflie-simulation
```

**WSL 路径**：
```
/home/hanzel/24774_project/24774_ACSI_F25_ZephyFlyer/crazyflie-simulation
```

**关键文件**：
- 世界文件：`simulator_files/webots/worlds/*.wbt`
- 控制器：`simulator_files/webots/controllers/crazyflie_controller_py/`
- PID 代码：`controllers_shared/python_based/pid_controller.py`

---

## 🔍 下一步探索

### 1. 分析 Webots PID 控制器（推荐）
```bash
# 在 WSL 中查看
cd crazyflie-simulation/controllers_shared/python_based
cat pid_controller.py
```

### 2. 修改控制参数
- 编辑 `pid_controller.py` 中的 `gains` 字典
- 重新加载世界测试效果

### 3. 将 MuJoCo 控制器移植到 Webots
- 创建新的控制器文件
- 使用您的 cascade PID 逻辑
- 在 Webots 中测试

### 4. 对比两个仿真器
- 记录各项性能指标
- 分析差异原因
- 改进控制算法

---

## 📞 获取帮助

测试完成后，请告诉我：
1. ✅ 安装是否成功？
2. ✅ CrazyFlie 能否正常起飞？
3. ✅ 键盘控制是否响应？
4. ❓ 遇到了什么问题？

我可以帮您：
- 解决技术问题
- 分析控制器代码
- 对比 Webots 和 MuJoCo
- 改进您的 PID 参数

---

**最后更新**: 2025-11-03
**创建者**: Claude Code
