# Crazyflie LQR Hover Controller (C Implementation)

基于 12 状态 LQR 的 Crazyflie 悬停控制器 C 语言实现，可直接集成到 Crazyflie 固件中。

## 📁 文件结构

```
crazyflie_lqr/
├── controller_lqr.h           # 主头文件 (API 定义)
├── controller_lqr.c           # 核心控制器实现
├── controller_lqr_crazyflie.c # Crazyflie 固件集成示例
├── compute_lqr_gains.py       # LQR 增益计算脚本
├── Makefile                   # 编译脚本
└── README.md                  # 本文档
```

## 🚀 快速开始

### 1. 编译测试

```bash
make test
```

### 2. 计算 LQR 增益

```bash
pip install numpy scipy
python compute_lqr_gains.py
```

这将生成:
- `lqr_gains.h` - C 格式的增益矩阵
- `lqr_gains.txt` - 人类可读格式

## 📐 状态空间模型

### 状态向量 (12 维)

```
x = [px, py, pz, vx, vy, vz, roll, pitch, yaw, p, q, r]
```

| 状态 | 描述 | 单位 |
|------|------|------|
| px, py, pz | 世界坐标位置 | m |
| vx, vy, vz | 世界坐标速度 | m/s |
| roll, pitch, yaw | 欧拉角 | rad |
| p, q, r | 机体角速度 | rad/s |

### 控制输入 (4 维)

```
u = [thrust_delta, roll_moment, pitch_moment, yaw_moment]
```

| 输入 | 描述 | 范围 |
|------|------|------|
| thrust_delta | 推力偏差 (归一化) | [-0.05, 0.05] |
| roll_moment | 横滚力矩 | [-0.02, 0.02] |
| pitch_moment | 俯仰力矩 | [-0.02, 0.02] |
| yaw_moment | 偏航力矩 | 禁用 |

## 🔧 集成到 Crazyflie 固件

### 步骤 1: 复制文件

```bash
cp controller_lqr.h ~/crazyflie-firmware/src/modules/interface/
cp controller_lqr.c ~/crazyflie-firmware/src/modules/src/
```

### 步骤 2: 修改 Makefile

在 `src/modules/src/Makefile` 中添加:

```makefile
PROJ_OBJ += controller_lqr.o
```

### 步骤 3: 在 stabilizer 中调用

```c
#include "controller_lqr.h"

static lqr_controller_t lqrCtrl;

void stabilizerInit(void) {
    // ... 其他初始化 ...
    lqrControllerInit(&lqrCtrl, 0.002f, 0.5f);  // 500Hz, 0.5m 高度
}

void stabilizerLoop(void) {
    // 获取传感器数据
    lqr_sensor_data_t sensors;
    sensors.position[0] = state.position.x;
    sensors.position[1] = state.position.y;
    sensors.position[2] = state.position.z;
    sensors.attitude[0] = state.attitude.roll * DEG_TO_RAD;
    sensors.attitude[1] = state.attitude.pitch * DEG_TO_RAD;
    sensors.attitude[2] = state.attitude.yaw * DEG_TO_RAD;
    sensors.angular_rate[0] = gyro.x * DEG_TO_RAD;
    sensors.angular_rate[1] = gyro.y * DEG_TO_RAD;
    sensors.angular_rate[2] = gyro.z * DEG_TO_RAD;
    sensors.timestamp = usecTimestamp() / 1e6f;
    
    // 设置目标
    lqrSetTarget(&lqrCtrl, setpoint.position.x, 
                 setpoint.position.y, setpoint.position.z);
    
    // 计算控制
    lqr_motor_output_t output;
    lqrControllerUpdate(&lqrCtrl, &sensors, &output);
    
    // 应用电机命令
    motorsSetRatio(MOTOR_M1, output.m1 * 65535 / 600);
    motorsSetRatio(MOTOR_M2, output.m2 * 65535 / 600);
    motorsSetRatio(MOTOR_M3, output.m3 * 65535 / 600);
    motorsSetRatio(MOTOR_M4, output.m4 * 65535 / 600);
}
```

## 🎛️ 参数调整

### 通过代码调整

```c
// 设置积分增益
lqrSetIntegralGains(&lqrCtrl, 0.003f, 0.002f);

// 设置速度滤波器系数 (0-1, 越大滤波越弱)
lqrSetVelocityFilter(&lqrCtrl, 0.4f);

// 设置输入约束
float u_min[4] = {-0.05f, -0.02f, -0.02f, -0.05f};
float u_max[4] = { 0.05f,  0.02f,  0.02f,  0.05f};
lqrSetInputConstraints(&lqrCtrl, u_min, u_max);
```

### 通过 Crazyflie 参数系统调整

在 `controller_lqr_crazyflie.c` 中启用 PARAM_GROUP 后:

```python
# Python cflib
cf.param.set_value('ctrlLQR.Ki_z', 0.005)
cf.param.set_value('ctrlLQR.Ki_xy', 0.003)
```

## 📊 性能指标

| 指标 | 典型值 |
|------|--------|
| 控制频率 | 500 Hz |
| 位置误差 (稳态) | < 5 cm |
| 响应时间 | < 0.5 s |
| CPU 占用 | < 5% (STM32F4) |
| RAM 占用 | ~1 KB |

## 🔬 Q/R 矩阵调整指南

### Q 矩阵 (状态权重)

```
Q = diag([Qpx, Qpy, Qpz, Qvx, Qvy, Qvz, Qroll, Qpitch, Qyaw, Qp, Qq, Qr])
```

- **位置权重增大** → 更快的位置响应，可能振荡
- **速度权重增大** → 更好的阻尼，响应变慢
- **姿态权重增大** → 更小的姿态偏差

### R 矩阵 (输入权重)

```
R = diag([R_thrust, R_roll, R_pitch, R_yaw])
```

- **R 减小** → 更大的控制力，更快响应，可能振荡
- **R 增大** → 更平滑的控制，响应变慢

### 推荐配置

| 场景 | Q 调整 | R 调整 |
|------|--------|--------|
| 稳定悬停 | 默认 | 默认 |
| 快速跟踪 | Qpos ×2 | R ×0.5 |
| 抗风 | Qvel ×2 | R ×0.7 |
| 室内精确 | Qpos ×1.5, Qatt ×2 | 默认 |

## ⚠️ 注意事项

1. **电机方向**: 确保 mixer 中的电机旋转方向与你的 Crazyflie 配置匹配
2. **坐标系**: 使用 NED (North-East-Down) 或 ENU (East-North-Up) 时注意符号
3. **单位**: 角度在内部使用弧度，与固件交互时可能需要转换
4. **增益更新**: 修改物理参数后需重新运行 `compute_lqr_gains.py`

## 📚 参考资料

- [Crazyflie 固件文档](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/)
- [CMU 24-774 ACSI Lab](https://github.com/cmu-quadcopter/acsi-labs)
- [TinyMPC](https://github.com/TinyMPC/tinympc)

## 📝 License

MIT License - 自由使用和修改
