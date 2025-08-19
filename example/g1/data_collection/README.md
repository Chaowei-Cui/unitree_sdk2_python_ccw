# G1机器人数据收集器

这是一个用于收集G1机器人所有传感器数据的Python脚本。

## 功能特性

- 收集所有29个电机的状态数据（位置、速度、扭矩、温度等）
- 收集IMU数据（姿态角、角速度、加速度、四元数）
- 收集机器人模式信息
- 数据以HDF5格式保存，包含完整的元数据
- 实时数据收集，采样频率500Hz（2ms间隔）
- 自动保持机器人稳定，不产生运动

## 安装依赖

```bash
pip install h5py numpy
```

## 使用方法

### 基本使用

```bash
cd example/g1/data_collection
python g1_data_collector.py
```

### 带网络配置使用

```bash
python g1_data_collector.py <network_interface>
```

例如：
```bash
python g1_data_collector.py en0
```

## 数据收集流程

1. **初始化连接**：连接到机器人并初始化通信
2. **等待就绪**：等待机器人状态信息就绪
3. **开始收集**：启动数据收集循环
4. **保存数据**：将收集的数据保存为HDF5格式

## 输出文件

数据将保存在 `g1_data_collection/` 目录下，文件名格式为：
```
g1_data_YYYYMMDD_HHMMSS.h5
```

## HDF5文件结构

```
g1_data_YYYYMMDD_HHMMSS.h5
├── metadata/
│   ├── robot_type: 'G1'
│   ├── total_records: 数据记录总数
│   ├── collection_date: 收集日期
│   └── sample_rate: 采样频率(500Hz)
└── data/
    ├── timestamp: Unix时间戳
    ├── elapsed_time: 相对时间
    ├── counter: 计数器
    ├── imu_rpy_x/y/z: IMU姿态角(弧度)
    ├── imu_gyro_x/y/z: IMU角速度(rad/s)
    ├── imu_accel_x/y/z: IMU加速度(m/s²)
    ├── imu_quaternion_w/x/y/z: IMU四元数
    ├── motor_0_q/dq/tau/mode/temperature: 电机0的状态
    ├── motor_1_q/dq/tau/mode/temperature: 电机1的状态
    ├── ... (共29个电机)
    ├── mode_machine: 机器人模式
    └── mode_pr: 控制模式
```

## 数据字段说明

### IMU数据
- `imu_rpy_x/y/z`: 横滚角、俯仰角、偏航角（弧度）
- `imu_gyro_x/y/z`: 角速度（rad/s）
- `imu_accel_x/y/z`: 加速度（m/s²）
- `imu_quaternion_w/x/y/z`: 四元数表示姿态

### 电机数据
- `motor_X_q`: 电机X的位置（弧度）
- `motor_X_dq`: 电机X的速度（rad/s）
- `motor_X_tau`: 电机X的扭矩（Nm）
- `motor_X_mode`: 电机X的模式
- `motor_X_temperature`: 电机X的温度

### 控制信息
- `mode_machine`: 机器人运行模式
- `mode_pr`: 控制模式（PR模式或AB模式）

## 安全注意事项

⚠️ **重要警告**：
- 运行脚本前请确保机器人周围没有障碍物
- 机器人将保持静止状态，不会产生运动
- 建议在安全环境中运行

## 故障排除

### 连接问题
- 检查网络接口配置
- 确保机器人已开机并连接到网络
- 检查防火墙设置

### 数据收集问题
- 确保机器人处于正确的模式
- 检查是否有其他程序占用机器人控制权

## 示例数据分析

收集完数据后，可以使用以下代码读取和分析数据：

```python
import h5py
import numpy as np
import matplotlib.pyplot as plt

# 读取数据
with h5py.File('g1_data_YYYYMMDD_HHMMSS.h5', 'r') as hf:
    # 读取IMU数据
    rpy_x = hf['data/imu_rpy_x'][:]
    rpy_y = hf['data/imu_rpy_y'][:]
    rpy_z = hf['data/imu_rpy_z'][:]
    
    # 读取电机数据
    motor_0_q = hf['data/motor_0_q'][:]
    
    # 读取时间信息
    elapsed_time = hf['data/elapsed_time'][:]

# 绘制IMU姿态角
plt.figure(figsize=(12, 8))
plt.subplot(3, 1, 1)
plt.plot(elapsed_time, np.rad2deg(rpy_x))
plt.ylabel('Roll (deg)')
plt.title('IMU Attitude Angles')

plt.subplot(3, 1, 2)
plt.plot(elapsed_time, np.rad2deg(rpy_y))
plt.ylabel('Pitch (deg)')

plt.subplot(3, 1, 3)
plt.plot(elapsed_time, np.rad2deg(rpy_z))
plt.ylabel('Yaw (deg)')
plt.xlabel('Time (s)')

plt.tight_layout()
plt.show()
``` 