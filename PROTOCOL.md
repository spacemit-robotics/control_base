# 底盘通信协议

本文档描述当前底盘驱动所依赖的通信接口与运动学约定。

## 当前支持驱动

小核底盘控制

### RPMSG 驱动接口 (drv_rpmsg_esos)

- 控制设备节点：`/dev/rpmsg_ctrl0`
- 数据端点节点：`/dev/rpmsg0`
- 适用场景：Linux 侧与 ESOS 小核之间的底盘控制通信

### 配置说明

使用 `struct chassis_rpmsg_config` 进行初始化，典型字段如下：

```c
struct chassis_rpmsg_config config = {
    .base = {
        .type = CHASSIS_TYPE_DIFF_2WD,
        .wheel_diameter = 0.067f,
        .wheel_base = 0.33f,
        .wheel_track = 0.0f,
        .max_speed = 1.0f,
        .max_angular = 3.14f,
    },
    .ctrl_dev = "/dev/rpmsg_ctrl0",
    .ept_dev = "/dev/rpmsg0",
};
```

> `drv_rpmsg_esos` 的具体消息定义和端点交互细节由 ESOS 固件侧协议决定；本目录主要约定 Linux 侧设备节点、初始化方式以及运动学换算。

## 速度单位转换

### 轮速 → 线速度

```text
线速度 (m/s) = 轮速 (rev/s) × π × 轮径 (m)
```

**示例** (轮径 67mm):

```text
v = 0.5 rev/s × π × 0.067 m ≈ 0.105 m/s
```

### 线速度 → 轮速

```text
轮速 (rev/s) = 线速度 (m/s) / (π × 轮径)
```

## 差速运动学

### 左右轮速计算

```text
v_left  = vx - wz × wheel_base / 2
v_right = vx + wz × wheel_base / 2
```

### 里程计计算

```text
v  = (v_right + v_left) / 2      # 线速度
w  = (v_right - v_left) / wheel_base  # 角速度

# 位姿更新 (dt = 时间间隔)
yaw += w × dt
x   += v × cos(yaw) × dt
y   += v × sin(yaw) × dt
```

## 待扩展协议

后续如恢复或新增驱动，可在本文档中补充对应协议定义，例如：

|驱动|底盘类型|状态|
|---|---|---|
|rpmsg_mecanum|四轮麦克纳姆|TODO|
|rpmsg_omni|全向轮|TODO|
|can_diff|CAN 差速|TODO|
