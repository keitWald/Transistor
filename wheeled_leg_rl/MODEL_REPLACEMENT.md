# 新机器人模型替换清单

## 1. 适用范围

当前环境假设机器人具有：

- 左腿两个转动关节；
- 左侧一个驱动轮；
- 右腿两个转动关节；
- 右侧一个驱动轮；
- 合计六个可动自由度。

如果新机器人不满足该拓扑，不要只修改 `num_actions`。需要重新设计观测、动作解释、奖励、部署控制器和网络输入输出。

## 2. 放入模型资源

推荐目录：

```text
resources/robots/<robot_name>/
├── urdf/<robot_name>.urdf
└── meshes/
```

检查 URDF 中：

- 网格路径为相对路径；
- 质量和惯量非零且单位正确；
- 长度单位是米；
- 角度单位是弧度；
- 左右关节轴方向明确；
- 关节上下限合理；
- `effort` 和 `velocity` 限制真实可信；
- 轮子关节为连续关节或具有足够大的位置范围；
- 碰撞模型不过度复杂；
- 基座坐标系符合右手系和 Z 轴向上约定。

## 3. 新建 RobotProfile

不要直接覆盖参考配置。复制 `INFANTRY_V4`，创建例如：

```python
MY_ROBOT = RobotProfile(
    name="my_robot",
    urdf="{CUSTOM_WHEEL_LEGGED_GYM_ROOT_DIR}/resources/robots/my_robot/urdf/my_robot.urdf",
    roles=JointRoles(
        left_hip="...",
        left_knee="...",
        left_wheel="...",
        right_hip="...",
        right_knee="...",
        right_wheel="...",
    ),
    default_joint_angles={...},
    kinematics=LegKinematics(...),
    initial_base_position=(0.0, 0.0, ...),
    stiffness={...},
    damping={...},
    position_action_scale=...,
    velocity_action_scale=...,
)
```

最后修改：

```python
ACTIVE_PROFILE = MY_ROBOT
```

## 4. 腿部角度符号

虚拟腿长根据两连杆正运动学计算。Profile 中可以分别设置：

- `left_hip_sign`
- `left_knee_sign`
- `right_hip_sign`
- `right_knee_sign`
- `left_knee_offset`
- `right_knee_offset`

必须用几个已知姿态人工验证：

1. 读取 URDF 对应关节角；
2. 手工计算或可视化实际轮心位置；
3. 比较环境得到的 `L0`；
4. 确认伸腿时 `L0` 增大，收腿时 `L0` 减小；
5. 确认左右腿相同物理姿态得到相同 `L0` 和镜像 `theta0`。

## 5. 默认姿态与初始高度

默认姿态必须满足：

- 不超过关节位置限制；
- 机器人生成时轮子接近地面但不穿透；
- 腿部没有接近奇异伸直；
- 左右质心基本对称；
- PD 打开后不会产生危险的初始冲击。

先在固定基座或单环境中检查，再开启大量并行环境。

## 6. 控制参数

需要从实际机器人控制器或辨识结果确认：

- 腿部 `Kp`、`Kd`；
- 轮子速度环等效增益；
- 电机峰值与连续力矩；
- 最大轮速；
- 控制周期；
- 传感器滤波和延迟；
- 通信延迟；
- 电机方向和减速比。

训练使用的动作缩放不应让大多数动作长期触及位置、速度或力矩极限。

## 7. 静态检查

```bash
python tools/validate_robot_profile.py
python tools/static_check.py
```

该检查不需要 Isaac Gym。它只能发现结构和语法问题，不能验证动力学参数是否合理。

## 8. 训练服务器上的分阶段验证

推荐顺序：

1. 单环境、固定基座、关闭随机化；
2. 检查关节方向和 PD 响应；
3. 单环境、释放基座、零速度指令；
4. 检查观测数值和奖励各分量；
5. 64～256 个环境短时间训练；
6. 可视化策略是否通过异常动作投机；
7. 恢复完整环境数和随机化；
8. 导出 ONNX 做 sim2sim；
9. 最后才连接真实硬件。

## 9. 真机前必须增加的安全层

本工程的部署控制器不是完整的硬件安全系统。至少应增加：

- 物理急停；
- 控制进程看门狗；
- 通信超时自动卸力；
- 独立于神经网络的力矩、速度和位置限制；
- 机体姿态越界保护；
- 电池、电机温度和驱动器故障检查；
- 首次测试时的悬挂架或保护绳；
- 逐步提升力矩上限的测试流程。
