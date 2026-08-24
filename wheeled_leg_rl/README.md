# Custom Wheel-Legged RL

这是一个面向“两条二自由度腿 + 两个驱动轮”机器人的强化学习控制工程模板。当前使用 `infantry_V4` 作为参考模型，后续可通过替换 URDF 和 `RobotProfile` 迁移到新机器人。

本目录只负责代码与模型文件管理。本地不要求安装 Isaac Gym、PyTorch、MuJoCo 或 ONNX Runtime，也不执行训练和仿真。

## 1. 当前实现

- Isaac Gym GPU 并行环境；
- 25 维本体观测和 5 帧观测历史；
- 三维历史速度估计 Encoder；
- 不对称 Actor-Critic；
- PPO + GAE；
- 四个腿位置动作和两个轮速动作；
- 运行时按关节名称解析角色，不依赖 URDF 声明顺序；
- 地形与指令课程学习；
- 动力学随机化；
- JIT/ONNX 导出；
- 带关节重排和 PD 控制的通用 ONNX 部署控制器；
- 无第三方依赖的 URDF/Profile 静态校验。

## 2. 目录结构

```text
custom_wheel_legged_rl/
├── custom_wheel_legged_gym/
│   ├── envs/
│   │   ├── base/                    通用仿真环境和任务配置
│   │   └── wheel_legged/
│   │       ├── robot_profile.py     机器人专用参数入口
│   │       └── wheel_legged_config.py
│   ├── rsl_rl/                      PPO、网络和经验缓存
│   ├── scripts/
│   │   ├── train.py
│   │   └── play.py
│   └── utils/
├── deployment/
│   └── policy_controller.py         ONNX 观测和力矩控制器
├── resources/robots/                URDF、网格和机器人资源
├── tools/
│   ├── validate_robot_profile.py
│   ├── static_check.py
│   └── export_onnx.py
├── MODEL_REPLACEMENT.md
└── setup.py
```

## 3. 当前参考模型

当前激活的配置位于：

```text
custom_wheel_legged_gym/envs/wheel_legged/robot_profile.py
```

它定义：

- 六个关节的语义角色；
- 默认关节角；
- 左右腿角度符号约定；
- 两段腿长；
- URDF 路径；
- 初始机体位置；
- PD 增益；
- 腿位置动作与轮速动作缩放。

环境启动时会根据关节名称解析实际 DOF 索引。如果 Profile 中的关节不存在、存在额外可动关节，或者动作数量与 DOF 数量不一致，程序会立即报错。

## 4. 本地静态检查

不安装任何强化学习依赖也可以执行：

```bash
python tools/validate_robot_profile.py
python tools/static_check.py
```

第一个命令检查 URDF 与机器人 Profile，第二个命令解析全部 Python 文件并报告语法错误。这两个命令不会启动训练或仿真。

## 5. 训练服务器环境

推荐环境与原工程保持一致：

- Ubuntu 22.04；
- Python 3.8；
- NVIDIA GPU 和兼容驱动；
- CUDA 兼容的 PyTorch；
- Isaac Gym Preview 4。

在训练服务器上安装 Isaac Gym 后：

```bash
cd custom_wheel_legged_rl
pip install -e .
python tools/validate_robot_profile.py
python custom_wheel_legged_gym/scripts/train.py --task=wheel_legged --headless
```

测试指定模型：

```bash
python custom_wheel_legged_gym/scripts/play.py \
  --task=wheel_legged \
  --resume \
  --experiment_name=custom_wheel_legged \
  --load_run=<run目录> \
  --checkpoint=<编号>
```

训练日志默认保存在：

```text
logs/custom_wheel_legged/
```

每个 checkpoint 会额外保存：

- Isaac Gym 实际关节顺序；
- 腿与轮子的运行时索引；
- 默认关节角；
- 观测和动作维数；
- 策略周期；
- 观测缩放；
- 动作缩放；
- 标称 PD 增益；
- URDF 力矩限制。

这些元数据用于保证 ONNX 部署时的观测和动作顺序一致。

## 6. ONNX 导出

在训练机器上执行：

```bash
python tools/export_onnx.py \
  logs/custom_wheel_legged/<run目录>/model_1000.pt \
  exported/policy.onnx
```

输出：

```text
exported/policy.onnx
exported/policy.json
```

JSON 文件记录部署控制器需要的关节顺序、缩放系数、PD 参数和维数。不要只复制 ONNX 而遗漏 JSON。

## 7. 部署控制器

`deployment.PolicyController` 接收按名称组织的关节状态，因此硬件驱动的数组顺序不需要与 Isaac Gym 相同。

```python
from deployment import PolicyController

controller = PolicyController("exported/policy.onnx")

torques = controller.step(
    base_angular_velocity=[wx, wy, wz],
    projected_gravity=[gx, gy, gz],
    command=[target_vx, target_yaw_rate, target_height],
    joint_position={
        "lf0_Joint": q_lf0,
        "lf1_Joint": q_lf1,
        "l_wheel_Joint": q_lw,
        "rf0_Joint": q_rf0,
        "rf1_Joint": q_rf1,
        "r_wheel_Joint": q_rw,
    },
    joint_velocity={
        "lf0_Joint": dq_lf0,
        "lf1_Joint": dq_lf1,
        "l_wheel_Joint": dq_lw,
        "rf0_Joint": dq_rf0,
        "rf1_Joint": dq_rf1,
        "r_wheel_Joint": dq_rw,
    },
)
```

返回值是 `{关节名: 力矩}`。真正连接电机前必须增加急停、通信超时、姿态保护、速度保护和独立的硬件力矩限制。

## 8. 更换新机器人

完整步骤见 [MODEL_REPLACEMENT.md](MODEL_REPLACEMENT.md)。原则是：

1. 放入新 URDF 和网格；
2. 新建一个 `RobotProfile`；
3. 修改 `ACTIVE_PROFILE`；
4. 执行静态校验；
5. 核对腿部运动学符号；
6. 在训练服务器上进行小规模可视化检查；
7. 确认无误后再开始大规模训练。

如果新机器人仍然是四个腿关节和两个轮子，通常不需要修改 PPO。如果自由度数量或拓扑发生变化，则必须同步修改观测、动作、网络维数和部署协议。

## 9. 来源说明

该模板延续当前仓库中基于 NVIDIA Isaac Gym、`legged_gym`、`rsl_rl` 和 Wheel-Legged-Gym 的实现结构，并针对机器人参数迁移、关节顺序和 ONNX 部署一致性进行了整理。
