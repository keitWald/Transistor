# Plane 首次训练工程

本目录是从原开源项目中整理出的独立训练与 sim2sim 验证工程，保留了原来的 `plane/`、`mujoco/` 同级结构。

## 基线来源

训练环境与配置取自：

```text
plane/logs/wheel_legged/linx_scales=3.0pd=51.50.25_randization=normal_new
```

选择原因：该实验为平地基础运动、从零训练配置（`resume = False`），没有使用恢复训练、台阶或高难随机地形，适合作为首次训练起点。为了保证版本匹配，同时采用了该实验保存的：

- `legged_robot.py`
- `legged_robot_config.py`
- `wheel_legged_config.py`

原配置中的服务器绝对 URDF 路径已改为本工程内部路径。

## 环境要求

- Ubuntu 22.04（原项目使用环境）
- Python 3.8
- NVIDIA GPU、驱动及兼容的 CUDA/PyTorch
- Isaac Gym Preview 4
- `pynput`（使用键盘控制的 `play.py` 时需要）

先安装 Isaac Gym，再安装本工程：

```bash
cd ~/isaacgym/python
pip install -e .

cd /path/to/plane_first_training/plane
pip install -e .
pip install pynput
```

## 从零开始训练

必须在本目录下执行：

```bash
cd /path/to/plane_first_training/plane
python wheel_legged_gym/scripts/train.py --task=wheel_legged --headless
```

训练结果会生成在：

```text
logs/wheel_legged/<日期时间>/
```

其中 `model_*.pt` 是检查点，TensorBoard 日志也保存在该实验目录中。

查看训练曲线：

```bash
tensorboard --logdir logs --port 8080
```

## 查看训练成果

默认查看最新一次训练的最新检查点：

```bash
python wheel_legged_gym/scripts/play.py --task=wheel_legged
```

指定实验目录和检查点：

```bash
python wheel_legged_gym/scripts/play.py \
  --task=wheel_legged \
  --load_run=<实验目录名> \
  --checkpoint=<检查点编号>
```

`play.py` 使用键盘控制：`W/S` 前进或后退，按住 `A/D` 转向，`E` 停止，`X/C` 调整高度，`Q` 或 `Esc` 退出。

## MuJoCo 中查看与验证

MuJoCo 部分需要 Python 3.9、MuJoCo 3.2.7，以及 `onnxruntime`、`numpy`、`torch`、`pynput`：

```bash
pip install mujoco==3.2.7 onnxruntime numpy torch pynput
```

训练结束后，先在 `plane/` 下导出自己的 ONNX：

```bash
cd /path/to/plane_first_training/plane
python export_onnx/export_onnx.py \
  --load_run=<实验目录名> \
  --checkpoint=<检查点编号> \
  --out=../mujoco/actor/yuntai/first_training.onnx
```

然后从工程根目录指定该策略进行 MuJoCo 验证：

```bash
cd /path/to/plane_first_training
python mujoco/python_tools/onnx_mj_binglian.py \
  --onnx=mujoco/actor/yuntai/first_training.onnx
```

`mujoco/actor/yuntai/` 中开源项目自带的两个 ONNX 是旧实验演示模型，不是本工程首次训练产生的模型。

需要注意：开源 MuJoCo 模型包含气弹簧机构和对应作用力，而所选 Isaac Gym 首训环境没有气弹簧力模型。因此这里可以用于成果观察和 sim2sim 差异测试，但二者目前并非完全等效动力学。

## 关键文件

```text
plane_first_training/
├── plane/
│   ├── resources/robots/infantry_V4/   # 当前机器人 URDF 与网格
│   ├── wheel_legged_gym/
│   │   ├── envs/base/legged_robot.py   # 环境、观测、奖励和控制逻辑
│   │   ├── envs/base/legged_robot_config.py
│   │   ├── envs/wheel_legged/wheel_legged_config.py
│   │   ├── rsl_rl/                      # PPO、网络、runner 和存储
│   │   ├── scripts/train.py             # 首次训练入口
│   │   ├── scripts/play.py              # Isaac Gym 成果查看
│   │   └── scripts/play_export.py
│   ├── export_onnx/                     # ONNX 导出脚本
│   ├── logs/wheel_legged/               # 新训练输出目录
│   ├── setup.py
│   └── pyproject.toml
├── mujoco/
│   ├── actor/yuntai/                    # ONNX 策略
│   ├── assert_now/                      # MuJoCo XML、机构和网格资源
│   └── python_tools/                    # sim2sim 查看脚本
└── FIRST_TRAINING_GUIDE.md
```

不要把原项目 `plane/logs` 中的源码快照当作模型权重；原开源目录没有包含 `model_*.pt`，必须先完成训练才能查看策略成果。
