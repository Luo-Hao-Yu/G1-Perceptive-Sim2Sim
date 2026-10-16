# sim2sim

**中文** | [English](README_EN.md)

面向 Unitree G1 29DoF 的 MuJoCo Sim2Sim 验证示例。本仓库只负责加载已有 ONNX 策略并在 MuJoCo 中运行。

## 范围

包含：

- `stand` 和 `parkour` 两种运行入口。
- MuJoCo 场景、G1 29DoF 模型和 ONNX 文件放置目录。
- 键盘或 Xbox/XInput 手柄速度指令。
- 可替换的 actor 和 depth encoder ONNX 路径。

不包含：

- 训练代码。
- reward、domain randomization、训练日志或 checkpoint。
- 真机部署代码。

## 环境依赖

需要 Python 3.10 或更高版本。如果没有可复用的训练环境，可以在项目目录创建虚拟环境并安装全部依赖：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

每次打开新终端后，先执行 `source .venv/bin/activate`。

如果终端的 `PYTHONPATH` 引入了 ROS 或其他环境的包，可使用 `python -E` 替代下面命令中的 `python`/`python3`，忽略 Python 环境变量，确保使用当前虚拟环境的依赖。

要复现本次验证的依赖版本，可将安装命令中的 `requirements.txt` 替换为 `requirements-lock.txt`（在 Linux x86_64、Python 3.11 环境验证）。

建议直接复用你已经用于训练和策略导出的 conda 环境。

```bash
conda activate <your_conda_env>
cd /path/to/sim2sim
```

如果你在训练工程根目录执行过下面的安装命令，`NumPy`、`OpenCV` 等项目依赖通常已经在当前环境中安装完成：

```bash
python -m pip install -e source/instinctlab
```

如果当前环境尚未安装 MuJoCo，执行：

```bash
python -m pip install mujoco
```

如果已经安装过 MuJoCo，可以跳过这一步。然后安装 ONNX Runtime：

```bash
python -m pip install onnxruntime
```

使用以下命令检查依赖是否安装成功：

```bash
python -c "import mujoco, onnxruntime; print('MuJoCo:', mujoco.__version__)"
```

完整配置检查（无需打开窗口）：

```bash
python check_config.py --task parkour
# 同时检查深度渲染，需要可用的 OpenGL/显示服务
python check_config.py --task parkour --render
```

检查包括依赖、场景和 mesh 加载、关节/传感器顺序、相机以及 ONNX 输入输出和一次推理。缺少策略文件时会列出路径并返回非零退出码。接口检查通过后，仍需用实际仿真确认策略行为及训练参数是否匹配。

Linux/WSL 桌面运行使用默认 GLFW 后端即可；需要能连接 `DISPLAY` 指向的显示服务。WSL 可用 `glxinfo -B` 检查 WSLg/OpenGL。

## 快速开始

运行前请先按“准备 ONNX 策略”放置对应模型文件。

运行 parkour policy：

```bash
python3 sim2sim.py --task parkour
```

启动时会打印 Python 路径和 MuJoCo 实际 OpenGL 渲染设备。设备名称含 `NVIDIA` 表示使用 NVIDIA 显卡；`llvmpipe` 或 `softpipe` 表示软件渲染。

出现卡顿时，启用分阶段耗时统计：

```bash
python3 sim2sim.py --task parkour --profile
```

每两秒输出控制循环频率及深度渲染、预处理、ONNX 推理、物理仿真和查看器同步耗时。控制循环频率与窗口显示帧率不同；默认控制目标为 50 Hz。

ONNX 默认使用单线程并关闭工作线程空转，避免两个小模型的线程池占用大量 CPU、影响渲染。可在 `config.py` 的 `ONNX_*` 配置中调整；启动时会打印实际配置。

运行站立 policy：

```bash
python3 sim2sim.py --task stand
```

parkour 模式键盘控制：

| 按键 | 功能 |
| --- | --- |
| `↑` / `KP8` | 增加前向速度 |
| `↓` / `KP2` | 减小前向速度 |
| `←` / `KP4` | 增加左转角速度 |
| `→` / `KP6` | 增加右转角速度 |
| `Space` / `KP5` | 速度命令归零 |
| `R` | 重置 |

parkour 默认启用航向保持：零手动转向输入时保持重置后的朝向，主动转向后保持新的方向。纠偏通过策略的转向速度指令完成，配置见 `config.py` 的 `HEADING_HOLD_*`；设置 `HEADING_HOLD_ENABLED=False` 可关闭。`--profile` 会显示实际朝向、目标朝向及策略收到的转向指令。

parkour 模式也支持 Xbox/XInput 手柄，无需增加依赖：

```bash
python3 sim2sim.py --task parkour --control gamepad
python3 gamepad_control.py --monitor  # 单独检查设备和输入
```

左摇杆上推控制前进，右摇杆左右控制转向，B 停止，Y 重置。摇杆回中或断连时速度指令归零；连接、停止、重置后需先将摇杆回中。键盘空格和 R 在手柄模式仍然有效，方向键速度控制仅在默认的键盘模式有效。死区与映射见 `config.py` 的 `GAMEPAD_*`；多手柄可指定 `--gamepad-id 0`。速度范围沿用 `COMMAND_RANGES`，默认不支持倒退和横移。

## 准备 ONNX 策略

本仓库不提供 ONNX 策略文件。请自行准备 actor 和 depth encoder，并放入 `policy/` 目录，或在 `config.py` 中修改路径。

parkour 模式默认读取：

```text
policy/parkour_actor.onnx
policy/parkour_depth_encoder.onnx
```

stand 模式默认读取：

```text
policy/stand_actor.onnx
policy/stand_depth_encoder.onnx
```

也可以保持自己的文件路径不变，在 `config.py` 中修改：

```python
PARKOUR_POLICY_FILE = "/path/to/exported/actor.onnx"
PARKOUR_DEPTH_ENCODER_FILE = "/path/to/exported/0-depth_encoder.onnx"
```

actor 和 depth encoder 应来自同一次训练或兼容配置。

## 目录结构

```text
.
|-- config.py
|-- mujoco_env.py
|-- onnx_inference.py
|-- sim2sim.py
|-- policy/
|-- g1_29dof/
|-- requirements.txt
|-- NOTICE.md
`-- LICENSE
```

## 常见问题

### 无法加载 ONNX

检查：

- `PARKOUR_POLICY_FILE` 是否指向实际存在的 actor ONNX。
- `PARKOUR_DEPTH_ENCODER_FILE` 是否指向实际存在的 depth encoder ONNX。
- 是否安装了 `onnxruntime`。
- actor 和 depth encoder 是否兼容。

## 开源边界

本仓库源码和文档使用 MIT License。你可以自由使用、复制、修改和分发本仓库代码，但需要保留原始版权声明和许可证文本。

请注意：

- 未经授权，请勿将本仓库中的策略权重、机器人模型、mesh 或其他第三方资产用于商业用途。
- ONNX 策略权重、`g1_29dof/*.xml` 和 `g1_29dof/meshes_recv_1_0/*.STL` 可能有各自的上游授权，公开再分发或商业使用前请自行确认许可边界。
- 本仓库不公开训练 reward、domain randomization、训练日志、checkpoint 或真机部署参数。
- 本仓库仅用于学习、研究和 Sim2Sim 验证，不构成真机部署建议。

## 共创方向

欢迎围绕以下方向共创： 

- 更清晰的安装和运行文档。
- 更多 MuJoCo 测试场景。
- 更稳定的示例策略接入方式。
- 不同系统和环境下的兼容性修复。
- 更轻量的 demo 资产组织方式。
