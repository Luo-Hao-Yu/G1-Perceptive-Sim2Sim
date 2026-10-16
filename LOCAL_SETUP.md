# 本机配置与启动

配置验证日期：2026-10-14。

## 运行环境

- Python 3.11.17：项目内 `.tools/python/`。
- 虚拟环境：`.venv/`，已安装 `requirements-lock.txt` 的全部依赖及 pip。
- `.python-version` 指定 Python 3.11；VS Code 默认解释器指向 `.venv/bin/python`。
- MuJoCo 3.15.0、ONNX Runtime 1.31.0、NumPy 2.4.6、OpenCV 5.0.0.93、GLFW 2.10.2。
- ONNX 推理使用 CPU；深度渲染使用 NVIDIA GeForce RTX 3070 Ti Laptop GPU。

`.tools/` 和 `.venv/` 已加入 Git 忽略规则。系统的 `python3` 是 Python 3.8，请使用以下项目解释器命令。

## 检查与运行

在项目根目录打开终端：

```bash
# 检查依赖、场景和 ONNX 接口
.venv/bin/python -E check_config.py --task parkour

# 同时验证深度渲染，需要能访问本机桌面
.venv/bin/python -E check_config.py --task parkour --render

# 启动仿真，每两秒输出性能统计
.venv/bin/python -E sim2sim.py --task parkour --profile
```

也可以先执行 `source .venv/bin/activate`，再用 `python -E` 运行。`-E` 避免外部 Python 环境变量干扰项目依赖。

键盘：上下箭头调节前进速度，左右箭头调节转向，空格停止，R 重置。

## VS Code 按钮生成地形

保存地形配置后，按 `Ctrl+Shift+B` 可直接执行默认任务「生成地形（项目 Python 3.11）」。也可以打开左侧「运行和调试」，选中同名配置，点击绿色三角形，或按 `Ctrl+F5`（不调试运行）/ `F5`（调试运行）。这两份配置都明确指定项目的 `.venv/bin/python`，不依赖 Python 扩展当前选中的解释器。

若使用编辑器右上角的「运行 Python 文件」按钮，需要执行一次 `Ctrl+Shift+P` → `Python: Select Interpreter` → 选择项目 `.venv/bin/python`。如果列表里没有，选择「输入解释器路径」，输入 `/home/luohaoyu/G1-Perceptive-Sim2Sim/.venv/bin/python`。项目已有默认解释器设置，但扩展之前选中的系统解释器可能仍然生效。

若运行日志显示 `/usr/bin/python3`，说明使用的是系统 Python 3.8，会在 `tuple[str, ...]` 等类型定义处报错；应使用项目 Python 3.11。看到 `Generated rough terrain scene: .../scene_rough.xml` 表示生成成功，再重启仿真加载新场景。

## Xbox / XInput 手柄控制

连接 USB 或已完成系统配对的蓝牙手柄后，使用：

```bash
.venv/bin/python -E sim2sim.py --task parkour --control gamepad --profile
```

| 操作 | 功能 |
| --- | --- |
| 左摇杆上推 | 按推杆幅度控制前进，范围 0～0.9 m/s |
| 左摇杆下拉 | 前进指令为零，当前配置不支持倒退 |
| 右摇杆向左 / 右 | 左转 / 右转，最大角速度 1 rad/s |
| 摇杆回中 | 对应速度指令归零 |
| B | 前进和转向指令归零；摇杆回中后才能恢复移动 |
| Y | 重置仿真；摇杆回中后才能恢复移动 |
| 键盘空格 / R | 手柄模式下仍可停止 / 重置 |

连接或重新连接手柄后，先松开两个摇杆再操作。断连或无可用手柄时，手动速度指令保持为零，程序等待连接。启用下文的航向保持时，策略仍可能收到少量自动转向纠偏指令。当前控制模式不响应键盘方向键，避免两个指令来源互相覆盖。只用键盘时省略 `--control gamepad`，或指定 `--control keyboard`。

手柄轴和按键使用 [GLFW 标准手柄映射](https://www.glfw.org/docs/latest/input_guide.html#gamepad)，无需新增依赖。默认死区为 0.15；映射与死区可在 `config.py` 的 `GAMEPAD_*` 配置中调整，最大速度沿用 `COMMAND_RANGES`。

单独检查设备识别或实时输入，无需启动仿真：

```bash
# 列出设备编号和是否有标准映射
.venv/bin/python -E gamepad_control.py

# 实时显示前进、转向、重置指令；Ctrl+C 退出
.venv/bin/python -E gamepad_control.py --monitor

# 多手柄时指定诊断输出的编号（示例为 0）
.venv/bin/python -E sim2sim.py --task parkour --control gamepad --gamepad-id 0
```

若没有识别到手柄，先确认系统已连接并识别为游戏手柄，使用 Xbox/XInput 模式，再运行诊断命令。连接诊断需能访问本机桌面。2026-10-15 接入时尚未检测到实体手柄；12 项模拟输入测试及带窗口的完整仿真控制检查已通过，覆盖映射、死区、停止、重置、回中和断连重连行为。实体摇杆和按键仍需接入后确认。

## 已验证的配置

- 权重：`policy/parkour_actor.onnx` 和 `policy/parkour_depth_encoder.onnx`，与 `config.py` 默认路径一致。
- 场景：`g1_29dof/scene_rough.xml`；模型和网格可正常加载。
- 29 个关节、执行器顺序、传感器顺序、第二 IMU 和深度相机通过检查。
- 仿真步长 0.005 秒，每 4 步控制一次，目标控制频率 50 Hz。
- 深度渲染分辨率 270 × 480；编码器输入 `(1, 8, 18, 32)`；actor 输入 `(1, 896)`；输出 29 个有限动作。
- 2026-10-14 初始场景的 `pip check` 通过；带窗口的完整控制循环运行 6 秒，通过有限数值检查，实测约 49.3–49.5 Hz，结束时基座高度约 0.773 米。

该短时验证使用零速度指令；跑酷运动表现以及参数与训练配置是否一致，仍需结合训练配置和运动测试确认。

## 窗口 FPS 排查与优化（2026-10-15）

本机复测发现主要瓶颈是两个 ONNX 会话默认创建的多核线程池及其空转等待。推理本身仅需约 0.2～0.5 ms，但进程会占用约 12 个 CPU 核心，影响深度渲染、窗口渲染及同步的调度。[ONNX Runtime 线程文档](https://onnxruntime.ai/docs/performance/tune-performance/threading.html)说明了默认线程配置与空转行为。

已在 `config.py` 配置 `ONNX_INTRA_OP_THREADS=1`、`ONNX_INTER_OP_THREADS=1`、`ONNX_ALLOW_SPINNING=False`，由 `onnx_inference.py` 应用于 actor 和深度编码器。启动时会打印这些设置。当前小模型在单线程下推理延迟仍约 0.2～0.3 ms。

同一木板场景、相同窗口尺寸（1706 × 1066）、零速度指令下的约 8 秒测试结果如下。窗口 FPS 为截图时的瞬时采样，控制频率为终端每两秒统计；截图采集期间的控制统计会受到短时阻塞影响。

| 指标 | 原 ONNX 默认设置 | 当前设置 |
| --- | --- | --- |
| 进程总 CPU 使用率，100% 表示一个核心 | 约 1233% | 约 44% |
| 进程线程总数，包含其他依赖的线程 | 56 | 30 |
| 稳态控制频率 | 约 40～45 Hz | 约 49～50 Hz |
| 窗口 FPS 截图采样 | 47 | 53～57 |
| 深度渲染耗时 | 约 8 ms | 约 3.4～3.9 ms |
| 窗口同步耗时 | 约 5～8 ms | 约 3.4 ms |

物理仿真每个控制周期仅约 0.4～0.9 ms；木板仍为 16 块，减小板间缝隙没有增加几何体数量。实际使用的是 NVIDIA 渲染器。窗口 FPS 还受窗口大小、显示同步、桌面合成和画面效果影响；本次关闭阴影/反射的收益不稳定，采用了实测有效的 ONNX 线程调整。

13 项测试及配置检查通过，其中使用三个随机输入比较了优化前后的深度特征和 actor 动作，结果在 `rtol=1e-5, atol=1e-6` 容差内一致。物理步长保持 0.005 s，控制周期为 0.02 s（50 Hz）；提高窗口 FPS 不需要改变控制频率。

重新启动后查看每两秒的分阶段统计：

```bash
.venv/bin/python -E sim2sim.py --task parkour --control gamepad --profile
```

## 重置后的身体朝向（2026-10-16）

检查确认重置瞬间骨盆、躯干和木板的前进方向均为世界坐标 +X，夹角为 0°。但原策略在零手动速度输入下，约 0.5 秒就将躯干转到 7.8°，约 1 秒达到 9.7°。策略观测包含角速度和重力方向，没有绝对航向，零转向速度指令不能自动消除已经形成的航向偏差。

当前 parkour 模式使用真实重置姿态填充本体观测历史，并默认开启外层航向保持：

- `HEADING_HOLD_ENABLED=True`：设为 `False` 可关闭航向反馈。
- `HEADING_HOLD_BODY="torso_link"`：以躯干的世界坐标前向轴计算朝向。
- `HEADING_HOLD_GAIN=2.0`：航向偏差转换为策略转向速度指令的比例系数。
- `HEADING_HOLD_MAX_YAW_RATE=0.5`：自动纠偏的最大转向指令，单位 rad/s。

重置时记录初始朝向；不手动转向时，额外转向指令使身体回到目标朝向。手柄或键盘主动转向时，采用手动指令并更新目标，松开转向后保持新的方向。按 R 的重置请求由控制线程执行，避免查看器回调在物理步或推理期间直接修改模型状态。重置会同时清空动作、命令和深度历史，使用当前姿态初始化角速度、重力、关节等本体观测历史。

本机零速度、起点平台上的 10 秒验证中，躯干偏角由原来的约 10～12°，降为约 1 秒 1.6°、2 秒 0.2°，后续采样约在 ±0.15° 内。起步前 0.5 秒仍有约 5.4° 的短暂姿态调整，并非每一帧都严格保持 0°。19 项测试、场景与 ONNX 检查通过，覆盖航向跨 ±180°、限幅、手动转向优先和重置状态。

`--profile` 会额外输出 `heading`、`target`（度）和 `policy_yaw`（rad/s）。后者包含自动纠偏，可能与手柄的手动转向输入不同。这项功能是在原策略外增加航向反馈；评估策略原始航向表现时应关闭 `HEADING_HOLD_ENABLED`，同时注意当前重置历史已采用真实姿态初始化。

## 当前地形：不等间距木板（2026-10-15）

原有五种地形已在 `g1_29dof/rough_terrain_cfg.py` 中注释保留，当前只生成横向长条木板，机器人沿 +X 方向行走并跨越板间缝隙。

| 参数 | 当前值 |
| --- | --- |
| 木板数量 `num_planks` | 16 |
| 沿 X 的落脚宽度 `plank_width` | 0.40 m |
| 沿 Y 的木板长度 `plank_length` | 2.40 m |
| 厚度 `thickness` | 0.12 m |
| 板边净缝隙 `min_gap` / `max_gap` | 0.10～0.15 m，固定种子 23 |
| 板面高度 | z=0，与起终点平台齐平 |
| 缝隙底部地面 `DEFAULT_GROUND_Z` | z=-0.35 m |

起点平台覆盖 x=-2～2 m，承托机器人初始位置。第一块木板从 x=2 m 开始，最后一块结束于约 x=10.233 m，终点平台继续延伸 2 m。木板为可碰撞的固定箱体，板间没有与板面齐平的碰撞支撑；下方保留低处地面承接失足。场景和深度渲染检查已通过，并通过向下射线确认板面及 15 条真实缝隙的高度。已按 10～15 cm 配置重新生成 XML，并核对当前 15 条实际净缝隙均在范围内，约为 10.12～14.74 cm。

修改参数或 `seed` 后重新生成，再重新启动仿真：

```bash
.venv/bin/python -E g1_29dof/rough_terrain_generator.py
.venv/bin/python -E sim2sim.py --task parkour --profile
```

可先按一次 ↑，使用 0.2 m/s 前进。场景提供跨缝落脚任务；现有 ONNX 是否能稳定踩板取决于训练得到的策略，修改地形不会让策略自动学会新的落脚行为。

2026-10-15 最初使用 10～30 cm 缝隙、0.4 m/s 和零转向指令进行了短时行走测试：脚部接触到前六块木板的顶面，但随后向侧边偏移、踩到低处地面，未始终沿板面通过整段木板。不同测试中出现了失足或在低处地面继续行走的情况。随后使用 10～20 cm 配置完成上述 FPS 复测；当前 10～15 cm 配置尚未验证稳定通过全程的能力。可以继续调整 `max_gap`，或增大 `plank_width` / `plank_length` 来降低难度。

恢复原地形：注释当前 `wooden_planks` 的 `DEFAULT_TERRAIN_SPECS`，取消上方原五种地形配置的注释，将 `DEFAULT_GROUND_Z` 改回 `0.0`，再运行生成器。若需要恢复原全景视角，将 `config.py` 中 `VIEWER_LOOKAT` 改回 `(15.0, 0.0, 0.5)`、`VIEWER_DISTANCE` 改回 `32.0`。

## stand 模式

当前未提供 stand 权重。需要另行放入同一训练配置对应的 `policy/stand_actor.onnx` 和 `policy/stand_depth_encoder.onnx`，然后运行：

```bash
.venv/bin/python -E check_config.py --task stand
.venv/bin/python -E sim2sim.py --task stand
```
