# 模型来源

- 机器人几何与动力学来自用户提供的 `/media/luohaoyu/HIKSEMI/g1_23dof.urdf`，原始字节已保存在本目录 `g1_23dof.urdf`。
- SHA256：`5d5a7380ec2f747e82dfd7c61fe70515715d413c3521a02911ef5dd9a007e374`，与本次策略 `policy_metadata.json` 的 `urdf_sha256` 一致。
- 策略和元数据从用户提供的 `/home/luohaoyu/下载/exported/` 复制至本目录 `policy/`。ONNX 文件保存在本机，并由 `.gitignore` 排除。
- STL 复用仓库 `g1_29dof/meshes_recv_1_0/`，没有重复复制模型资源。`UPSTREAM_LICENSE` 保留已有 Unitree 资源的上游许可声明。

`build_model.py` 使用 MuJoCo 的 URDF 导入器生成 `g1_23dof.xml`。保留 23 个活动关节、固定子链接质量、URDF 关节轴、硬限位和力矩上限；启用 URDF 中原本被注释的自由基座。增加 23 个力矩电机、传感器，以及安装于 URDF `d435_link` 的深度相机。相机使用 URDF 安装位姿，并将 ROS 链接坐标转换为 MuJoCo 相机坐标。

仿真补充参数：初始 pelvis 高度 0.793 m、5 ms 物理步长、implicitfast 积分器、关节 armature=0.01 kg·m²、相机垂直视场角 58.29°。这些值来自本项目的仿真参考设置，未由训练导出元数据确认。未对腰部应用 29DoF 配置中的符号翻转；观测根体按元数据选 pelvis。

`policy_metadata.json` 中关节位置限位是训练侧限位，数值比 URDF 硬限位窄；模型保留 URDF 物理硬限位。SDK 电机索引含空位，只作导出信息；实际读取关节、写入电机均按关节名映射，不把 SDK 索引当成 MuJoCo 数组下标。
