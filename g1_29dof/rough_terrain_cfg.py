from __future__ import annotations

from typing import Any

TerrainSpec = tuple[str, dict[str, Any]]

# 原有五种地形暂时停用。恢复时注释下方木板配置，取消本段注释，
# 并将 DEFAULT_GROUND_Z 改回 0.0，然后重新运行生成器。
# DEFAULT_TERRAIN_SPECS: tuple[TerrainSpec, ...] = (
#     (
#         "pyramid_stairs_with_gaps",
#         {
#             "name": "pyramid_stairs_with_gaps_1",
#             "center": (6.0, 0.0, 0.0),
#             "width": 8.0,
#             "length": 8.0,
#             "num_layers": 5,
#             "layer_height": 0.10,
#             "step_width": 0.30,
#             "center_platform_width": 2.0,
#             "gap_width": 0.5,
#         },
#     ),
#     (
#         "pyramid_stairs",
#         {
#             "center": (16.0, 0.0, 0.0),
#             "width": 8.0,
#             "length": 8.0,
#             "num_layers": 10,
#             "layer_height": 0.15,
#             "step_width": 0.30,
#         },
#     ),
#     (
#         "boxes",
#         {
#             "center": (26.0, 0.0, 0.0),
#             "width": 8.0,
#             "length": 8.0,
#             "nrow": 12,
#             "ncol": 12,
#             "min_height": 0.05,
#             "max_height": 0.20,
#         },
#     ),
#     (
#         "slope_pyramid",
#         {
#             "center": (36.0, 0.0, 0.0),
#             "width": 8.0,
#             "length": 8.0,
#             "top_width": 1.0,
#             "top_length": 1.0,
#             "height": 0.8,
#         },
#     ),
#     (
#         "random_rough",
#         {
#             "center": (46.0, 0.0, 0.0),
#             "width": 8.0,
#             "length": 8.0,
#             "nrow": 24,
#             "ncol": 24,
#             "height_scale": 0.20,
#             "smooth_passes": 1,
#         },
#     ),
# )

# 缝隙底部地面比板面低 35 cm；板面及起终点平台均保持 z=0。
DEFAULT_GROUND_Z = -0.35

DEFAULT_TERRAIN_SPECS: tuple[TerrainSpec, ...] = (
    (
        "wooden_planks",
        {
            "name": "wooden_planks",
            "start": (2.0, 0.0, 0.0),  # 第一块木板前缘中心，z 为板面高度
            "num_planks": 16,
            "plank_width": 0.80,  # 沿行走方向 X 的落脚宽度，单位 m
            "plank_length": 2.40,  # 沿横向 Y 的木板长度
            "thickness": 0.12,
            "min_gap": 0.05, # 相邻板边缘之间的净距离，不是中心间距
            "max_gap": 0.15,
            "seed": 23,  # 改变种子可重新排列缝隙，重置仿真不会改变布局
            "approach_length": 4.0,  # 起点平台覆盖 x=-2..2，承托初始机器人
            "landing_length": 2.0,
        },
    ),
)
