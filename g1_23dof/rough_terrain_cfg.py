"""23DoF 独立木板场景，距离单位 m。修改后运行本目录生成器。"""
DEFAULT_GROUND_Z = -0.35
PLANK_SETTINGS = {
    "name": "wooden_planks",
    "start": (2.0, 0.0, 0.0),
    "num_planks": 16,
    "plank_width": 0.90,
    "plank_length": 2.40,
    "thickness": 0.12,
    "min_gap": 0.05,
    "max_gap": 0.15,
    "seed": 23,
    "approach_length": 4.0,
    "landing_length": 2.0,
}
