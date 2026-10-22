"""23DoF 独立木板场景，距离单位 m。修改后运行本目录生成器。"""
DEFAULT_GROUND_Z = -0.05  # 沟底名义高度（m），比板面低 12 cm；可与木板底面齐平，粗糙起伏叠加于此高度

# 对应训练的 PerlinPlaneTerrainCfg；整条路线共用一片噪声，平台和沟底均叠加。
PERLIN_SETTINGS = {
    "noise_scale": [0.01, 0.04],  # 缩放参数随 difficulty 线性插值，不是逐点凸起范围
    "noise_frequency": 20,
    "fractal_octaves": 2,
    "fractal_lacunarity": 2.0,
    "fractal_gain": 0.25,
    "centering": True,
    "difficulty": 0.5,  # 0～1；原始噪声缩放为 0.025，最终幅值另受 height_limit 限制
    "height_limit": 0.002,  # 实际起伏上限 ±0.5 cm = ±0.005 m；整片噪声等比例缩小，None 恢复原幅值
    "horizontal_scale": 0.025,  # 2.5 cm，训练提供的水平采样间距
    "vertical_scale": 0.001,  # 1 mm，训练提供的高度量化
    "seed": 23,  # 固定种子可复现整片表面；改变后需重新生成
    "ground_padding": 1.0,  # 沟底粗糙高度场向路线四周额外延伸 1 m
}
PLANK_SETTINGS = {  # 木板地形参数，由生成器传给 WoodenPlanksCfg；当前默认沿 +X 方向排列
    "name": "wooden_planks",  # 生成的几何体名称前缀，用于标识木板及起终点平台
    "start": (2.0, 0.0, 0.0),  # 第一块木板起始边中点的世界坐标 (x, y, z)，单位 m；z 为名义板面高度
    "num_planks": 16,  # 独立木板数量（正整数），不包含起点平台和终点平台
    "plank_width": 0.60,  # 单块木板沿行走方向 X 的完整尺寸（m）：60 cm，必须大于 0
    "plank_length": 0.90,  # 单块木板沿横向 Y 的完整尺寸（m）：90 cm，也用于起终点平台，必须大于 0
    "thickness": 0.05,  # 名义板面到底面的距离（m）：12 cm；粗糙顶面在名义高度上下起伏
    "min_gap": 0.20,  # 相邻木板边缘净间距的最小值（m），必须大于 0；间距在上下限之间均匀随机采样
    "max_gap": 0.20,  # 净间距最大值（m），不得小于 min_gap；当前上下限相等，所有缝隙固定为 30 cm
    "seed": 23,  # 缝隙随机采样的种子，相同参数和种子可复现布局；固定间距时不影响布局
    "approach_length": 4.0,  # 起点平台沿 X 的长度（m），从第一块木板起始边向 -X 延伸，与板面齐平，必须大于 0
    "landing_length": 2.0,  # 终点平台沿 X 的长度（m），紧接最后一块木板向 +X 延伸，与板面齐平，必须大于 0
    #"perlin_cfg": PERLIN_SETTINGS,  # 改为 None 可恢复平整木板和沟底
}
