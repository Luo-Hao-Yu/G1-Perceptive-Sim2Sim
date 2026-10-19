
"""Generate the independent 23DoF plank scene using shared terrain geometry."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_23dof import config as cfg
from g1_23dof.rough_terrain_cfg import DEFAULT_GROUND_Z, PLANK_SETTINGS
from g1_29dof.rough_terrain_generator import RoughTerrainGenerator, SceneCfg, WoodenPlanksCfg


def build_scene():
    scene = SceneCfg(terrains=(WoodenPlanksCfg(**PLANK_SETTINGS),), ground_z=DEFAULT_GROUND_Z)
    generator = RoughTerrainGenerator(scene, template_scene=cfg.HERE / "scene_flat.xml")
    generator.add_all()
    path = cfg.HERE / "scene_rough.xml"
    generator.save(path)
    print(f"Generated 23DoF scene: {path}")


if __name__ == "__main__":
    build_scene()
