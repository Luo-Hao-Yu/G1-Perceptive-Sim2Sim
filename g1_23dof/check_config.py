"""Validate the supplied URDF, 23DoF scenes and real ONNX interfaces."""
import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mujoco
import numpy as np
from g1_23dof import config as cfg
from g1_23dof.runtime import Policy, Simulation, validate_model


def check_urdf(model):
    root = ET.parse(cfg.URDF_FILE).getroot()
    for link in root.findall("link"):
        mass = link.find("inertial/mass")
        if mass is not None and not np.isclose(model.body(link.get("name")).mass[0], float(mass.get("value")), atol=1e-7):
            raise ValueError(f"Link mass mismatch: {link.get('name')}")
    for joint in root.findall("joint"):
        if joint.get("type") != "revolute":
            continue
        name = joint.get("name")
        mjoint = model.joint(name)
        limit = joint.find("limit")
        if not np.allclose(mjoint.range, [float(limit.get("lower")), float(limit.get("upper"))]):
            raise ValueError(f"Physical joint range mismatch: {name}")
        if not np.allclose(mjoint.axis, np.fromstring(joint.find("axis").get("xyz"), sep=" ")):
            raise ValueError(f"Joint axis mismatch: {name}")
        if not np.allclose(model.jnt_actfrcrange[mjoint.id], [-float(limit.get("effort")), float(limit.get("effort"))]):
            raise ValueError(f"URDF torque limit mismatch: {name}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-only", action="store_true")
    parser.add_argument("--render", action="store_true", help="Render actual camera and run one full policy step")
    args = parser.parse_args()
    if args.render and args.model_only:
        parser.error("--render cannot be combined with --model-only")
    for scene in ("flat", "rough"):
        model = mujoco.MjModel.from_xml_path(str(cfg.HERE / f"scene_{scene}.xml"))
        validate_model(model)
        check_urdf(model)
        print(f"[PASS] {scene}: 23 joints/motors, nq=30, nv=29; URDF masses, axes, hard ranges and efforts")
    print(f"[PASS] URDF SHA256 matches metadata; root={cfg.ROOT_BODY}, control={1/cfg.POLICY_DT:g} Hz")
    if not args.model_only:
        print(f"Policy directory: {cfg.POLICY_DIR}\nMetadata checkpoint: {cfg.METADATA['checkpoint']}")
        policy = Policy()
        history = [np.zeros(dim * cfg.HISTORY_LENGTH, np.float32) for dim in cfg.OBS_DIMS]
        action = policy(history, np.zeros(cfg.DEPTH_INPUT_SHAPE, np.float32))
        print(f"[PASS] encoder {cfg.DEPTH_INPUT_SHAPE} -> (1, 128); actor (1, {cfg.ACTOR_INPUT_DIM}) -> {action[None].shape}; finite inference")
    if args.render:
        instance = Simulation(headless=True)
        try:
            instance.step()
            print(f"[PASS] actual camera -> preprocessing -> encoder -> actor -> 4 physics steps; depth={instance.depth.get_depth_obs().shape}")
        finally:
            instance.env.close()
    print("[NOTE] Metadata omits depth FOV, clipping and temporal sampling; reference defaults need training-side confirmation.")


if __name__ == "__main__":
    main()
