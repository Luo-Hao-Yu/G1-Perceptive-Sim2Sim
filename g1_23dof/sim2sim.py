"""Independent G1 23DoF simulator entry point."""
import argparse
import math
from pathlib import Path
import sys

# Resolve shared root helpers without shadowing them with this entry's filename.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from g1_23dof.runtime import Simulation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", choices=("parkour",), default="parkour")
    parser.add_argument("--scene", choices=("flat", "rough"), default="flat")
    parser.add_argument("--control", choices=("keyboard", "gamepad"), default="keyboard")
    parser.add_argument("--gamepad-id", type=int, choices=range(16))
    parser.add_argument("--preview", action="store_true", help="PD pose only, no ONNX or depth renderer")
    parser.add_argument("--headless", action="store_true", help="No viewer; policy still needs an OpenGL depth context")
    parser.add_argument("--seconds", type=float, help="Exit after this many simulated seconds")
    parser.add_argument("--profile", action="store_true")
    args = parser.parse_args()
    if args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds <= 0):
        parser.error("--seconds must be finite and positive")
    if args.headless and args.seconds is None:
        parser.error("--headless requires --seconds")
    if args.gamepad_id is not None and args.control != "gamepad":
        parser.error("--gamepad-id requires --control gamepad")
    if args.headless and args.control == "gamepad":
        parser.error("--control gamepad requires the viewer (GLFW initialization)")
    instance = Simulation(scene=args.scene, preview=args.preview, headless=args.headless,
                          control=args.control, gamepad_id=args.gamepad_id)
    try:
        instance.run(seconds=args.seconds, profile=args.profile)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
