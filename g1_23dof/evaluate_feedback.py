"""Compare off/heading/path feedback using the same export, scene and seed."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from g1_23dof import config as cfg
from g1_23dof.runtime import Simulation


def evaluate(scene,mode,seconds,forward,seed):
    np.random.seed(seed)
    sim=Simulation(scene=scene,headless=True,feedback=mode)
    samples=[]
    stopped=False
    try:
        for i in range(math.ceil(seconds/cfg.POLICY_DT)):
            sim.commands[:]=(forward if i*cfg.POLICY_DT>=2 else 0,0,0)
            sim.step()
            data=sim.env.model_data
            sample={"time":float(data.time),"x":float(data.qpos[0]),"z":float(data.qpos[2]),**sim.command_diagnostics()}
            samples.append(sample)
            if sample['z']<.5:
                stopped=True
                break
        return {"scene":scene,"feedback":mode,"seed":seed,"forward":forward,
                "simulated_seconds":samples[-1]['time'],"stopped_below_0_5m":stopped,
                "final":samples[-1],"max_abs_y":max(abs(s['world_y']) for s in samples),
                "max_abs_yaw_deg":max(abs(s['yaw_deg']) for s in samples),
                "min_pelvis_z":min(s['z'] for s in samples),
                "trajectory":samples[::10]+[samples[-1]]}
    finally:
        sim.env.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',choices=('flat','rough','both'),default='both')
    parser.add_argument('--seconds',type=float,default=20.)
    parser.add_argument('--forward',type=float,default=.4)
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds<=2:
        parser.error('--seconds must exceed the 2-second stationary warmup')
    if not math.isfinite(args.forward) or not cfg.COMMAND_RANGES['lin_vel_x'][0]<=args.forward<=cfg.COMMAND_RANGES['lin_vel_x'][1]:
        parser.error('--forward must be within configured velocity limits')
    result={"checkpoint":cfg.METADATA['checkpoint'],"actor_sha256":hashlib.sha256(cfg.ACTOR_FILE.read_bytes()).hexdigest(),
            "encoder_sha256":hashlib.sha256(cfg.DEPTH_ENCODER_FILE.read_bytes()).hexdigest(),
            "settings":{"gain":cfg.HEADING_HOLD_GAIN,"yaw_limit":cfg.HEADING_HOLD_MAX_YAW_RATE,
                        "lookahead":cfg.PATH_LOOKAHEAD,"max_heading_offset_rad":cfg.PATH_MAX_HEADING_OFFSET},"runs":[]}
    scenes=('flat','rough') if args.scene=='both' else (args.scene,)
    result['scene_sha256']={scene:hashlib.sha256((cfg.HERE/f'scene_{scene}.xml').read_bytes()).hexdigest() for scene in scenes}
    result['model_sha256']=hashlib.sha256((cfg.HERE/'g1_23dof.xml').read_bytes()).hexdigest()
    result['settings'].update(sim_dt=cfg.SIM_DT,decimation=cfg.DECIMATION)
    for scene in scenes:
        for mode in ('off','heading','path'):
            run=evaluate(scene,mode,args.seconds,args.forward,args.seed)
            result['runs'].append(run)
            print(f"{scene}/{mode}: t={run['simulated_seconds']:.2f}s, X={run['final']['x']:.3f}m, Y={run['final']['world_y']:+.3f}m, "
                  f"max_yaw={run['max_abs_yaw_deg']:.2f}deg, min_z={run['min_pelvis_z']:.3f}m",flush=True)
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print(f"Saved comparison: {args.output}")


if __name__=='__main__':
    main()
