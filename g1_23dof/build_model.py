"""Rebuild MJCF from the supplied URDF, preserving its links and joint axes."""
import hashlib
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mujoco
from g1_23dof import config as cfg


def build_model():
    if hashlib.sha256(cfg.URDF_FILE.read_bytes()).hexdigest() != cfg.METADATA["urdf_sha256"]:
        raise ValueError("URDF hash does not match policy_metadata.json")
    urdf = ET.parse(cfg.URDF_FILE).getroot()
    joints = [j for j in urdf.findall("joint") if j.get("type") == "revolute"]
    names = [j.get("name") for j in joints]
    if len(names) != 23 or set(names) != set(cfg.ACTION_JOINT_NAMES):
        raise ValueError("URDF must contain exactly the 23 named policy joints")
    compiler = urdf.find("mujoco/compiler")
    mesh_dir = cfg.HERE.parent / "g1_29dof" / "meshes_recv_1_0"
    compiler.set("meshdir", str(mesh_dir))
    compiler.set("strippath", "true")
    compiler.set("fusestatic", "false")
    # The original URDF's floating world joint is commented out.
    ET.SubElement(urdf, "link", name="world")
    base = ET.SubElement(urdf, "joint", name="floating_base_joint", type="floating")
    ET.SubElement(base, "parent", link="world")
    ET.SubElement(base, "child", link="pelvis")
    model = mujoco.MjModel.from_xml_string(ET.tostring(urdf, encoding="unicode"))
    with tempfile.TemporaryDirectory(prefix="g1_23dof_") as temp:
        path = str(Path(temp) / "imported.xml")
        mujoco.mj_saveLastXML(path, model)
        root = ET.parse(path).getroot()
    root.set("model", "G1 23DoF from supplied URDF")
    root.find("compiler").set("meshdir", "../g1_29dof/meshes_recv_1_0")
    ET.SubElement(root, "option", timestep=str(cfg.SIM_DT), integrator="implicitfast")
    pelvis = root.find(".//body[@name='pelvis']")
    pelvis.set("pos", f"0 0 {cfg.SPAWN_HEIGHT}")
    for name in names:
        root.find(f".//joint[@name='{name}']").set("armature", str(cfg.JOINT_ARMATURE))
    for name in ("imu_in_pelvis", "imu_in_torso"):
        body = root.find(f".//body[@name='{name}']")
        ET.SubElement(body, "site", name=name, size="0.005", group="3")
    camera_body = root.find(".//body[@name='d435_link']")
    # ROS link +X forward/+Y left/+Z up -> OpenGL camera -Z forward/+Y up.
    ET.SubElement(camera_body, "camera", name=cfg.DEPTH_CAMERA_NAME,
                  xyaxes="0 -1 0 0 0 1", fovy=str(cfg.DEPTH_FOVY))
    motors = ET.SubElement(root, "actuator")
    effort_by_name = dict(zip(cfg.ACTION_JOINT_NAMES, cfg.TORQUE_LIMITS))
    for name in names:
        limit = effort_by_name[name]
        ET.SubElement(motors, "motor", name=name, joint=name,
                      ctrllimited="true", ctrlrange=f"{-limit} {limit}")
    sensors = ET.SubElement(root, "sensor")
    for tag in ("jointpos", "jointvel", "jointactuatorfrc"):
        for name in names:
            ET.SubElement(sensors, tag, name=f"{name}_{tag}", joint=name)
    for prefix, site in (("", "imu_in_pelvis"), ("secondary_", "imu_in_torso")):
        ET.SubElement(sensors, "framequat", name=f"{prefix}imu_quat", objtype="site", objname=site)
        ET.SubElement(sensors, "gyro", name=f"{prefix}imu_gyro", site=site)
        ET.SubElement(sensors, "accelerometer", name=f"{prefix}imu_acc", site=site)
    ET.indent(root, space="  ")
    output = cfg.HERE / "g1_23dof.xml"
    ET.ElementTree(root).write(output, encoding="utf-8")
    verified = mujoco.MjModel.from_xml_path(str(cfg.HERE / "scene_flat.xml"))
    assert (verified.nq, verified.nv, verified.nu) == (30, 29, 23)
    print(f"Generated {output}: nq=30, nv=29, nu=23")


if __name__ == "__main__":
    build_model()
