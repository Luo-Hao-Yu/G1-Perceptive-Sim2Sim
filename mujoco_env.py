import os
import time
import threading
import mujoco
import mujoco.viewer
import numpy as np

from typing import Tuple

class DepthImageRender:
    def __init__(
        self,
        camera_name: str,
        model: mujoco.MjModel,
        model_data: mujoco.MjData,
        height: int,
        width: int,
    ):
        self._camera_name = camera_name
        self._model = model
        self._model_data = model_data
        self._render = mujoco.Renderer(model, height, width)
        self._render.enable_depth_rendering()
        self._data = np.zeros((height, width), dtype=np.float32)
        self.last_render_ms = 0.0

        # Query the active MuJoCo context, rather than a separate glxinfo context.
        from OpenGL import GL
        renderer = GL.glGetString(GL.GL_RENDERER)
        self.gl_renderer = renderer.decode() if renderer else "unknown"
        print(f"MuJoCo OpenGL renderer: {self.gl_renderer}")
        print(f"Depth render resolution: {width}x{height}")

    def render(self, model_data_lock=None) -> np.ndarray:
        start = time.perf_counter()
        if model_data_lock is not None:
            with model_data_lock:
                self._render.update_scene(self._model_data, self._camera_name)
        else:
            self._render.update_scene(self._model_data, self._camera_name)
        self._render.render(out=self._data)
        self.last_render_ms = (time.perf_counter() - start) * 1000
        return self._data

class MujocoEnv:
    def __init__(
        self,
        mjcf_file: str,
        sim_dt: float = 0.005,
        decimation: int = 4,
        depth_camera_name: str = "depth_camera",
        depth_image_shape: Tuple[int, int] = (270, 480),
        use_secondray_imu: bool = True,
        viewer_lookat: Tuple[float, float, float] | None = None,
        viewer_distance: float | None = None,
        viewer_azimuth: float | None = None,
        viewer_elevation: float | None = None,
        key_callback=None,
        viewer_enabled: bool = True,
        depth_enabled: bool = True,
    ):
        self.mjcf_file = mjcf_file
        self.sim_dt = sim_dt
        self.decimation = decimation

        self.depth_camera_name = depth_camera_name
        self.depth_image_shape = depth_image_shape

        self.use_secondary_imu = use_secondray_imu
        self.viewer_lookat = viewer_lookat
        self.viewer_distance = viewer_distance
        self.viewer_azimuth = viewer_azimuth
        self.viewer_elevation = viewer_elevation
        self.key_callback = key_callback
        self.viewer = None
        self.depth_render = None
        self._viewer_threads = []

        self._init_model()
        try:
            if viewer_enabled:
                self._init_viewer()
            if depth_enabled:
                self._init_depth_render()
        except Exception:
            self.close()
            raise

    def _init_model(self):
        if not self.mjcf_file.endswith(".xml"):
            raise ValueError(f"Unsupport file type, MujocoEnv only support xml file.")
        if not os.path.exists(self.mjcf_file):
            raise FileNotFoundError(f"No such file called {self.mjcf_file}")
        self.model = mujoco.MjModel.from_xml_path(self.mjcf_file)
        self.model_data = mujoco.MjData(self.model)
        self.model.opt.timestep = self.sim_dt

        self.nb_actuators = self.model.nu
        self.nb_sensors = self.model.nsensor
        self.imu_offset = self.nb_actuators * 3
        next_sensor_name = mujoco.mj_id2name(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, self.imu_offset
        )
        self.have_imu = True if next_sensor_name == "imu_quat" else False
        if self.use_secondary_imu:
            secondary_imu_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SENSOR, "secondary_imu_quat"
            )
            self.imu_offset = self.model.sensor_adr[secondary_imu_id]

    def _init_viewer(self):
        previous_threads = set(threading.enumerate())
        self.viewer = mujoco.viewer.launch_passive(
            self.model, self.model_data, key_callback=self.key_callback
        )
        self._viewer_threads = [
            thread for thread in threading.enumerate()
            if thread not in previous_threads and "_launch_internal" in thread.name
        ]
        self._setup_initial_free_camera()

    def _setup_initial_free_camera(self):
        self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        if self.viewer_lookat is not None:
            self.viewer.cam.lookat[:] = np.asarray(self.viewer_lookat, dtype=np.float64)
        if self.viewer_distance is not None:
            self.viewer.cam.distance = self.viewer_distance
        if self.viewer_azimuth is not None:
            self.viewer.cam.azimuth = self.viewer_azimuth
        if self.viewer_elevation is not None:
            self.viewer.cam.elevation = self.viewer_elevation

    def _init_depth_render(self):
        self.depth_render = DepthImageRender(
            self.depth_camera_name,
            self.model,
            self.model_data,
            self.depth_image_shape[0],
            self.depth_image_shape[1],
        )

    def render_depth_image(self):
        return self.depth_render.render()

    def forward(self):
        mujoco.mj_forward(self.model, self.model_data)

    def reset(self, default_joint_pos: np.ndarray | None = None):
        mujoco.mj_resetData(self.model, self.model_data)
        if default_joint_pos is not None:
            joint_count = len(default_joint_pos)
            self.model_data.qpos[7 : 7 + joint_count] = default_joint_pos
        mujoco.mj_forward(self.model, self.model_data)

    def physical_step(self):
        mujoco.mj_step(self.model, self.model_data)

    def close(self):
        if self.depth_render is not None:
            self.depth_render._render.close()
        if self.viewer is not None:
            self.viewer.close()
        for thread in self._viewer_threads:
            if thread is not threading.current_thread():
                thread.join(timeout=5)
