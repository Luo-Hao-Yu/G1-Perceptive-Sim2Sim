"""Check that the local policies keep their outputs after thread tuning."""

from pathlib import Path
import unittest

import numpy as np
import onnxruntime as ort

import config
from onnx_inference import OnnxPolicyInference


@unittest.skipUnless(
    all(Path(path).is_file() for path in (config.PARKOUR_POLICY_FILE, config.PARKOUR_DEPTH_ENCODER_FILE)),
    "Local parkour ONNX weights are required",
)
class PolicyThreadingTests(unittest.TestCase):
    def test_encoder_and_actor_outputs_match_original_thread_settings(self):
        old_encoder = ort.InferenceSession(config.PARKOUR_DEPTH_ENCODER_FILE, providers=['CPUExecutionProvider'])
        old_actor = ort.InferenceSession(config.PARKOUR_POLICY_FILE, providers=['CPUExecutionProvider'])
        encoder = OnnxPolicyInference(config.PARKOUR_DEPTH_ENCODER_FILE)
        actor = OnnxPolicyInference(config.PARKOUR_POLICY_FILE)
        rng = np.random.default_rng(123)
        y1, y2, x1, x2 = config.OBS_DEPTH_IMAGE_CROP_REGION
        height, width = config.RESIZED_DEPTH_IMAGE_SHAPE
        depth_shape = (1, 8, height - y1 - y2, width - x1 - x2)
        for index in range(3):
            with self.subTest(sample=index):
                depth = rng.uniform(0, 1, depth_shape).astype(np.float32)
                old_features = old_encoder.run(None, {'input': depth})[0]
                features = encoder({'input': depth})
                np.testing.assert_allclose(features, old_features, rtol=1e-5, atol=1e-6)
                history = rng.uniform(-1, 1, (1, sum(config.OBS_DIMS) * config.HISTORY_LENGTH)).astype(np.float32)
                old_obs = np.concatenate((history, old_features.reshape(1, -1)), axis=1)
                obs = np.concatenate((history, features.reshape(1, -1)), axis=1)
                expected = old_actor.run(None, {'input': old_obs})[0]
                np.testing.assert_allclose(actor({'input': obs}), expected, rtol=1e-5, atol=1e-6)


if __name__ == '__main__':
    unittest.main()
