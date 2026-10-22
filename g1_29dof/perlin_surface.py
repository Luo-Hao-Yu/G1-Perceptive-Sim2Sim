"""Metric, quantized noise field shared by plank tops and the ditch floor."""
from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

if __package__:
    from .perlin_noise import generate_fractal_noise_2d
else:
    from perlin_noise import generate_fractal_noise_2d


@dataclass(frozen=True)
class PerlinSurfaceCfg:
    noise_scale: float | Sequence[float] = (0.01, 0.04)
    noise_frequency: int = 20
    fractal_octaves: int = 2
    fractal_lacunarity: float = 2.0
    fractal_gain: float = 0.25
    centering: bool = True
    difficulty: float = 0.5
    horizontal_scale: float = 0.025
    vertical_scale: float = 0.001
    seed: int = 23
    ground_padding: float = 1.0
    height_limit: float | None = None  # Optional maximum absolute metric deviation.

    def amplitude(self) -> float:
        if not math.isfinite(self.difficulty) or not 0 <= self.difficulty <= 1:
            raise ValueError("perlin_cfg.difficulty must be in [0, 1]")
        if isinstance(self.noise_scale, (list, tuple)):
            if len(self.noise_scale) != 2:
                raise ValueError("perlin_cfg.noise_scale must be a scalar or a two-value range")
            low, high = self.noise_scale
            if not (math.isfinite(low) and math.isfinite(high) and 0 <= low <= high):
                raise ValueError("perlin_cfg.noise_scale must satisfy 0 <= low <= high")
            return float(low + self.difficulty * (high - low))
        if not math.isfinite(self.noise_scale) or self.noise_scale < 0:
            raise ValueError("perlin_cfg.noise_scale must be finite and nonnegative")
        return float(self.noise_scale)


class PerlinSurface:
    def __init__(self, cfg: PerlinSurfaceCfg, x_min: float, x_max: float, y_min: float, y_max: float):
        self.cfg = cfg
        amplitude = cfg.amplitude()
        for name in ("noise_frequency", "horizontal_scale", "vertical_scale", "fractal_lacunarity"):
            value = getattr(cfg, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"perlin_cfg.{name} must be finite and positive")
        if not isinstance(cfg.fractal_octaves, int) or not 1 <= cfg.fractal_octaves <= 8:
            raise ValueError("perlin_cfg.fractal_octaves must be an integer in 1..8")
        if not math.isfinite(cfg.fractal_gain) or not 0 <= cfg.fractal_gain <= 1:
            raise ValueError("perlin_cfg.fractal_gain must be in [0, 1]")
        if not math.isfinite(cfg.ground_padding) or cfg.ground_padding < 0:
            raise ValueError("perlin_cfg.ground_padding must be nonnegative")
        if not isinstance(cfg.seed, int) or not 0 <= cfg.seed < 2**32:
            raise ValueError("perlin_cfg.seed must be an integer in 0..2**32-1")
        if cfg.height_limit is not None and (not math.isfinite(cfg.height_limit) or cfg.height_limit < 0):
            raise ValueError("perlin_cfg.height_limit must be finite and nonnegative, or None")
        # Include both physical edges. Unlike a single large stepped hfield,
        # separate plank patches preserve vertical edges and exact net gaps.
        nx = max(2, math.ceil((x_max-x_min) / cfg.horizontal_scale - 1e-9) + 1)
        ny = max(2, math.ceil((y_max-y_min) / cfg.horizontal_scale - 1e-9) + 1)
        size_x = (nx-1) * cfg.horizontal_scale
        size_y = (ny-1) * cfg.horizontal_scale
        self.x = x_min + np.arange(nx) * cfg.horizontal_scale
        self.y = y_min + np.arange(ny) * cfg.horizontal_scale
        # Keep the training library's full-field centering, then cropping.
        # Some non-integer lacunarity/frequency combinations cannot tile its
        # internal gradient lattice; reject these instead of changing spectra.
        lattice = (int(cfg.noise_frequency*size_x), int(cfg.noise_frequency*size_y))
        internal = tuple(int(n * cfg.fractal_lacunarity**cfg.fractal_octaves) for n in lattice)
        current = lattice
        for _ in range(cfg.fractal_octaves):
            if any(n <= 0 or total % n for total, n in zip(internal, current)):
                raise ValueError("Perlin octave lattice must divide the internal sample shape")
            current = tuple(int(n*cfg.fractal_lacunarity) for n in current)
        if nx > internal[0] or ny > internal[1]:
            raise ValueError("Perlin sample count exceeds the training sampler's internal grid; increase horizontal_scale or noise_frequency")
        raw = generate_fractal_noise_2d(
            xSize=size_x, ySize=size_y, xSamples=nx, ySamples=ny,
            frequency=cfg.noise_frequency, fractalOctaves=cfg.fractal_octaves,
            fractalLacunarity=cfg.fractal_lacunarity, fractalGain=cfg.fractal_gain,
            zScale=amplitude, centering=cfg.centering, rng=np.random.RandomState(cfg.seed),
        )
        if cfg.height_limit is not None:
            # Uniformly reduce the whole field, preserving its spatial shape
            # and the common noise used by the boards and ditch. Never amplify
            # an already-small field or clip peaks into flat plateaus.
            peak = float(np.max(np.abs(raw)))
            if peak > cfg.height_limit:
                raw *= cfg.height_limit / peak
        # InstinctLab truncates noise/vertical_scale to int16. Keep int64 here
        # to avoid silent wrapping for invalid, excessively large amplitudes.
        units = raw / cfg.vertical_scale
        if cfg.height_limit is not None:
            # Snap roundoff at exact millimeter boundaries before truncating,
            # so e.g. 4.999999999999999 mm remains the intended 5 mm peak.
            rounded = np.rint(units)
            units = np.where(np.abs(units-rounded) < 1e-12, rounded, units)
        heights = np.trunc(units) * cfg.vertical_scale
        if cfg.height_limit is not None:
            # Keep even sub-grid limits strict; quantization remains toward 0.
            max_units = math.floor(cfg.height_limit / cfg.vertical_scale)
            heights = np.clip(heights, -max_units*cfg.vertical_scale, max_units*cfg.vertical_scale)
        self.heights = heights.T

    def patch(self, x_min, x_max, y_min, y_max):
        """Sample one patch in row-major Y/X order, including its exact edges."""
        nx = max(2, math.ceil((x_max-x_min)/self.cfg.horizontal_scale - 1e-9)+1)
        ny = max(2, math.ceil((y_max-y_min)/self.cfg.horizontal_scale - 1e-9)+1)
        xs = np.linspace(x_min, x_max, nx)
        ys = np.linspace(y_min, y_max, ny)
        if xs[0] < self.x[0]-1e-7 or xs[-1] > self.x[-1]+1e-7 or ys[0] < self.y[0]-1e-7 or ys[-1] > self.y[-1]+1e-7:
            raise ValueError("Patch lies outside the shared Perlin field")
        along_x = np.array([np.interp(xs, self.x, row) for row in self.heights])
        return np.array([np.interp(ys, self.y, col) for col in along_x.T]).T
