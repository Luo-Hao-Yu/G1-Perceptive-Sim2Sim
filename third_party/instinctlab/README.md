# InstinctLab Perlin sampler attribution

`g1_29dof/perlin_noise.py` is adapted from the project-instinct/InstinctLab
Perlin utility, retrieved on 2026-10-20:

https://github.com/project-instinct/InstinctLab/blob/main/source/instinctlab/instinctlab/utils/perlin.py

Original file SHA256:
`59215de2f0b14c315bb035b6b389f486a3f39f97940ccbfa6f957d167461da09`

The adaptation adds explicit local RNG injection to make terrain generation
reproducible without changing NumPy's global RNG. The interpolation, octave
weights, full-field centering, internal sample resolution, and crop semantics
are retained. Regression samples were checked against the original utility.

This upstream material is licensed under CC BY-NC 4.0. The upstream license is
included in `LICENSE` in this directory; the original project is credited above.
The new metric surface wrapper, heightfield integration and tests are separate
project code.

Training-side amplitude interpolation and height quantization reference:
https://github.com/project-instinct/InstinctLab/blob/main/source/instinctlab/instinctlab/terrains/height_field/hf_terrains.py
