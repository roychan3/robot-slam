# robot-slam

Simultaneous localization and mapping (SLAM) for a robot platform.

This repository has just been initialized. Source, dependencies, and the real
documentation land in later commits.

## Development

```bash
python3 -m venv .venv
source .venv/bin/activate
```

## What is not tracked

`.venv/` is ignored, along with the usual Python build and cache output. So are
the large binary artifacts a SLAM project accumulates, because they are
rewritten constantly and would dominate the history:

- recorded sensor data — `*.bag`, `*.db3`, `rosbag2_*/`
- point clouds and meshes — `*.pcd`, `*.ply`, `*.las`, `*.laz`
- generated maps, trajectories, and evaluation output
- learned model weights — `*.pt`, `*.pth`, `*.ckpt`, `*.onnx`

Keep datasets outside the repository, or under `data/`, which is also ignored.
