# SDF to URDF conversion

`sdf_to_urdf.py` converts a strict tree-structured SDF model to URDF and resolves
SDFormat pose frames before exporting. Link poses default to `__model__`; joint
poses default to their parent link; an explicit `relative_to` value overrides
those defaults. The reserved `__model__` frame is supported.

The converter preserves model, link, joint, visual, and mesh names, along with
visual geometry and inertial data. Sensor-bearing links are kept, but SDF sensor
definitions, collisions, Gazebo plugins, and `ros2_control` are omitted because
they have no direct URDF equivalent in this exporter.

```bash
python3 sdf_to_urdf.py --verbose \
  ../../models/car/model.sdf \
  ../../models/car/model.urdf
```

The input must contain exactly one top-level model with a single rooted link
tree. Loops, multiple parent joints, unknown pose frames, malformed poses, and
unknown parent or child links are rejected with an error. Debug mode reports
the parsed tree and pose-resolution stages.
