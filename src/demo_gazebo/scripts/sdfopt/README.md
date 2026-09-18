# Blender scripts

## Installation
To install bpy, we need to use python3.11:

```bash
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:deadsnakes/ppa
sudo apt update
sudo apt install -y python3.11 python3.11-venv
python3.11 -m venv .venv
source .venv/bin/activate
pip install bpy==4.5.8
```

## SDF optimize
Simplify meshes of FusionSDF exported SDFs (TODO and add materials to them) using blender.

You might need to up/downgrade python to be able to install it.

### Usage
```
> python3 sdfopt.py --help
usage: sdfopt.py [-h] [--format {glb,dae}] sdf_dir

Combine and optimize SDF files exported by FusionSDF.

positional arguments:
  sdf_dir             Directory containing SDF file to optimize

options:
  -h, --help          show this help message and exit
  --format {glb,dae}  Export format for the combined meshes (default: glb)
```

### Features
This script takes all bodies within one component, combines them, optimizes the mesh a bit and exports as a single .glb file.
This for one reduces the size of the mesh drastically and for the other reduces the amount of visuals in general drastically.
It also updates the .sdf file by removing all visuals that have been combined to one and readd the one that has been created.

## Transform .glb to .dae
This script transforms .glb files to .dae files, which are more compatible with rviz2 for example.
