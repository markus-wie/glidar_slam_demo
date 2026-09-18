import trimesh
import argparse
import os


def convert(input_path, output_path):
    print(f"Loading {input_path}...")
    # Load the GLB scene/mesh
    mesh = trimesh.load(input_path, force="mesh")

    # Ensure directory exists
    output_dir = os.path.dirname(output_path)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)

    print(f"Exporting to {output_path}...")
    # Export as DAE
    mesh.export(output_path)
    print("Done!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="Source .glb file")
    parser.add_argument("output", help="Destination .dae file")
    args = parser.parse_args()

    convert(args.input, args.output)
