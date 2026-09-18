#!/usr/bin/env python3

import argparse
import math
import os
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
import re

import bpy


def find_meshes(directory):
    """Find all mesh files in the given directory."""
    mesh_extensions = ["*.obj", "*.stl", "*.glb", "*.gltf", "*.ply", "*.dae"]
    mesh_files = []
    for ext in mesh_extensions:
        mesh_files.extend(directory.rglob(ext))
    return mesh_files


def main():
    parser = argparse.ArgumentParser(
        description="Combine and optimize SDF files exported by FusionSDF."
    )
    parser.add_argument(
        "sdf_dir", type=str, help="Directory containing SDF file to optimize"
    )
    # Added argument for export format
    parser.add_argument(
        "--format",
        type=str,
        choices=["glb", "dae"],
        default="glb",
        help="Export format for the combined meshes (default: glb)",
    )
    args = parser.parse_args()

    sdf_dir = Path(args.sdf_dir)
    if not sdf_dir.exists() or not sdf_dir.is_dir():
        print(f'Error: "{sdf_dir}" does not exist or is not a directory')
        return
    print(f"Optimizing and combining SDF files in directory: {sdf_dir}")

    sdf_file_path = sdf_dir / "model.sdf"
    if not sdf_file_path.exists():
        print(f"Error: No such file: {sdf_file_path}")
        return

    meshes_dir = sdf_dir / "meshes"
    if not meshes_dir.exists():
        print(f'Directory "{sdf_dir}" does not contain a "meshes" directory. Exiting.')
        return

    # Backup originals
    sdf_orig_file_path = sdf_dir / "model.sdf.orig"
    if not sdf_orig_file_path.exists():
        shutil.copy(sdf_file_path, sdf_orig_file_path)

    meshes_orig_dir = sdf_dir / "meshes_orig"
    if not meshes_orig_dir.exists():
        shutil.copytree(meshes_dir, meshes_orig_dir)

    # Find all subdirectories inside /meshes/ (each representing a link/rigid body)
    link_directories = [d for d in meshes_dir.iterdir() if d.is_dir()]

    # Dictionary to keep track of the new combined mesh URIs for the SDF update
    combined_mesh_uris = {}

    for link_dir in link_directories:
        link_name = link_dir.name
        mesh_file_paths = find_meshes(link_dir)

        if not mesh_file_paths:
            continue

        print(f"\n--- Processing directory (Link): {link_name} ---")

        # 1. Clear the Blender Scene
        bpy.ops.object.select_all(action="SELECT")
        bpy.ops.object.delete()

        # 2. Import all meshes in this directory
        imported_any = False
        for mesh_file_path in mesh_file_paths:
            # Skip if we somehow encounter a previously combined file
            if "combined" in mesh_file_path.name:
                continue

            print(f"  Importing: {mesh_file_path.name}")
            if mesh_file_path.suffix in [".glb", ".gltf"]:
                bpy.ops.import_scene.gltf(filepath=str(mesh_file_path))
            elif mesh_file_path.suffix == ".stl":
                bpy.ops.wm.stl_import(filepath=str(mesh_file_path))
            elif mesh_file_path.suffix == ".obj":
                bpy.ops.wm.obj_import(filepath=str(mesh_file_path))
            elif mesh_file_path.suffix == ".ply":
                bpy.ops.wm.ply_import(filepath=str(mesh_file_path))
            elif mesh_file_path.suffix == ".dae":
                bpy.ops.wm.collada_import(filepath=str(mesh_file_path))
            imported_any = True

        if not imported_any:
            continue

        # 3. Select only the imported meshes
        bpy.ops.object.select_all(action="DESELECT")
        selected_meshes = []
        for obj in bpy.context.scene.objects:
            if obj.type == "MESH":
                obj.select_set(True)
                selected_meshes.append(obj)

        if not selected_meshes:
            print(f"  No valid mesh data found in {link_name}. Skipping.")
            continue

        # 4. Join meshes
        print("  Joining meshes...")
        bpy.context.view_layer.objects.active = selected_meshes[0]
        bpy.ops.object.join()
        combined_mesh = bpy.context.active_object

        # --- SET UP & FORWARD AXES ---
        print("  Applying custom forward/up orientation...")

        bpy.ops.object.select_all(action="DESELECT")
        combined_mesh.select_set(True)
        bpy.context.view_layer.objects.active = combined_mesh

        combined_mesh.rotation_euler = (0.0, 0.0, math.radians(180.0))
        bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)

        combined_mesh.rotation_euler = (0.0, 0.0, math.radians(180.0))
        bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)

        print(
            f"  Before optimization: {len(combined_mesh.data.vertices)} verts, {len(combined_mesh.data.polygons)} polys"
        )

        # --- GEOMETRY CLEANUP (Done FIRST) ---
        print("  Cleaning geometry and fixing normals...")
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.select_all(action="SELECT")

        # Merge by distance (1mm threshold)
        bpy.ops.mesh.remove_doubles(threshold=0.001)
        # Dissolve zero-area faces
        bpy.ops.mesh.dissolve_degenerate(threshold=0.001)
        # Fix normals to point outside
        bpy.ops.mesh.normals_make_consistent(inside=False)

        bpy.ops.object.mode_set(mode="OBJECT")

        # --- DECIMATION (Done SECOND) ---
        modifier_collapse = combined_mesh.modifiers.new(
            name="decimate_collapse", type="DECIMATE"
        )
        modifier_collapse.decimate_type = "COLLAPSE"
        modifier_collapse.ratio = 0.5
        bpy.ops.object.modifier_apply(modifier="decimate_collapse")

        # --- SHADING (Done LAST) ---
        if combined_mesh.data.has_custom_normals:
            bpy.ops.mesh.customdata_custom_splitnormals_clear()

        try:
            bpy.ops.object.shade_smooth_by_angle(angle=math.radians(45.0))
        except AttributeError:
            bpy.ops.object.shade_smooth()
            combined_mesh.data.use_auto_smooth = True
            combined_mesh.data.auto_smooth_angle = math.radians(45.0)

        print(
            f"  After optimization: {len(combined_mesh.data.vertices)} verts, {len(combined_mesh.data.polygons)} polys"
        )

        # 6. Export the combined mesh based on selected format
        export_filename = f"{link_name}.{args.format}"
        export_mesh_file_path = link_dir / export_filename

        if args.format == "glb":
            bpy.ops.export_scene.gltf(
                filepath=str(export_mesh_file_path),
                export_format="GLB",
                export_materials="EXPORT",
                export_yup=True,
            )
        elif args.format == "dae":
            bpy.ops.wm.collada_export(
                filepath=str(export_mesh_file_path),
                apply_modifiers=True,
                export_mesh_type_selection="view",
            )

        # Save the URI for the SDF update
        relative_uri = f"meshes/{link_name}/{export_filename}"
        combined_mesh_uris[link_name] = relative_uri

        # 7. Cleanup: Delete the original small mesh files and mtls from the working directory
        print("  Cleaning up original files...")
        for mesh_file_path in mesh_file_paths:
            if mesh_file_path.exists():
                mesh_file_path.unlink()
            if mesh_file_path.suffix == ".obj":
                mtl_file_path = mesh_file_path.with_suffix(".mtl")
                if mtl_file_path.exists():
                    mtl_file_path.unlink()

    print("\n--- Updating model.sdf ---")
    tree = ET.parse(sdf_file_path)
    root = tree.getroot()

    base_name_to_uri = {}
    for dir_name, uri in combined_mesh_uris.items():
        # regex removes an optional '_' followed by any digits at the end of the string
        base_name = re.sub(r"_?\d+$", "", dir_name)
        base_name_to_uri[base_name] = uri

    for link in root.iter("link"):
        link_name = link.get("name")
        if not link_name:
            continue

        # Extract the base name from the link (e.g., "wheel_3" becomes "wheel")
        link_base_name = re.sub(r"_?\d+$", "", link_name)

        if link_base_name in base_name_to_uri:
            new_uri = base_name_to_uri[link_base_name]
            visuals = link.findall("visual")
            if visuals:
                first_visual = visuals[0]
                first_visual.set("name", f"{link_name}_visual")

                mesh_tag = first_visual.find("./geometry/mesh")
                if mesh_tag is not None:
                    uri_tag = mesh_tag.find("uri")
                    if uri_tag is not None:
                        uri_tag.text = new_uri

                for v in visuals[1:]:
                    link.remove(v)

            collisions = link.findall("collision")
            if collisions:
                first_coll = collisions[0]
                first_coll.set("name", f"{link_name}_collision")

                mesh_tag_c = first_coll.find("./geometry/mesh")
                if mesh_tag_c is not None:
                    uri_tag_c = mesh_tag_c.find("uri")
                    if uri_tag_c is not None:
                        uri_tag_c.text = new_uri

                for c in collisions[1:]:
                    link.remove(c)

    if hasattr(ET, "indent"):
        ET.indent(tree, space="  ", level=0)

    tree.write(sdf_file_path, encoding="utf-8", xml_declaration=True)
    print("Optimization and combination complete!")


if __name__ == "__main__":
    main()
