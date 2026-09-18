import sys
import xml.etree.ElementTree as ET
from xml.dom import minidom
import os
import shutil


def replace_continuous_joints_with_revolute(root):
    for joint in root.iter("joint"):
        if joint.get("type") == "continuous":
            joint.set("type", "revolute")
    return root


def simplify_model(input_file, output_file, meshes_directory):
    """
    This script finds all reused meshes in the model and simplifies them to only use one mesh for all.

    Example:
     - Meshes "suspension_arm_1" and "suspension_arm_2" are used in the model. Since they have the same name, except for the number, we can assume they are the same mesh.
     - The script will keep "suspension_arm_1" as the main mesh and replace all occurrences of "suspension_arm_2" with "suspension_arm_1".

    Arguments:
        - input_file: path to the input SDF file
        - output_file: path to the output SDF file with simplified meshes
        - meshes_directory: path to the directory containing the mesh files. In here for every instance, a directory with the instance name is expected:
            - meshes/
                - suspension_arm_1/
                - suspension_arm_2/
    """

    meshes_to_remove = []

    try:
        tree = ET.parse(input_file)
        root = tree.getroot()

        replaced_mesh_name_count = 0

        for link in root.iter("link"):
            visual_tags = link.findall("visual")

            extracted_names = []

            for visual in visual_tags:
                extracted_names.append(visual.get("name"))

            if len(extracted_names) == 0:
                continue

            # SDFusion seperates the mesh name from the instance name with a double underscore.
            mesh_name = extracted_names[0].split("__")[0]

            # check if the same mesh name is used in all visual tags of this link
            if not all(mesh_name == name.split("__")[0] for name in extracted_names):
                raise ValueError(
                    f"Error: Link {link.get('name')} has visual tags with different mesh names: {extracted_names}"
                )

            instance_num = int(mesh_name.split("_")[-1])

            if instance_num > 1:
                main_mesh_name = mesh_name[:-1] + str(1)

                for visual in visual_tags:
                    for geometry in visual.findall("geometry"):
                        for mesh in geometry.findall("mesh"):
                            for uri in mesh.findall("uri"):
                                uri.text = uri.text.replace(mesh_name, main_mesh_name)
                                replaced_mesh_name_count += 1
                                meshes_to_remove.append(mesh_name)

        root = replace_continuous_joints_with_revolute(root)

        xml_str = minidom.parseString(ET.tostring(root)).toprettyxml(indent="  ")
        clean_xml = "\n".join([line for line in xml_str.split("\n") if line.strip()])
        with open(output_file, "w") as f:
            f.write(clean_xml)

        # Remove the mesh directories of the removed meshes
        # ask the user for confirmation before deleting
        if len(meshes_to_remove) > 0 and len(meshes_directory) > 0:
            print(f"The following meshes will be removed: {set(meshes_to_remove)}")
            confirm = input("Do you want to proceed? (y/n): ")
            if confirm.lower() == "y":
                for mesh_name in set(meshes_to_remove):
                    mesh_dir = f"{meshes_directory}/{mesh_name}"
                    if os.path.exists(mesh_dir):
                        shutil.rmtree(mesh_dir)
                        print(f"Removed mesh directory: {mesh_dir}")
                    else:
                        print(f"Mesh directory not found, skipping: {mesh_dir}")
            else:
                print("Aborting mesh removal.")

        print(f"Success! Replaced {replaced_mesh_name_count} mesh names.")
        print(f"Cleaned model saved to: {output_file}")

    except FileNotFoundError:
        print(f"Error: The file '{input_file}' was not found.")
    except ET.ParseError:
        print(f"Error: Could not parse '{input_file}'. Check if it is valid XML/SDF.")


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print(
            "Usage: python3 simplify_model.py <input_model.sdf> <output_model.sdf> <meshes_directory>"
        )
        sys.exit(1)

    input_sdf = sys.argv[1]
    output_sdf = sys.argv[2]
    meshes_directory = sys.argv[3]

    simplify_model(input_sdf, output_sdf, meshes_directory)
