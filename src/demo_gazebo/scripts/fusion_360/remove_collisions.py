import sys
import xml.etree.ElementTree as ET
from xml.dom import minidom


def remove_all_collisions(input_file, output_file):
    try:
        # 1. Parse the input SDF file
        tree = ET.parse(input_file)
        root = tree.getroot()

        removed_count = 0

        # 2. Find all 'link' elements (recursively)
        # We iterate over every link in the file to ensure we catch nested models too
        for link in root.iter("link"):
            # Find all collision tags inside this link
            collisions = link.findall("collision")

            # Remove them one by one
            for collision in collisions:
                link.remove(collision)
                removed_count += 1

        # 3. Save the result
        # We use minidom to make the output XML "pretty" (indented correctly)
        xml_str = minidom.parseString(ET.tostring(root)).toprettyxml(indent="  ")

        # Remove empty lines that minidom sometimes inserts
        clean_xml = "\n".join([line for line in xml_str.split("\n") if line.strip()])

        with open(output_file, "w") as f:
            f.write(clean_xml)

        print(f"Success! Removed {removed_count} collision elements.")
        print(f"Cleaned model saved to: {output_file}")

    except FileNotFoundError:
        print(f"Error: The file '{input_file}' was not found.")
    except ET.ParseError:
        print(f"Error: Could not parse '{input_file}'. Check if it is valid XML/SDF.")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 remove_collisions.py <input_model.sdf> <output_model.sdf>")
        sys.exit(1)

    input_sdf = sys.argv[1]
    output_sdf = sys.argv[2]

    remove_all_collisions(input_sdf, output_sdf)

