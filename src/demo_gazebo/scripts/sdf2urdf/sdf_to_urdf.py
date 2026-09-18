#!/usr/bin/env python3
"""Convert a tree-structured SDFormat model to URDF."""

from __future__ import annotations

import argparse
import logging
import math
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional
from xml.dom import minidom

LOGGER = logging.getLogger("sdf_to_urdf")
IDENTITY = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Transform:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    def matrix(self) -> tuple[tuple[float, ...], ...]:
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        return (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, self.x),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, self.y),
            (-sp, cp * sr, cp * cr, self.z),
            (0.0, 0.0, 0.0, 1.0),
        )

    @staticmethod
    def from_matrix(m: tuple[tuple[float, ...], ...]) -> "Transform":
        pitch = math.asin(max(-1.0, min(1.0, -m[2][0])))
        if abs(math.cos(pitch)) > 1e-9:
            roll = math.atan2(m[2][1], m[2][2])
            yaw = math.atan2(m[1][0], m[0][0])
        else:
            roll = 0.0
            yaw = math.atan2(-m[0][1], m[1][1])
        return Transform(m[0][3], m[1][3], m[2][3], roll, pitch, yaw)

    def __matmul__(self, other: "Transform") -> "Transform":
        a, b = self.matrix(), other.matrix()
        return Transform.from_matrix(
            tuple(
                tuple(sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4))
                for i in range(4)
            )
        )

    def inverse(self) -> "Transform":
        m = self.matrix()
        rotation = tuple(tuple(m[j][i] for j in range(3)) for i in range(3))
        translation = tuple(
            -sum(rotation[i][j] * m[j][3] for j in range(3)) for i in range(3)
        )
        return Transform.from_matrix(
            (
                (*rotation[0], translation[0]),
                (*rotation[1], translation[1]),
                (*rotation[2], translation[2]),
                (0.0, 0.0, 0.0, 1.0),
            )
        )


@dataclass(frozen=True)
class RawPose:
    value: Transform = Transform(*IDENTITY)
    relative_to: Optional[str] = None


@dataclass
class Geometry:
    kind: str
    values: dict[str, str]


@dataclass
class Visual:
    name: str
    pose: RawPose
    geometry: Geometry
    material: Optional[ET.Element] = None


@dataclass
class Link:
    name: str
    pose: RawPose
    inertial: Optional[ET.Element] = None
    inertial_pose: RawPose = field(default_factory=RawPose)
    visuals: list[Visual] = field(default_factory=list)
    world_pose: Optional[Transform] = None


@dataclass
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    pose: RawPose
    source: ET.Element
    world_pose: Optional[Transform] = None


@dataclass
class Model:
    name: str
    links: dict[str, Link]
    joints: dict[str, Joint]
    root_link: str

    def resolve_poses(self) -> None:
        frames: dict[str, Transform] = {"__model__": Transform(*IDENTITY)}
        resolving: set[str] = set()

        def resolve(name: str) -> Transform:
            if name in frames:
                return frames[name]
            if name in resolving:
                raise ValueError(f"Pose frame cycle detected at '{name}'")
            resolving.add(name)
            if name in self.links:
                entity = self.links[name]
                default_frame = "__model__"
            elif name in self.joints:
                entity = self.joints[name]
                # In SDFormat, a joint's default frame is the child link's frame, not the parent link's frame
                default_frame = entity.child
            else:
                raise ValueError(f"Pose references unknown frame '{name}'")
            reference = entity.pose.relative_to or default_frame
            world = resolve(reference) @ entity.pose.value
            entity.world_pose = world
            frames[name] = world
            resolving.remove(name)
            return world

        for name in self.links:
            resolve(name)
        for name in self.joints:
            resolve(name)

        for link in self.links.values():
            for visual in link.visuals:
                reference = visual.pose.relative_to or link.name
                # Temporarily store the absolute world pose
                visual.pose = RawPose(resolve(reference) @ visual.pose.value)
            if link.inertial is not None:
                reference = link.inertial_pose.relative_to or link.name
                # Temporarily store the absolute world pose
                link.inertial_pose = RawPose(
                    resolve(reference) @ link.inertial_pose.value
                )

        for link in self.links.values():
            if link.world_pose is None:
                raise ValueError(f"Could not resolve link '{link.name}'")
        LOGGER.debug(
            "Resolved %d links and %d joints", len(self.links), len(self.joints)
        )

    def normalize(self) -> None:
        self.resolve_poses()

        # 1. Determine the URDF coordinate frame for each link.
        # In URDF, a child link's frame is defined exactly by its incoming joint.
        urdf_frames: dict[str, Transform] = {}
        for link in self.links.values():
            if link.name == self.root_link:
                urdf_frames[link.name] = link.world_pose
            else:
                parent_joint = next(
                    j for j in self.joints.values() if j.child == link.name
                )
                if parent_joint.world_pose is None:
                    raise ValueError(f"Could not resolve joint '{parent_joint.name}'")
                urdf_frames[link.name] = parent_joint.world_pose

        # 2. Recalculate joint origins relative to their parent's aligned URDF frame.
        for joint in self.joints.values():
            parent_urdf_frame = urdf_frames[joint.parent]
            if joint.world_pose is None:
                raise ValueError(f"Could not resolve pose for joint '{joint.name}'")
            joint.pose = RawPose(parent_urdf_frame.inverse() @ joint.world_pose)

        # 3. Recalculate visuals and inertials relative to their link's aligned URDF frame.
        for link in self.links.values():
            urdf_frame = urdf_frames[link.name]
            for visual in link.visuals:
                visual.pose = RawPose(urdf_frame.inverse() @ visual.pose.value)
            if link.inertial is not None:
                link.inertial_pose = RawPose(
                    urdf_frame.inverse() @ link.inertial_pose.value
                )
            if link.name == self.root_link:
                link.pose = RawPose(Transform(*IDENTITY))
            else:
                parent_joint = next(
                    j for j in self.joints.values() if j.child == link.name
                )
                parent_urdf_frame = urdf_frames[parent_joint.parent]
                link.pose = RawPose(parent_urdf_frame.inverse() @ link.world_pose)

        LOGGER.debug("Normalized all link and joint poses to parent-link frames")


def _child(element: ET.Element, name: str) -> Optional[ET.Element]:
    return element.find(name)


def _text(element: Optional[ET.Element], default: str = "") -> str:
    return (element.text or "").strip() if element is not None else default


def parse_pose(element: Optional[ET.Element]) -> RawPose:
    if element is None or not _text(element):
        return RawPose()
    values = _text(element).split()
    if len(values) != 6:
        raise ValueError(
            f"Pose must contain six values, got {len(values)}: '{_text(element)}'"
        )
    try:
        pose = Transform(*(float(value) for value in values))
    except ValueError as exc:
        raise ValueError(f"Invalid numeric pose '{_text(element)}'") from exc
    return RawPose(pose, element.get("relative_to"))


def parse_geometry(element: ET.Element) -> Geometry:
    geometry = _child(element, "geometry")
    if geometry is None or len(geometry) == 0:
        raise ValueError(f"Visual '{element.get('name', '')}' has no geometry")
    shape = geometry[0]
    values = dict(shape.attrib)
    if shape.text and shape.text.strip():
        values["text"] = shape.text.strip()
    for child in shape:
        values[child.tag] = _text(child)
        values.update(
            {f"{child.tag}.{key}": value for key, value in child.attrib.items()}
        )
    return Geometry(shape.tag, values)


def parse_sdf(path: Path) -> Model:
    root = ET.parse(path).getroot()
    model = root.find("model")
    if model is None or not model.get("name"):
        raise ValueError("SDF must contain a named top-level model")
    links: dict[str, Link] = {}
    joints: dict[str, Joint] = {}
    for element in model.findall("link"):
        name = element.get("name")
        if not name or name in links:
            raise ValueError(f"Invalid or duplicate link name '{name}'")
        visuals = [
            Visual(
                v.get("name", ""),
                parse_pose(v.find("pose")),
                parse_geometry(v),
                v.find("material"),
            )
            for v in element.findall("visual")
        ]
        inertial = element.find("inertial")
        links[name] = Link(
            name,
            parse_pose(element.find("pose")),
            inertial,
            parse_pose(inertial.find("pose")) if inertial is not None else RawPose(),
            visuals,
        )
    for element in model.findall("joint"):
        name = element.get("name")
        parent = _text(element.find("parent"))
        child = _text(element.find("child"))
        if not name or name in joints:
            raise ValueError(f"Invalid or duplicate joint name '{name}'")
        if not parent or not child:
            raise ValueError(f"Joint '{name}' must have parent and child links")
        if parent not in links or child not in links:
            raise ValueError(
                f"Joint '{name}' references unknown link '{parent}' or '{child}'"
            )
        if child in {joint.child for joint in joints.values()}:
            raise ValueError(
                f"Link '{child}' has multiple parent joints; loops are unsupported"
            )
        joints[name] = Joint(
            name,
            element.get("type", "fixed"),
            parent,
            child,
            parse_pose(element.find("pose")),
            element,
        )
    children = {joint.child for joint in joints.values()}
    roots = [name for name in links if name not in children]
    if len(roots) != 1:
        raise ValueError(f"Expected exactly one root link, found {roots}")
    result = Model(model.get("name"), links, joints, roots[0])
    LOGGER.debug(
        "Parsed model '%s': %d links, %d joints, root '%s'",
        result.name,
        len(links),
        len(joints),
        result.root_link,
    )
    return result


def _number(value: float) -> str:
    return f"{value:.12g}"


def _origin(parent: ET.Element, pose: Transform) -> None:
    parent.set("xyz", " ".join(_number(value) for value in (pose.x, pose.y, pose.z)))
    parent.set(
        "rpy", " ".join(_number(value) for value in (pose.roll, pose.pitch, pose.yaw))
    )


def _copy_inertial(link_element: ET.Element, source: ET.Element, pose: RawPose) -> None:
    inertial = ET.SubElement(link_element, "inertial")

    # 1. Create an explicit <origin> child element
    if source.find("pose") is not None:
        origin = ET.SubElement(inertial, "origin")
        _origin(origin, pose.value)

    for tag in ("mass", "inertia"):
        child = source.find(tag)
        if child is not None:
            inertial.append(ET.fromstring(ET.tostring(child)))

    mass = inertial.find("mass")
    if mass is not None and "value" not in mass.attrib:
        mass.set("value", _text(mass))
        mass.text = None

    inertia = inertial.find("inertia")
    if inertia is not None:
        # 2. Map child tags (ixx, iyy, etc.) to attributes on the <inertia> element itself
        for child in list(inertia):
            val = child.attrib.get("value")
            if val is None and child.text:
                val = child.text.strip()

            if val is not None:
                inertia.set(child.tag, val)

            inertia.remove(child)
        inertia.text = None


def export_urdf(model: Model, output: Path) -> None:
    root = ET.Element("robot", {"name": model.name})
    for link in model.links.values():
        target = ET.SubElement(root, "link", {"name": link.name})
        if link.inertial is not None:
            _copy_inertial(target, link.inertial, link.inertial_pose)

        for visual in link.visuals:
            target_visual = ET.SubElement(target, "visual")

            print(f"Exporting visual '{visual.name}'")

            origin = ET.SubElement(target_visual, "origin")
            _origin(origin, visual.pose.value)

            geometry = ET.SubElement(target_visual, "geometry")
            shape = ET.SubElement(geometry, visual.geometry.kind)
            values = visual.geometry.values
            if visual.geometry.kind == "mesh":
                if "uri" in values:
                    shape.set("filename", values["uri"])
                if "scale" in values:
                    shape.set("scale", values["scale"])
            else:
                for key, value in values.items():
                    if "." not in key and key != "text":
                        shape.set(key, value)
            # if visual.material is not None:
            #     target_visual.append(ET.fromstring(ET.tostring(visual.material)))

    for joint in model.joints.values():
        kind = joint.kind
        limit = joint.source.find("axis/limit")
        axis = joint.source.find("axis/xyz")

        if kind == "revolute" and limit is None:
            LOGGER.warning(
                "Joint '%s' is revolute but has no limit; converting to continuous",
                joint.name,
            )
            kind = "continuous"

        target = ET.SubElement(root, "joint", {"name": joint.name, "type": kind})
        origin = ET.SubElement(target, "origin")
        _origin(origin, joint.pose.value)
        ET.SubElement(target, "parent", {"link": joint.parent})
        ET.SubElement(target, "child", {"link": joint.child})

        if axis is not None and kind != "fixed":
            ET.SubElement(target, "axis", {"xyz": _text(axis)})

        if limit is not None and kind not in {"fixed", "continuous"}:
            ET.SubElement(
                target, "limit", {key: value for key, value in limit.attrib.items()}
            )

    xml = minidom.parseString(ET.tostring(root, encoding="unicode")).toprettyxml(
        indent="  "
    )
    output.write_text(
        "\n".join(line for line in xml.splitlines() if line.strip()) + "\n"
    )
    LOGGER.info("Wrote URDF: %s", output)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_sdf", type=Path)
    parser.add_argument("output_urdf", type=Path)
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s: %(message)s",
    )
    try:
        model = parse_sdf(args.input_sdf)
        model.normalize()
        export_urdf(model, args.output_urdf)
    except (OSError, ET.ParseError, ValueError) as exc:
        LOGGER.error("Conversion failed: %s", exc)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
