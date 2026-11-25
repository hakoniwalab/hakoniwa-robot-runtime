"""Add the iiwa14 MJCF's physical properties to its kinematic URDF."""

from copy import deepcopy
import math
import xml.etree.ElementTree as ET


def rpy(quaternion):
    w, x, y, z = map(float, quaternion.split())
    norm = math.hypot(w, x, y, z)
    w, x, y, z = (v / norm for v in (w, x, y, z))
    angles = (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
              math.asin(max(-1, min(1, 2 * (w * y - z * x)))),
              math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return " ".join(map(str, angles))


def sphere(link, name, position, radius, color):
    collision = ET.SubElement(link, "collision", name=name)
    ET.SubElement(collision, "origin", xyz=position, rpy="0 0 0")
    ET.SubElement(ET.SubElement(collision, "geometry"), "sphere", radius=radius)
    visual = ET.SubElement(link, "visual", name=name)
    visual.extend(deepcopy(child) for child in collision)
    ET.SubElement(ET.SubElement(visual, "material", name=name), "color", rgba=color)


def physical_urdf(kinematic_path, mjcf_path):
    robot = ET.parse(kinematic_path).getroot()
    bodies = {body.get("name"): body for body in ET.parse(mjcf_path).iterfind(".//worldbody//body")}
    for link in robot.findall("link"):
        name = link.attrib["name"]
        body = bodies[name]
        source = body.find("inertial")
        if source is None:
            raise ValueError(f"MJCF link {name} needs explicit inertia")
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", xyz=source.attrib["pos"],
                      rpy=rpy(source.get("quat", "1 0 0 0")))
        ET.SubElement(inertial, "mass", value=source.attrib["mass"])
        xx, yy, zz = source.attrib["diaginertia"].split()
        ET.SubElement(inertial, "inertia", ixx=xx, iyy=yy, izz=zz, ixy="0", ixz="0", iyz="0")
        for index, geom in enumerate(body.findall("geom[@class='collision']")):
            if geom.get("type", "sphere") != "sphere":
                raise ValueError(f"MJCF link {name} has a non-spherical collision geom")
            sphere(link, f"{name}_sphere_{index}", geom.get("pos", "0 0 0"), geom.attrib["size"], "0.39 0.45 0.51 1")
        gravity = float(body.get("gravcomp", "0"))
        if gravity not in (0, 1):
            raise ValueError(f"Cannot map partial MJCF gravity compensation on {name}")
        ET.SubElement(ET.SubElement(robot, "gazebo", reference=name), "gravity").text = (
            "false" if gravity == 1 else "true")
    sphere(robot.find("link[@name='tool']"), "tool_tip", "0 0 0.098", "0.010", "0.9 0.9 0.2 1")
    return robot


def apply_noise(robot, noise_sdf):
    target = robot.find(".//sensor[@name='wrist_ft']/force_torque")
    source = ET.parse(noise_sdf).find(".//sensor[@type='force_torque']/force_torque")
    for quantity in ("force", "torque"):
        target.append(deepcopy(source.find(quantity)))
