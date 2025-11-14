#!/usr/bin/env python3
"""
Script to generate modelone_sim.xml based on humanoid.xml
"""

import os
import xml.etree.ElementTree as ET
from xml.dom import minidom

# Joint definitions
WAIST_JOINTS = ['body_yaw']
HEAD_JOINTS = ['head_yaw', 'head_pitch']
HAND_JOINTS = []
ARM_JOINTS = ['right_arm_flex', 'right_arm_ab', 'right_arm_rot', 'right_arm_elbow',
              'left_arm_flex', 'left_arm_ab', 'left_arm_rot', 'left_arm_elbow']
LIMITED_ARM_JOINTS = ['right_arm_flex', 'left_arm_flex', 'right_arm_elbow', 'left_arm_elbow']
LEG_JOINTS = ['right_hip_flex', 'right_hip_ab', 'right_hip_rot', 'right_knee', 'right_ankle', 'right_ankle_lower',
              'left_hip_flex', 'left_hip_ab', 'left_hip_rot', 'left_knee', 'left_ankle', 'left_ankle_lower']
FOOT_JOINTS = ['left_foot_pitch', 'left_foot_roll', 'right_foot_pitch', 'right_foot_roll']
BALL_JOINTS = []

# All controlled joints - legs and limited arms (ball joints are passive)
ALL_CONTROLLED_JOINTS = LEG_JOINTS + LIMITED_ARM_JOINTS

# All joints that should be included in the model (controlled + passive + foot)
ALL_MODEL_JOINTS = ALL_CONTROLLED_JOINTS + FOOT_JOINTS

# Position limits from move.py
POSITION_LIMITS = {
    'left_hip_flex':     {'min': -0.9, 'max': 0.95},
    'left_hip_ab':       {'min': -0.15,'max': 0.13},
    'left_hip_rot':      {'min': -0.5, 'max': 0.5},
    'left_knee':         {'min': -1.2, 'max': 0.0},
    'left_ankle':        {'min': -0.6, 'max': 0.6},
    'left_ankle_lower':  {'min': -0.6, 'max': 0.6},

    'right_hip_flex':    {'min': -0.9, 'max': 0.95},
    'right_hip_ab':      {'min': -0.13,'max': 0.15},
    'right_hip_rot':     {'min': -0.5, 'max': 0.5},
    'right_knee':        {'min': -1.2, 'max': 0.0},
    'right_ankle':       {'min': -0.6, 'max': 0.6},
    'right_ankle_lower': {'min': -0.6, 'max': 0.6},

    'left_arm_flex':     {'min': -0.4, 'max': 1.4},
    'left_arm_ab':       {'min': -0.9, 'max': 0.1},
    'left_arm_rot':      {'min': -1.0, 'max': 1.0},
    'left_arm_elbow':    {'min': -0.0, 'max': 1.4},

    'right_arm_flex':    {'min': -0.4, 'max': 1.4},
    'right_arm_ab':      {'min': -0.9, 'max': 0.1},
    'right_arm_rot':     {'min': -1.0, 'max': 1.0},
    'right_arm_elbow':   {'min': -0.0, 'max': 1.4},

    'head_yaw':          {'min': -1.5, 'max': 1.5},
    'head_pitch':        {'min': -0.9, 'max': 0.0},
    'body_yaw':          {'min': -0.4, 'max': 0.4},
    
    # Ball joints - these are typically unactuated but we include them for completeness
    'left_ankle_lever':      {'min': 0.0, 'max': 1.570796},
    'left_ankle_lever_lower': {'min': 0.0, 'max': 1.570796},
    'right_ankle_lever':     {'min': 0.0, 'max': 1.570796},
    'right_ankle_lever_lower': {'min': 0.0, 'max': 1.570796},
    
    # Foot joints - these are typically unactuated but allow foot movement
    'left_foot_pitch':       {'min': -1.047198, 'max': 1.047198},
    'left_foot_roll':        {'min': -1.047198, 'max': 1.047198},
    'right_foot_pitch':      {'min': -1.047198, 'max': 1.047198},
    'right_foot_roll':       {'min': -1.047198, 'max': 1.047198},
}

# Define joint to motor class mapping
joint_motor_classes = {
    # Hip flexion joints - motor_04 (highest torque)
    'left_hip_flex': 'motor_04',
    'right_hip_flex': 'motor_04',
    
    # Hip abduction, rotation, knee, body_yaw - motor_03 (medium torque)
    'left_hip_ab': 'motor_03',
    'left_hip_rot': 'motor_03',
    'left_knee': 'motor_03',
    'right_hip_ab': 'motor_03',
    'right_hip_rot': 'motor_03',
    'right_knee': 'motor_03',
    'body_yaw': 'motor_03',
    
    # Ankle joints and head joints - motor_02 (lower torque)
    'left_ankle': 'motor_02',
    'left_ankle_lower': 'motor_02',
    'right_ankle': 'motor_02',
    'right_ankle_lower': 'motor_02',
    'head_yaw': 'motor_02',
    'head_pitch': 'motor_02',
    
    # Arm joints - motor_02 and motor_03
    'left_arm_flex': 'motor_03',
    'left_arm_ab': 'motor_02',
    'left_arm_rot': 'motor_02',
    'left_arm_elbow': 'motor_02',
    'right_arm_flex': 'motor_03',
    'right_arm_ab': 'motor_02',
    'right_arm_rot': 'motor_02',
    'right_arm_elbow': 'motor_02',
    
    # Foot pitch/roll controlled by ankle actuators mapping
    'left_foot_pitch': 'motor_02',
    'left_foot_roll': 'motor_02',
    'right_foot_pitch': 'motor_02',
    'right_foot_roll': 'motor_02',
}

def generate_modelone_sim():
    """Generate modelone_sim.xml"""
    
    # Read the original humanoid.xml
    humanoid_path = "humanoid.xml"
    if not os.path.exists(humanoid_path): raise FileNotFoundError(f"Could not find {humanoid_path}")
    print("Reading humanoid.xml...")
    tree = ET.parse(humanoid_path)
    root = tree.getroot()

    # Create new root element for modelone_sim
    new_root = ET.Element("mujoco")
    new_root.set("model", "modelone")
    
    # Add compiler settings
    compiler = ET.SubElement(new_root, "compiler")
    compiler.set("autolimits", "true")
    compiler.set("angle", "radian")
    compiler.set("meshdir", "assets")
    
    # Add visual settings
    visual = ET.SubElement(new_root, "visual")
    headlight = ET.SubElement(visual, "headlight")
    headlight.set("ambient", "0.3 0.3 0.35")
    headlight.set("diffuse", "0.35 0.35 0.39")
    headlight.set("specular", "0 0 0")
    rgba = ET.SubElement(visual, "rgba")
    rgba.set("haze", "0.7 0.7 0.8 0.5")
    global_elem = ET.SubElement(visual, "global")
    global_elem.set("offwidth", "1920")
    global_elem.set("offheight", "1080")
    
    # Add optimized solver settings for performance
    option = ET.SubElement(new_root, "option")
    option.set("timestep", "0.001")
    option.set("iterations", "50")
    option.set("tolerance", "1e-10")
    option.set("solver", "Newton")
    option.set("jacobian", "dense")
    option.set("cone", "pyramidal")
    
    # Add default settings
    default = ET.SubElement(new_root, "default")
    root_default = ET.SubElement(default, "default")
    root_default.set("class", "/")
    humanoid_default = ET.SubElement(root_default, "default")
    humanoid_default.set("class", "humanoid")

    # Add joint defaults
    joint_default = ET.SubElement(humanoid_default, "joint")
    joint_default.set("armature", "0.1")
    joint_default.set("damping", "1.0")
    joint_default.set("frictionloss", "0.1")

    # Add position defaults
    position_default = ET.SubElement(humanoid_default, "position")
    position_default.set("kp", "50")
    position_default.set("dampratio", "1")

    # Add motor class definitions with different torque limits and PD gains
    motor_00_default = ET.SubElement(root_default, "default")
    motor_00_default.set("class", "motor_00")
    motor_00_joint = ET.SubElement(motor_00_default, "joint")
    motor_00_joint.set("armature", "0.01")
    motor_00_joint.set("frictionloss", "0.1")
    motor_00_joint.set("actuatorfrcrange", "-14.0 14.0")
    motor_00_position = ET.SubElement(motor_00_default, "position")
    motor_00_position.set("kp", "20.0")
    motor_00_position.set("dampratio", "1.0")
    motor_00_position.set("forcelimited", "true")
    motor_00_position.set("forcerange", "-14.0 14.0")

    motor_02_default = ET.SubElement(root_default, "default")
    motor_02_default.set("class", "motor_02")
    motor_02_joint = ET.SubElement(motor_02_default, "joint")
    motor_02_joint.set("armature", "0.01")
    motor_02_joint.set("frictionloss", "0.1")
    motor_02_joint.set("actuatorfrcrange", "-17.0 17.0")
    motor_02_position = ET.SubElement(motor_02_default, "position")
    motor_02_position.set("kp", "30.0")
    motor_02_position.set("dampratio", "1.0")
    motor_02_position.set("forcelimited", "true")
    motor_02_position.set("forcerange", "-17.0 17.0")

    motor_03_default = ET.SubElement(root_default, "default")
    motor_03_default.set("class", "motor_03")
    motor_03_joint = ET.SubElement(motor_03_default, "joint")
    motor_03_joint.set("armature", "0.01")
    motor_03_joint.set("frictionloss", "0.1")
    motor_03_joint.set("actuatorfrcrange", "-60.0 60.0")
    motor_03_position = ET.SubElement(motor_03_default, "position")
    motor_03_position.set("kp", "80.0")
    motor_03_position.set("dampratio", "1.0")
    motor_03_position.set("forcelimited", "true")
    motor_03_position.set("forcerange", "-60.0 60.0")

    motor_04_default = ET.SubElement(root_default, "default")
    motor_04_default.set("class", "motor_04")
    motor_04_joint = ET.SubElement(motor_04_default, "joint")
    motor_04_joint.set("armature", "0.01")
    motor_04_joint.set("frictionloss", "0.1")
    motor_04_joint.set("actuatorfrcrange", "-120.0 120.0")
    motor_04_position = ET.SubElement(motor_04_default, "position")
    motor_04_position.set("kp", "160.0")
    motor_04_position.set("dampratio", "1.0")
    motor_04_position.set("forcelimited", "true")
    motor_04_position.set("forcerange", "-120.0 120.0")

    # Add visual defaults
    visual_default = ET.SubElement(humanoid_default, "default")
    visual_default.set("class", "visual")
    visual_geom = ET.SubElement(visual_default, "geom")
    visual_geom.set("type", "mesh")
    visual_geom.set("contype", "0")
    visual_geom.set("conaffinity", "0")
    visual_geom.set("group", "2")

    # Add collision defaults
    collision_default = ET.SubElement(humanoid_default, "default")
    collision_default.set("class", "collision")
    collision_geom = ET.SubElement(collision_default, "geom")
    collision_geom.set("condim", "3")
    collision_geom.set("group", "0")
    collision_geom.set("rgba", "1 0 0 0")

    # Add assets (copy from original)
    print("Copying assets...")
    assets = root.find("asset")
    if assets is not None: new_root.append(assets)
    
    # Add skybox texture
    skybox_texture = ET.SubElement(new_root.find("asset"), "texture")
    skybox_texture.set("type", "skybox")
    skybox_texture.set("builtin", "gradient")
    skybox_texture.set("rgb1", "0.6 0.67 0.82")
    skybox_texture.set("rgb2", "0.74 0.78 0.9")
    skybox_texture.set("width", "512")
    skybox_texture.set("height", "1024")
    
    # Add floor texture and material
    floor_texture = ET.SubElement(new_root.find("asset"), "texture")
    floor_texture.set("name", "floor")
    floor_texture.set("type", "2d")
    floor_texture.set("file", "assets/textures/floor.png")
    floor_material = ET.SubElement(new_root.find("asset"), "material")
    floor_material.set("name", "floor")
    floor_material.set("texture", "floor")
    floor_material.set("texrepeat", "0.7 0.7")
    floor_material.set("reflectance", "0")
        
    # Create worldbody
    worldbody = ET.SubElement(new_root, "worldbody")
    
    # Copy and modify the main body from original
    print("Processing main body...")
    original_body = root.find(".//body[@name='body']")
    if original_body is not None:
        # Create new body with modified position
        new_body = ET.SubElement(worldbody, "body")
        new_body.set("name", "body")
        new_body.set("pos", "0 0 0.88")
        new_body.set("quat", "1 0 0 0")
        new_body.set("childclass", "humanoid")
        
        # Add freejoint for root
        freejoint = ET.SubElement(new_body, "freejoint")
        freejoint.set("name", "root")
        
        # Add IMU body for visualization
        imu_body = ET.SubElement(new_body, "body")
        imu_body.set("name", "imu")
        imu_body.set("pos", "-0.1 0.015 0.19")
        imu_body.set("quat", "1.0 0.0 0.0 0.0")

        imu_inertial = ET.SubElement(imu_body, "inertial")
        imu_inertial.set("pos", "-0.003012 0 0.002999")
        imu_inertial.set("quat", "1.0 0.0 0.0 0.0")
        imu_inertial.set("mass", "0.005700")
        imu_inertial.set("diaginertia", "1e-06 1e-06 2e-06")

        imu_geom = ET.SubElement(imu_body, "geom")
        imu_geom.set("name", "imu_visual")
        imu_geom.set("pos", "0 0 0")
        imu_geom.set("quat", "1.0 0.0 0.0 0.0")
        imu_geom.set("type", "box")
        imu_geom.set("size", "0.01 0.02 0.003")
        imu_geom.set("rgba", "0 1 0 1")
        imu_geom.set("class", "visual")

        imu_site = ET.SubElement(imu_body, "site")
        imu_site.set("name", "imu_site")
        imu_site.set("pos", "0 0 0")
        imu_site.set("quat", "1 0 0 0")

        # Copy inertial properties
        inertial = original_body.find("inertial")
        if inertial is not None: new_body.append(inertial)

        # Copy geoms from the main body but remove problematic collision geoms
        remove_collision_geoms = [
            'body_collision',
            'body_lower_collision',
            'hip_abd_mount_collision',
            'leg_upper_upper_left_collision',
            'hip_abd_mount_mirror_collision',
            'leg_upper_upper_right_collision',
            'neck_collision',
            'right_arm_shoulder_mount_collision',
            'right_arm_top_collision',
            'right_arm_upper_collision',
            'forearm_right_collision',
            'left_arm_shoulder_mount_collision',
            'left_arm_top_collision',
            'left_arm_upper_collision',
            'forearm_left_collision',
            'leg_upper_left_collision',
            'leg_upper_right_collision',
            'leg_lower_left_collision',
            'leg_lower_right_collision',
            'left_foot_collision',
            'right_foot_collision',
            'body_collision_mesh',
            'body_mesh',
            'torso_collision',
            'torso_mesh'
        ]
        for geom in original_body.findall("geom"):
            geom_name = geom.get("name", "")
            mesh_name = geom.get("mesh", "")
            
            # Skip problematic collision geoms
            if (geom_name in remove_collision_geoms or 
                mesh_name in remove_collision_geoms or
                "body_collision" in mesh_name or
                "body_collision" in geom_name):
                print(f"    Skipping problematic main body collision geom: {geom_name} (mesh: {mesh_name})")
                continue
            
            # Add geom to new body
            new_geom = ET.SubElement(new_body, "geom")
            for attr_name, attr_value in geom.attrib.items(): new_geom.set(attr_name, attr_value)
        
        # Process all child bodies recursively
        print("Processing child bodies...")
        process_child_bodies(original_body, new_body)
    
    # Add light
    light = ET.SubElement(worldbody, "light")
    light.set("pos", "0 0 3.5")
    light.set("dir", "0 0 -1")
    light.set("directional", "true")
    
    # Add floor (moved to 0,0,0)
    floor_geom = ET.SubElement(worldbody, "geom")
    floor_geom.set("name", "floor")
    floor_geom.set("size", "0 0 0.1")
    floor_geom.set("pos", "0 0 0")
    floor_geom.set("type", "plane")
    floor_geom.set("material", "floor")
    floor_geom.set("friction", "0.8 0.005 0.0001")
    floor_geom.set("solref", "0.01 1")
    floor_geom.set("solimp", "0.9 0.95 0.002")
    floor_geom.set("condim", "3")
    
    # Add cameras
    lookat_camera = ET.SubElement(worldbody, "camera")
    lookat_camera.set("name", "lookat")
    lookat_camera.set("mode", "targetbodycom")
    lookat_camera.set("target", "body")
    lookat_camera.set("pos", "0.65 -0.6 1.4")

    # Add headcam camera
    headcam_camera = ET.SubElement(worldbody, "camera")
    headcam_camera.set("name", "headcam")
    headcam_camera.set("mode", "fixed")
    headcam_camera.set("pos", "0.0 -0.02 1.27")
    headcam_camera.set("euler", "-1.7 0 3.14159")
    headcam_camera.set("fovy", "57")
    
    # Copy contact exclusions from scene.xml and drop linkage excludes
    print("Adding contact exclusions...")
    contact_section = ET.SubElement(new_root, "contact")
    scene_tree = ET.parse('scene.xml')
    scene_root = scene_tree.getroot()
    scene_contact = scene_root.find("contact")
    if scene_contact is not None:
        for exclude in scene_contact.findall("exclude"):
            b1 = exclude.get("body1", "")
            b2 = exclude.get("body2", "")
            if "linkage" in b1 or "linkage" in b2 or "lever" in b1 or "lever" in b2:
                continue
            new_exclude = ET.SubElement(contact_section, "exclude")
            for attr_name, attr_value in exclude.attrib.items():
                new_exclude.set(attr_name, attr_value)

    # No equality constraints for ankle linkage

    # Add actuators for all joints with position control and motor classes
    print("Adding actuators...")
    actuator = ET.SubElement(new_root, "actuator")

    # Create actuators for all joints
    global joint_motor_classes
    controlled_joints = ALL_CONTROLLED_JOINTS
    replace_to_feet = {
        'left_ankle': 'left_foot_pitch',
        'left_ankle_lower': 'left_foot_roll',
        'right_ankle': 'right_foot_pitch',
        'right_ankle_lower': 'right_foot_roll',
    }
    for joint_name in controlled_joints:
        actuated_joint = replace_to_feet.get(joint_name, joint_name)
        motor_class = joint_motor_classes.get(actuated_joint, joint_motor_classes.get(joint_name, 'motor_03'))
        position_actuator = ET.SubElement(actuator, "position")
        actuator_name = f"{actuated_joint}_ctrl" if joint_name in replace_to_feet else f"{joint_name}_ctrl"
        position_actuator.set("name", actuator_name)
        position_actuator.set("joint", actuated_joint)
        position_actuator.set("class", motor_class)
        
        # Set ctrlrange to match joint range
        if actuated_joint in POSITION_LIMITS:
            limits = POSITION_LIMITS[actuated_joint]
            if 'right' in actuated_joint:
                min_val = -limits['max']  # Flip sign and use max as min
                max_val = -limits['min']  # Flip sign and use min as max
            else:
                min_val = limits['min']
                max_val = limits['max']
            position_actuator.set("ctrlrange", f"{min_val} {max_val}")
    
    # Add sensors
    print("Adding sensors...")
    sensor = ET.SubElement(new_root, "sensor")
    
    # IMU sensors
    gyro = ET.SubElement(sensor, "gyro")
    gyro.set("site", "imu_site")
    gyro.set("name", "gyro")
    gyro.set("noise", "0.01")
    
    velocimeter = ET.SubElement(sensor, "velocimeter")
    velocimeter.set("site", "imu_site")
    velocimeter.set("name", "local_linvel")
    
    accelerometer = ET.SubElement(sensor, "accelerometer")
    accelerometer.set("site", "imu_site")
    accelerometer.set("name", "accelerometer")
    accelerometer.set("noise", "0.01")
    
    imu_quat = ET.SubElement(sensor, "framequat")
    imu_quat.set("objtype", "site")
    imu_quat.set("objname", "imu_site")
    imu_quat.set("name", "imu_site_quat")
    imu_quat.set("noise", "0.01")
    
    imu_angvel = ET.SubElement(sensor, "frameangvel")
    imu_angvel.set("objtype", "site")
    imu_angvel.set("objname", "imu_site")
    imu_angvel.set("name", "imu_site_angvel")
    imu_angvel.set("noise", "0.01")
    
    imu_linvel = ET.SubElement(sensor, "framelinvel")
    imu_linvel.set("objtype", "site")
    imu_linvel.set("objname", "imu_site")
    imu_linvel.set("name", "imu_site_linvel")
    
    imu_pos = ET.SubElement(sensor, "framepos")
    imu_pos.set("objtype", "site")
    imu_pos.set("objname", "imu_site")
    imu_pos.set("name", "imu_site_pos")
    
    # Foot sensors
    l_foot_linvel = ET.SubElement(sensor, "framelinvel")
    l_foot_linvel.set("objtype", "site")
    l_foot_linvel.set("objname", "l_foot")
    l_foot_linvel.set("name", "l_foot_global_linvel")
    
    r_foot_linvel = ET.SubElement(sensor, "framelinvel")
    r_foot_linvel.set("objtype", "site")
    r_foot_linvel.set("objname", "r_foot")
    r_foot_linvel.set("name", "r_foot_global_linvel")
    
    l_foot_upvector = ET.SubElement(sensor, "framexaxis")
    l_foot_upvector.set("objtype", "site")
    l_foot_upvector.set("objname", "l_foot")
    l_foot_upvector.set("name", "l_foot_upvector")
    
    r_foot_upvector = ET.SubElement(sensor, "framexaxis")
    r_foot_upvector.set("objtype", "site")
    r_foot_upvector.set("objname", "r_foot")
    r_foot_upvector.set("name", "r_foot_upvector")
    
    l_foot_pos = ET.SubElement(sensor, "framepos")
    l_foot_pos.set("objtype", "site")
    l_foot_pos.set("objname", "l_foot")
    l_foot_pos.set("name", "l_foot_pos")
    
    r_foot_pos = ET.SubElement(sensor, "framepos")
    r_foot_pos.set("objtype", "site")
    r_foot_pos.set("objname", "r_foot")
    r_foot_pos.set("name", "r_foot_pos")
    
    # Clean up quaternion values by rounding
    def clean_quat_values(element):
        """Clean up quaternion values by rounding to 7 decimal places."""
        if element.tag in ['body', 'geom'] and 'quat' in element.attrib:
            quat_str = element.attrib['quat']
            quat_parts = quat_str.split()
            if len(quat_parts) == 4:
                cleaned_parts = []
                for part in quat_parts:
                    try:
                        val = float(part)
                        rounded_val = round(val, 5)
                        if rounded_val == 0.0: cleaned_parts.append('0')
                        else: cleaned_parts.append(str(rounded_val))
                    except ValueError:
                        cleaned_parts.append(part)
                element.attrib['quat'] = ' '.join(cleaned_parts)
        
        # Recursively process children
        for child in element:
            clean_quat_values(child)
    
    # Apply quaternion cleanup
    clean_quat_values(new_root)
    
    # Write the new XML file
    output_path = os.path.join(".", "modelone_sim.xml")
    print(f"Writing {output_path}...")
    
    # Pretty print the XML
    rough_string = ET.tostring(new_root, 'unicode')
    reparsed = minidom.parseString(rough_string)
    pretty_xml = reparsed.toprettyxml(indent="  ")
    
    # Remove empty lines and fix formatting
    lines = [line for line in pretty_xml.split('\n') if line.strip()]
    pretty_xml = '\n'.join(lines)
    
    with open(output_path, 'w') as f:
        f.write(pretty_xml)
        print(f"Successfully generated {output_path}")
    return output_path


def process_child_bodies(original_body, new_parent):
    """Recursively process all child bodies, but skip arm-related bodies"""
    
    for child in original_body.findall("body"):
        body_name = child.get("name")
        if body_name and ("linkage" in body_name or "lever" in body_name):
            print(f"  Skipping body: {body_name}")
            continue
        print(f"  Processing body: {body_name}")
        
        # Create new body
        new_body = ET.SubElement(new_parent, "body")
        new_body.set("name", body_name)
        new_body.set("pos", child.get("pos", "0 0 0"))
        new_body.set("quat", child.get("quat", "1 0 0 0"))
        
        # Copy all joints
        for joint in child.findall("joint"):
            joint_name = joint.get("name")
            if joint_name and joint_name in ALL_MODEL_JOINTS:  # Copy all model joints (controlled + passive)
                new_joint = ET.SubElement(new_body, "joint")
                for attr_name, attr_value in joint.attrib.items():
                    new_joint.set(attr_name, attr_value)
                
                # Apply position limits from POSITION_LIMITS
                if joint_name in POSITION_LIMITS:
                    limits = POSITION_LIMITS[joint_name]
                    # For right-side joints, flip the limits and signs (except ball joints and foot joints)
                    if 'right' in joint_name and joint_name not in BALL_JOINTS and joint_name not in FOOT_JOINTS:
                        min_val = -limits['max']  # Flip sign and use max as min
                        max_val = -limits['min']  # Flip sign and use min as max
                    else:
                        min_val = limits['min']
                        max_val = limits['max']
                    
                    new_joint.set("range", f"{min_val} {max_val}")
                    
                    # Set motor class only for controlled joints (not ball joints or foot joints)
                    if joint_name not in BALL_JOINTS and joint_name not in FOOT_JOINTS:
                        new_joint.set("class", "motor_03")
                    print(f"    Added joint: {joint_name} with limits: {min_val} to {max_val}")
                else:
                    # Set motor class only for controlled joints (not ball joints or foot joints)
                    if joint_name not in BALL_JOINTS and joint_name not in FOOT_JOINTS:
                        new_joint.set("class", "motor_03")
                    print(f"    Added joint: {joint_name}")
            elif joint_name:
                print(f"    Skipping uncontrolled joint: {joint_name}")
        
        # Skip linkage closing sites
                 
        # Copy inertial
        inertial = child.find("inertial")
        if inertial is not None:
            new_body.append(inertial)
        
        # Copy geoms but remove problematic collision geoms
        remove_collision_geoms = [
            'leg_lower_left-geom-1',
            'leg_lower_right-geom-1', 
            'left_foot-geom-1',
            'right_foot-geom-1',
            'body_collision',
            'body_lower_collision',
            'hip_abd_mount_collision',
            'leg_upper_upper_left_collision',
            'hip_abd_mount_mirror_collision',
            'leg_upper_upper_right_collision',
            'neck_collision',
            'right_arm_shoulder_mount_collision',
            'right_arm_top_collision',
            'right_arm_upper_collision',
            'forearm_right_collision',
            'left_arm_shoulder_mount_collision',
            'left_arm_top_collision',
            'left_arm_upper_collision',
            'forearm_left_collision',
            'leg_upper_left_collision',
            'leg_upper_right_collision',
            'leg_lower_left_collision',
            'leg_lower_right_collision',
            'left_foot_collision',
            'right_foot_collision',
            'body_collision_mesh',
            'body_mesh',
            'torso_collision',
            'torso_mesh'
        ]
        
        for idx, geom in enumerate(child.findall("geom")):
            # Map names for unnamed geoms
            geom_name = geom.get("name", "")
            if not geom_name:
                geom_name = body_name + '-geom-' + repr(idx)
            
            # Remove specific problematic collision geoms
            mesh_name = geom.get("mesh", "")
            if (geom_name in remove_collision_geoms or 
                mesh_name in remove_collision_geoms or
                "body_collision" in mesh_name or
                "body_collision" in geom_name):
                print(f"    Skipping problematic collision geom: {geom_name} (mesh: {mesh_name})")
                continue
                
            new_geom = ET.SubElement(new_body, "geom")
            for attr_name, attr_value in geom.attrib.items():
                new_geom.set(attr_name, attr_value)
        
        # Add foot box collision geoms and sites
        if body_name == 'left_foot':
            # Add box collision geom
            foot_geom = ET.SubElement(new_body, "geom")
            foot_geom.set("name", "l_foot")
            foot_geom.set("class", "collision")
            foot_geom.set("pos", "-0.001 -0.05 -0.04")
            foot_geom.set("quat", "1 0 0 0")
            foot_geom.set("type", "box")
            foot_geom.set("size", "0.06 0.014 0.12")
            foot_geom.set("friction", "1.1 0.005 0.0001") # Replicate rubber friction
            foot_geom.set("solref", "0.005 1")
            foot_geom.set("solimp", "0.92 0.96 0.001")
            foot_geom.set("condim", "3")
            
            # Add site for sensors
            foot_site = ET.SubElement(new_body, "site")
            foot_site.set("name", "l_foot")
            foot_site.set("pos", "-0.001 -0.05 -0.04")
            foot_site.set("quat", "1 0 0 0")
            print(f"    Added box collision geom and site for {body_name}")
            
        elif body_name == 'right_foot':
            # Add box collision geom
            foot_geom = ET.SubElement(new_body, "geom")
            foot_geom.set("name", "r_foot")
            foot_geom.set("class", "collision")
            foot_geom.set("pos", "-0.001 -0.05 -0.04")
            foot_geom.set("quat", "1 0 0 0")
            foot_geom.set("type", "box")
            foot_geom.set("size", "0.06 0.014 0.12")
            foot_geom.set("friction", "1.1 0.005 0.0001") # Replicate rubber friction
            foot_geom.set("solref", "0.005 1")
            foot_geom.set("solimp", "0.92 0.96 0.001")
            foot_geom.set("condim", "3")

            # Add site for sensors
            foot_site = ET.SubElement(new_body, "site")
            foot_site.set("name", "r_foot")
            foot_site.set("pos", "-0.001 -0.05 -0.04")
            foot_site.set("quat", "1 0 0 0")
            print(f"    Added box collision geom and site for {body_name}")
        
        # Recursively process children
        process_child_bodies(child, new_body)

if __name__ == '__main__':
    generate_modelone_sim()
