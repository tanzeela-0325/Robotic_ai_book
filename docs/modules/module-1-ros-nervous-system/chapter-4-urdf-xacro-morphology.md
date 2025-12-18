# Chapter 4: URDF/XACRO and Humanoid Morphology

## Introduction

The Universal Robot Description Format (URDF) and Extensible Hierarchical ARchitecture (XACRO) are fundamental to defining the physical structure and kinematics of humanoid robots in ROS 2. These formats provide a standardized way to describe robot geometry, joints, links, and inertial properties that are essential for simulation, control, and visualization.

## Understanding URDF

### What is URDF?

URDF is an XML-based format for describing robot models in ROS. It defines:
- **Links**: Physical parts of the robot (e.g., torso, arms, legs)
- **Joints**: Connections between links (e.g., rotational, prismatic)
- **Inertial Properties**: Mass, center of mass, inertia tensors
- **Visual and Collision Geometry**: Mesh representations for visualization and collision detection

### Basic URDF Structure

A minimal URDF model looks like:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Link definitions -->
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint definitions -->
  <joint name="base_joint" type="fixed">
    <parent link="base_link"/>
    <child link="upper_body"/>
    <origin xyz="0 0 0.2"/>
  </joint>

  <!-- Additional links and joints -->
  <!-- ... -->
</robot>
```

## XACRO for Complex Models

### Why Use XACRO?

XACRO extends URDF with:
- **Macros**: Reusable code blocks for repetitive structures
- **Parameters**: Dynamic values for customization
- **Mathematical Expressions**: Calculations within the model
- **Conditional Statements**: Different configurations based on parameters

### XACRO Basics

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

  <!-- Parameters -->
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>

  <!-- Macro for wheel -->
  <xacro:macro name="wheel" params="prefix x_loc y_loc z_loc">
    <link name="${prefix}_wheel">
      <inertial>
        <mass value="1.0"/>
        <origin xyz="0 0 0"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
      </inertial>
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="chassis"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${x_loc} ${y_loc} ${z_loc}" rpy="1.5708 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:wheel prefix="front_left" x_loc="0.2" y_loc="0.3" z_loc="0.0"/>
  <xacro:wheel prefix="front_right" x_loc="0.2" y_loc="-0.3" z_loc="0.0"/>
  <!-- ... more wheels -->

</robot>
```

## Humanoid-Specific Considerations

### Joint Types for Humanoids

Humanoid robots require specific joint configurations:

#### Revolute Joints
Used for articulated limbs:
```xml
<joint name="shoulder_roll" type="revolute">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <origin xyz="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
</joint>
```

#### Continuous Joints
For rotating components:
```xml
<joint name="waist_yaw" type="continuous">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.2"/>
  <axis xyz="0 0 1"/>
  <limit effort="100" velocity="2.0"/>
</joint>
```

### Kinematic Chain Structure

Humanoid robots typically follow these kinematic chains:

#### Upper Body
```
base_link
└── torso
    ├── neck
    │   ├── head
    │   └── left_arm
    │       ├── upper_arm
    │       ├── lower_arm
    │       └── hand
    └── right_arm
        ├── upper_arm
        ├── lower_arm
        └── hand
```

#### Lower Body
```
base_link
└── pelvis
    ├── left_hip
    │   ├── left_thigh
    │   ├── left_calf
    │   └── left_foot
    └── right_hip
        ├── right_thigh
        ├── right_calf
        └── right_foot
```

## Humanoid-Specific URDF Features

### Inertial Properties

Accurate inertial properties are crucial for simulation and control:

```xml
<link name="left_upper_leg">
  <inertial>
    <mass value="3.2"/>
    <origin xyz="0 0 -0.15"/>
    <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.04" iyz="0" izz="0.03"/>
  </inertial>
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
  </collision>
</link>
```

### Joint Limits

Proper joint limits prevent mechanical damage:

```xml
<joint name="left_knee" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_calf"/>
  <origin xyz="0 0 -0.3"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="0.0" effort="100" velocity="2.0"/>
</joint>
```

### Visual and Collision Geometry

Separate geometries for visualization and collision detection:

```xml
<link name="hand">
  <visual>
    <geometry>
      <mesh filename="package://robot_description/meshes/hand.stl"/>
    </geometry>
    <material name="white"/>
  </visual>
  <collision>
    <geometry>
      <sphere radius="0.05"/>
    </geometry>
  </collision>
</link>
```

## Advanced XACRO Techniques

### Conditional Definitions

```xml
<xacro:if value="$(arg enable_head)">
  <link name="head">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
</xacro:if>
```

### Mathematical Expressions

```xml
<xacro:property name="leg_length" value="0.4"/>
<xacro:property name="hip_offset" value="${leg_length * 0.5}"/>
```

### Parameterized Macros

```xml
<xacro:macro name="limb" params="side length">
  <link name="${side}_upper_arm">
    <inertial>
      <mass value="${length * 0.5}"/>
      <origin xyz="0 0 ${-length/2}"/>
      <inertia ixx="${0.01 * length}" ixy="0" ixz="0" iyy="${0.01 * length}" iyz="0" izz="${0.005 * length}"/>
    </inertial>
  </link>
</xacro:macro>
```

## Integration with ROS 2 Systems

### TF Tree Structure

URDF defines the transform tree that ROS 2 uses for coordinate transformations:

```xml
<joint name="base_to_torso" type="fixed">
  <parent link="base_link"/>
  <child link="torso"/>
  <origin xyz="0 0 0.5"/>
</joint>
```

This creates a TF tree where:
- `base_link` is the root
- `torso` is a child of `base_link`
- All other links are descendants of `torso`

### Visualization in RViz

Proper URDF enables rich visualization:

```xml
<visual>
  <geometry>
    <mesh filename="package://robot_description/meshes/torso.dae"/>
  </geometry>
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
</visual>
```

### Control Integration

URDF links and joints are used by controllers:

```xml
<ros2_control name="HumanoidRobotSystem" type="system">
  <hardware>
    <plugin>fake_components/GenericSystem</plugin>
  </hardware>
  <joint name="left_shoulder_pitch">
    <command_interface name="position"/>
    <state_interface name="position"/>
  </joint>
</ros2_control>
```

## Best Practices for Humanoid URDF

### Modularity

Break large models into manageable components:

1. **Base Structure**: Main chassis and mounting points
2. **Upper Body**: Arms, head, torso
3. **Lower Body**: Legs and hips
4. **Appendages**: Additional sensors or tools

### Parameterization

Make models adaptable to different configurations:

```xml
<!-- Parameters for different humanoid sizes -->
<xacro:property name="height" value="1.7"/>
<xacro:property name="arm_length" value="${height * 0.4}"/>
<xacro:property name="leg_length" value="${height * 0.45}"/>
```

### Documentation

Include clear comments for maintainability:

```xml
<!--
  Humanoid Robot URDF Model
  Version: 1.0
  Author: Robotics Team
  Description: Complete humanoid model for ROS 2 simulation
  Joint limits based on anthropomorphic constraints
-->
```

## Testing and Validation

### URDF Validation

Use ROS 2 tools to validate models:

```bash
# Validate URDF syntax
check_urdf robot.urdf

# Visualize in RViz
ros2 run rviz2 rviz2 -d robot.rviz

# Check TF tree
ros2 run tf2_tools view_frames
```

### Performance Considerations

- **Collision Geometry**: Simplified shapes for better performance
- **Visual Geometry**: Detailed meshes only for visualization
- **Joint Count**: Balance complexity with computational requirements
- **Link Count**: Minimize unnecessary intermediate links

## Conclusion

URDF and XACRO provide the foundation for humanoid robot modeling in ROS 2. Properly defined robot models enable accurate simulation, reliable control, and effective visualization. Understanding these formats and implementing best practices ensures that humanoid robots can be developed, tested, and deployed efficiently in both simulation and real-world environments.

This chapter has covered the essential concepts and techniques for creating URDF/XACRO models for humanoid robots, setting the stage for more advanced topics in subsequent chapters.