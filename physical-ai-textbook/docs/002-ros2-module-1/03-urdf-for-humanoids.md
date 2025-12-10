# URDF for Humanoids: Describing Robot Structure

In robotics, precisely describing the physical structure of a robot is fundamental for simulation, motion planning, and control. URDF (Unified Robot Description Format) is an XML format used in ROS 2 (and ROS 1) to represent the kinematic and dynamic properties of a robot. This chapter will introduce you to the core components of URDF, focusing on how it can be used to model humanoid robot structures, including links, joints, and sensors.

## What is URDF?

URDF is a standard way to describe a robot's physical characteristics, such as:
*   **Kinematics**: The robot's links (rigid bodies) and joints (connections between links).
*   **Dynamics**: Mass, inertia, and friction properties of links and joints.
*   **Visuals**: How the robot looks in a simulator (e.g., mesh files, colors).
*   **Collisions**: The collision geometry of the robot for physics simulation.
*   **Sensors**: The location and type of sensors attached to the robot.

A URDF file typically represents a single robot. For more complex scenarios or robots that change configuration (e.g., grippers), Xacro (XML Macros) is often used to generate URDF files from more modular definitions.

## Links: The Robot's Rigid Bodies

A **link** in URDF represents a rigid body of the robot. This could be a segment of a limb, a torso, a head, or any other fixed component. Each link has physical properties associated with it:

*   **`inertial`**: Describes the mass, center of mass, and inertia matrix of the link. This is crucial for accurate physics simulation.
    ```xml
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <inertia ixx="0.083" ixy="0" ixz="0" iyy="0.083" iyz="0" izz="0.083"/>
    </inertial>
    ```
*   **`visual`**: Defines how the link appears in a simulator. This can include geometry (e.g., box, cylinder, mesh file), color, and origin (offset from the link's origin).
    ```xml
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 1.0"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    ```
*   **`collision`**: Defines the geometry used for collision detection in a physics simulator. This is often a simplified version of the visual geometry to reduce computational load.
    ```xml
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 1.0"/>
      </geometry>
    </collision>
    ```

## Joints: Connecting the Links

**Joints** define the connection between two links (a `parent` link and a `child` link) and specify their relative motion. URDF supports various types of joints:

*   **`revolute`**: A hinge joint that rotates around a single axis. It has upper and lower limits.
*   **`continuous`**: Similar to `revolute`, but without limits.
*   **`prismatic`**: A sliding joint that translates along a single axis. It has upper and lower limits.
*   **`fixed`**: A rigid connection between two links, allowing no relative motion. Useful for connecting parts that don't move relative to each other (e.g., a camera mounted on a robot's head).
*   **`planar`**: Allows motion in a plane.
*   **`floating`**: Allows all 6 degrees of freedom. Typically used for the base link of a mobile robot.

Each joint specifies:
*   **`parent`** and **`child`** links.
*   **`origin`**: The transform from the parent link's origin to the joint's origin.
*   **`axis`**: The axis of rotation or translation for revolute, continuous, and prismatic joints.
*   **`limit`**: Defines the upper and lower limits (for `revolute` and `prismatic` joints), velocity limits, and effort limits.

```xml
<joint name="base_link_to_arm_joint" type="revolute">
  <parent link="base_link"/>
  <child link="shoulder_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit effort="100" velocity="1" lower="-1.57" upper="1.57"/>
</joint>
```

## Sensors: Integrating Perception

While URDF itself doesn't define the behavior of sensors, it's used to specify their physical attachment and pose relative to a robot's links. You can represent sensors as `fixed` joints connecting a sensor link to a robot body link. The sensor link might then have visual or collision properties that represent the sensor's physical casing.

For example, attaching a camera to a robot's head:

```xml
<joint name="head_to_camera_joint" type="fixed">
  <parent link="head_link"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>

<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.02 0.02"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
</link>
```

The actual data from the camera (images, point clouds) would be published by a separate ROS 2 node, but its physical mounting is described in the URDF.

## Building a Simple Humanoid Structure (Conceptual)

Let's consider a very basic humanoid robot. It might consist of:
*   **`base_link`**: The core of the robot (e.g., torso).
*   **`head_link`**: Connected to `base_link` by a `revolute` or `fixed` neck joint.
*   **`shoulder_link`**: Connected to `base_link` by a `revolute` shoulder joint.
*   **`upper_arm_link`**: Connected to `shoulder_link` by a `revolute` elbow joint.
*   **`forearm_link`**: Connected to `upper_arm_link`.
*   **`hand_link`**: Connected to `forearm_link`.

Each of these links would have its own inertial, visual, and collision properties. The joints would define how these parts move relative to each other, creating the robot's kinematic chain.

## Conclusion

URDF is an indispensable tool for robotics engineers, allowing them to define the intricate physical details of their robots in a standardized, machine-readable format. Understanding links, joints, and how to represent sensors within this framework is key to working with ROS 2 and robotic simulations. In the next section, we'll create a simple URDF file and learn how to inspect its structure.

## Inspecting the Example URDF File

After creating a URDF file, it's essential to inspect and validate its structure. ROS 2 provides tools to help with this.

1.  **Source your ROS 2 environment**:
    If you haven't already, source your ROS 2 installation:
    ```bash
    source /opt/ros/humble/setup.bash
    ```
    (Replace with your actual ROS 2 path.)

2.  **Validate the URDF file**:
    Use the `check_urdf` tool to parse and validate your URDF for syntax errors.
    ```bash
    check_urdf examples/simple_humanoid.urdf
    ```
    If there are no errors, it will print information about the robot's links and joints.

3.  **Visualize the kinematic tree with `urdf_to_graphiz`**:
    This tool generates a graphical representation of your robot's kinematic tree, which is very helpful for understanding the link-joint hierarchy.
    First, you might need to install it:
    ```bash
    sudo apt install liburdfdom-tools
    ```
    Then, generate the graph:
    ```bash
    urdf_to_graphiz examples/simple_humanoid.urdf
    ```
    This will create a `simple_humanoid.pdf` or `simple_humanoid.gv` file in your current directory, which you can open to see the robot's structure.

4.  **Visualize the robot model in `rviz2`**:
    `rviz2` is a powerful 3D visualization tool for ROS 2. You can load your URDF model into `rviz2` to see its visual representation.

    First, ensure you have the `urdf_tutorial` package installed (it contains a `state_publisher` that helps `rviz2` display the URDF):
    ```bash
    sudo apt install ros-humble-urdf-tutorial
    ```

    Then, launch `rviz2` and load the URDF:
    ```bash
    ros2 launch urdf_tutorial display.launch.py model:=$(ros2 pkg prefix --share urdf_tutorial)/urdf/01-myfirst.urdf
    ```
    (Note: The command above launches a generic example. To load your `simple_humanoid.urdf`, you'll typically need to set up a small launch file or use a `robot_state_publisher` with your URDF. For simplicity, you can also directly open `rviz2` and add a "RobotModel" display, then manually set the `Robot Description` parameter to the content of your URDF file, or point it to a `robot_description` parameter published by a `robot_state_publisher`.)

    A simpler way to view *just your URDF* without a full launch file:
    ```bash
    # (Requires robot_state_publisher and joint_state_publisher_gui to be installed)
    # Open a terminal and run:
    # ros2 launch urdf_tutorial display.launch.py model:=./examples/simple_humanoid.urdf
    # This will open rviz2 and a joint state publisher GUI.
    ```
    If the above `ros2 launch` command does not directly support loading a local file path as easily, you can manually set it up in `rviz2`:
    *   Start `rviz2` (`rviz2`).
    *   In the "Displays" panel, click "Add".
    *   Find "RobotModel" and add it.
    *   In the "RobotModel" properties, under "Robot Description", change the value to the full content of your `simple_humanoid.urdf` file (copy-paste, or set up a parameter server).
    *   You may also need to add "JointStatePublisher" and "TF" displays, and set the "Fixed Frame" to `base_link` for proper visualization.

## Further Reading

*   [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
*   [ROS 2 URDF Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/index.html)

---

*(Placeholder for example code: A simple humanoid URDF will be added here in a later task.)*
