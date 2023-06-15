# Gazebo Skeleton Visual Model Plugin

## Overview
This repository contains a Gazebo model plugin that allows for the visualization of a skeleton (bone or armature) model in the Gazebo simulator, while the collision model utilizes primitive models such as spheres or cylinders instead of the skeleton. The plugin is provided as a ROS package.

This project is based on the "Actor" plugin from Gazebo. You can find the original plugin [here](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/Actor.hh).

## Prerequisites
In order to use this plugin, you will need:
- A collision model written in Xacro.
- A skeleton model in Collada format (optional textures attached to it are also supported).
- Extra settings to be made to the Xacro file as shown below.

```xml
<gazebo>
  <plugin name="model_visuals" filename="libskeleton_model_plugin.so">
    <skin>
      <filename>/path/to/skeleton_visual.dae</filename>
      <scale>1.0</scale>
      <bone name="Hand_L" link="left_hand_link" parent="left_arm_link"/>
      <bone name="Hand_R" link="right_hand_link" parent="right_arm_link"/>
      <bone name="Head" link="head_link" parent="body_link"/>
    </skin>
  </plugin>
</gazebo>
```

In this context, `name` refers to the name of the bone in the Collada model, `link` represents the link name in the Xacro model, and `parent` represents the parent link name in the Xacro model. This configuration correlates the physical simulation (collision) and visualization (skeleton).

## Installation
To install the Gazebo Skeleton Visual Model Plugin, clone the repository into your catkin workspace and build the package using catkin.

```bash
cd ~/catkin_ws/src
git clone https://github.com/groove-x/gazebo_skeleton_visual_model_plugin.git
cd ..
catkin_make
```

## Usage
After building the plugin, ensure that your model has the necessary settings in its Xacro file, and the correct collision and skeleton models. To apply the plugin, you will need to add the aforementioned XML block into your robot's Xacro file.

## Contributing
Contributions are welcome! Please fork the repository and create a pull request with your changes.

## License
This project is licensed under BSD License.
