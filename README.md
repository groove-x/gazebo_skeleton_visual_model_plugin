# Gazebo Skeleton Visual Model Plugin

## Overview
This repository contains a Gazebo visual plugin that allows for the visualization of a skeleton model in the Gazebo simulator, while the collision model utilizes primitive models such as spheres or cylinders instead of the skeleton. The plugin is provided as a ROS package.

This project is based on the "Actor" plugin from Gazebo. You can find the original plugin [here](https://github.com/arpg/Gazebo/blob/master/gazebo/physics/Actor.hh).

## Installation
To install the Gazebo Skeleton Visual Model Plugin, clone the repository into your catkin workspace and build the package using catkin.

```bash
cd ~/catkin_ws/src
git clone https://github.com/groove-x/gazebo_skeleton_visual_model_plugin.git
cd ..
catkin_make
```

## Usage
To apply the plugin, you will need:
- A model for the physical simulation (links and collisions) in Xacro format.
- A skeleton model for visualization in Collada format (optional textures attached to it are also supported).
- Extra settings to be made to the Xacro file as shown below.

## Example

See details in the example files:
- `xacro/fish.xacro`
- `meshes/fish.dae`
- `config/example.yaml`
- `launch/example.launch`

### Joints, Links and Collisions in Xacro Format

Joints, links and collision models are defined in Xacro format.

`origin` in `joint` element is the original offset from the parent link to the child link when the joint is at zero position.

This is obtained directly from design tools like blender,
or can be shown by setting `<printTransforms>true</printTransforms>` element to plugin's `<skin>` element.

```xml
<joint name="ventral_body_joint" type="revolute">
  <parent link="head_link"/>  <!-- NOTE: parent link of `body` bone -->
  <child link="ventral_body_link" />  <!-- NOTE: child link of `body` bone -->
  <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
  <axis xyz="0 0 1"/>
  <limit effort="20" velocity="5" lower="-0.5" upper="0.5"/>
  <dynamics damping="0.1"/>
</joint>

<link name="ventral_body_link">
  <!-- inertial and collision description here -->
</link>

<transmission name="ventral_body_trans">
  <!-- transmission description which is optional -->
</transmission>
```

### Skeleton Model in Collada Format

The skeleton model defined in Collada format is exported from design tools with bones attached to the mesh.

### Plugin Setting in Xacro Format

Plugin settings are used to map xacro joints to collada bones.


```xml
<gazebo>
  <plugin name="model_visuals" filename="libskeleton_model_plugin.so">
    <skin>
      <filename>/path/to/skeleton_visual.dae</filename>
      <scale>1.0</scale>
      <!-- <printTransforms>true</printTransforms> -->
      <bone name="body" link="ventral_body_link" parent="head_link"/>
      <bone name="fin" link="caudal_fin_link" parent="ventral_body_link"/>
    </skin>
  </plugin>
</gazebo>
```

In this context, `name` refers to the name of the bone in the Collada model, `link` represents the link name in the Xacro model, and `parent` represents the parent link name in the Xacro model. This configuration correlates the physical simulation (collision) and visualization (skeleton).

When the `printTransforms` element is set to `true`, the plugin prints the transform of each bone in the skeleton model to the terminal.

If you need the original offset from the parent link to the child link when the joint is at zero position, you need to remove all `bone` elements.

## Support
The Gazebo Skeleton Visual Model Plugin is tested and supported on Gazebo 11 with ROS noetic.

## Contributing
Contributions are welcome! Please fork the repository and create a pull request with your changes.

## License
This project is licensed under BSD License.
