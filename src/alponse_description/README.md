# Alponse Description

ROS 2 description package generated from the original `alponse.SLDASM` SolidWorks URDF export.

The package contains a cleaned ROS 2 URDF at `urdf/alponse.urdf`, STL meshes with package-safe names, an RViz2 launch file, and a MuJoCo-loadable URDF copy at `mujoco/alponse_mujoco.urdf`.

## RViz2

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select alponse_description
source install/setup.bash
ros2 launch alponse_description display.launch.py
```

## MuJoCo

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select alponse_description
source install/setup.bash
ros2 run alponse_description view_alponse_mujoco
```
