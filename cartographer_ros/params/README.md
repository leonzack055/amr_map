# ROS2 Params Files for byd_amr_lds

This directory contains ROS2 parameter files corresponding to the `byd_amr_lds.launch.py` launch file.

## Files Overview

### 1. [`cartographer_node_params.yaml`](cartographer_node_params.yaml)
Parameters for the `cartographer_node`:
- `use_sim_time`: Enable simulation time (default: true)
- `configuration_directory`: Path to Cartographer Lua configuration files
- `configuration_basename`: Lua configuration file name (`online_byd_landmark.lua`)
- `save_state_filename`: Path where the map state will be saved (`map.pbstream`)
- `load_state_filename`: Path to a previously saved state file (default: empty)
- `load_frozen_state`: Whether to load the state as frozen (default: false)
- `log_level`: Logging level (info)
- `odom`: Topic remapping for odometry (`/odom_combined`)

### 2. [`cartographer_occupancy_grid_node_params.yaml`](cartographer_occupancy_grid_node_params.yaml)
Parameters for the `cartographer_occupancy_grid_node`:
- `use_sim_time`: Enable simulation time (default: true)
- `resolution`: Map resolution in meters per pixel (default: 0.05)

### 3. [`robot_state_publisher_params.yaml`](robot_state_publisher_params.yaml)
Parameters for the `robot_state_publisher`:
- `use_sim_time`: Enable simulation time (default: true)
- `robot_description`: Robot URDF content (loaded at runtime)

### 4. [`rviz2_params.yaml`](rviz2_params.yaml)
Parameters for `rviz2`:
- `use_sim_time`: Enable simulation time (default: true)
- `rviz_config`: Path to RViz configuration file (`demo_docker2d.rviz`)

## Usage Examples

### Using ros2 launch with params files

```bash
# Launch cartographer_node with params file
ros2 launch cartographer_ros cartographer_node.launch.py \
  --ros-args \
  --params-file $(ros2 pkg prefix cartographer_ros)/share/cartographer_ros/params/cartographer_node_params.yaml

# Launch cartographer_occupancy_grid_node with params file
ros2 run cartographer_ros cartographer_occupancy_grid_node \
  --ros-args \
  --params-file $(ros2 pkg prefix cartographer_ros)/share/cartographer_ros/params/cartographer_occupancy_grid_node_params.yaml

# Launch robot_state_publisher with URDF and params
xacro $(ros2 pkg prefix cartographer_ros)/share/cartographer_ros/urdf/byd_amr.urdf | \
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  --params-file $(ros2 pkg prefix cartographer_ros)/share/cartographer_ros/params/robot_state_publisher_params.yaml

# Launch rviz2 with params file
rviz2 \
  -d $(ros2 pkg prefix cartographer_ros)/share/cartographer_ros/configuration_files/demo_docker2d.rviz \
  --ros-args \
  --params-file $(ros2 pkg prefix cartographer_ros)/share/cartographer_ros/params/rviz2_params.yaml
```

### Using in a custom launch file

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('cartographer_ros')
    
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        parameters=[f'{pkg_share}/params/cartographer_node_params.yaml'],
        remappings=[('odom', '/odom_combined')],
        output='screen'
    )
    
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        parameters=[f'{pkg_share}/params/cartographer_occupancy_grid_node_params.yaml'],
        output='screen'
    )
    
    return LaunchDescription([
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
```

## Parameter Overrides

You can override parameters from the params files using command-line arguments:

```bash
ros2 run cartographer_ros cartographer_node \
  --ros-args \
  --params-file $(ros2 pkg prefix cartographer_ros)/share/cartographer_ros/params/cartographer_node_params.yaml \
  -p load_state_filename:=/path/to/saved_map.pbstream \
  -p load_frozen_state:=true
```

## Notes

- The URDF file (`byd_amr.urdf`) is located in the `urdf/` directory of the cartographer_ros package
- The Lua configuration file (`online_byd_landmark.lua`) is in the `configuration_files/` directory
- The RViz configuration file (`demo_docker2d.rviz`) is also in the `configuration_files/` directory
- Topic remappings can be done either in the params file or using `--remap` flags