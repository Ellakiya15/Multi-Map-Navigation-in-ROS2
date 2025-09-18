# Multi Map Navigation in ROS2
## WormholeNavigator 🌀
A ROS 2-based multi-map navigation system that enables seamless transitions between disconnected environments using "wormholes" - predefined entry and exit points that facilitate map switching.

## 📋 Overview
WormholeNavigator allows robots to navigate between multiple pre-mapped environments by defining transition points (wormholes) stored in a SQLite database. The system integrates with Nav2 for autonomous navigation and provides seamless map switching capabilities.

## Key Features

* 🗺️ Multi-map navigation with seamless transitions
* 🎯 Action server for automated map switching
* 🗄️ Database-driven wormhole management
* 🤖 Full Nav2 integration
* 📍 Automatic pose initialization on new maps

## 🚀 Quick Start
### Prerequisites
* ROS 2 Humble 
* Ubuntu 22.04

1. Setup Workspace
```
mkdir -p ~/wormhole_ws/src
cd ~/wormhole_ws/src
```
2. Clone and Install dependencies
```
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```
```
git clone https://github.com/Ellakiya15/Multi-Map-Navigation-in-ROS2.git
```
```
sudo apt install libsqlite3-dev
```
3. Build workspace
```
colcon build
```
4. Source the workspace
```
source install/setup.bash
```
## 🎮 Running the Demo
### Terminal Setup
Open 5 separate terminals and source the workspace in each:
```
cd ~/wormhole_ws
source install/setup.bash
```
### Step-by-Step Execution

Terminal 1: Launch Gazebo Simulation
```
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

Terminal 2: Start Multi-Map Navigation Stack
```
ros2 launch multi_map_nav nav.launch.py
```

Terminal 3: Launch RViz Visualization
```
ros2 launch nav2_bringup rviz_launch.py
```

Terminal 4: Start WormholeNavigator Node
```
ros2 run wormhole_navigator wormhole_navigator_node
```
Node should initialize and connect to database

Terminal 5: Execute Demonstration Commands

1️⃣ Test Basic Navigation
```
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.63, y: 2.82, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
```

2️⃣ Execute Wormhole Transition
```
ros2 action send_goal /switch_map wormhole_interfaces/action/SwitchMap \
"{from_map: 'map1.yaml', to_map: 'map2.yaml'}"
```

3️⃣ Validate Post-Transition Navigation
```
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: 'map'}, pose: {position: {x: -1.63, y: -0.60, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
```
## Expected Results

- Step 1: Robot navigates to position (1.63, 2.82) on the initial map
- Step 2: Map switches from map1.yaml to map2.yaml, robot appears at wormhole entry point
- Step 3: Robot navigates to position (-1.63, -0.60) on the new map
