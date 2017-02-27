# Pioneer Hallway

Making a pioneer robot move down a hallway -- fast!

# Nodes

## Obstacle Tracker

## Controller

## Executive

   `controller_msg` - publisher type `String` - the actions from the planner
   
   `exec_planner.py` - Forks the planner as a subprocess, sends it the current state and publishes the action from the planner, listens to `obst_tracker` to update the state for the planner

   `exec_control_pub.py` - Sends a "forward" message to the controller as String

   `exec_obst_listen.py` - Listens to `obst_tracker` for String

   `exec_test_listener.py` - Simple test by publishing a dummy node

# Usage

Make sure you have your own catkin workspace and use this repo as your src/ folder.

You can work inside your own folder in src/ make sure to make it executable.

To build the package/nodes:
```bash
cd ~/your_ws
catkin_make
```

Testing/Running

```bash
roscore
cd ~/your_ws
source devel/setup.bash
rosrun pioneer_hallway node.py
```