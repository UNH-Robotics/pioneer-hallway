# Pioneer Hallway

Making a pioneer robot move down a hallway -- fast!

# Nodes

Obstacle Tracker

Executive

Controller

# Usage

Make sure you have your own catkin workspace and use this repo as your src/ folder.

You can work inside your own folder in src/ make sure to place your node into scripts/ as well.

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