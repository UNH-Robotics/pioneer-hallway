# nav_bundle
Folder containing roslaunch files calling any combination of the following:
- slam_gmapping/amcl
- move_base
- simple_navigation_goals
  - waypoint server/client nodes
- nodes reshaping platform commanded velocities

The launch files in this folder are experiment specific, please do not overwrite any existing files but rather add your own launch files for your particular experiment.

Naming convention is to prefix all launch files to be called from within a group environment with 'single'.

A basic launch file is provided in nav_bundle.launch:

NOTE: MAKE SURE TO CALL THE NAVIGATION PACKAGE RELEVANT TO YOUR ROBOT ON LINE 9 OR LINE 12.

For pioneer gazebo simulation or hardware use pioneer_2dnav.

For erratic stage robot simulation use stagebot_2dnav
