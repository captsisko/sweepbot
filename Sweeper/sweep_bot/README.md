# Sweep-Bot

Model of street sweeper sanitation robot.

## Launching application

**drive.launch** is the main launch file for the application.
It includes all the primary nodes required by the applicaion with the exeception of the laser merging node, more on that later.

The argument to the gazebo empty world which includes the small-city test environment can be commented out for a lighter test environment with the benefit that it loads quicker than the city.

Execute `roslaunch sweep_bot drive.launch`

**merge_laser_scans.launch** is used to merge the laserscans into a single laserscan topic giving a unified 360 degrees coverage with 25 meters range of the robot. Somewhat strangely, any scan outside of the configured range is marked as *1 meter + the configured range of the lasers* i:e 26 meters. This is instead of the *inf* value encountered in default scan topics.

Execute `roslaunch sweep_bot merge_laser_scans.launch`

**wall_follower.py** is the work-in-progress script used to implement logic to make the robot locate, move towards and follow a wall. In the full simulation of the small-city the wall would actually be a kerb.

Execute `sweep_bot wall_follower.py`

