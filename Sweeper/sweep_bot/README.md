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

## Illustrations

In the lighter simulation environment described above, an obstacle is required for the robot to detect. The following shows the robot deployed in such an environment with walls manually placed on the right to experiment with.

![Screenshot 2021-08-10 00:01:02](https://user-images.githubusercontent.com/3543536/129274102-6451b841-cfa1-4b9f-adee-7902732b9d76.png)

It also shows the full laserscan coverage of the robot.

![Screenshot 2021-08-12 22:51:36](https://user-images.githubusercontent.com/3543536/129274918-47f205c2-9a76-4e89-afd6-c3e7e0a9351c.png)
Configured left-side laser coverage

![Screenshot 2021-08-12 22:56:16](https://user-images.githubusercontent.com/3543536/129275209-25809207-e66a-40f2-905c-712b668a343d.png)
Configured right-side laser coverage

<!-- Test Scripts
As previously mentioned, the **laser_ranges_class.py** script, though poorly named, has evolved into the primary test script with it's #code tested and copied into wall_follower.py.
Below is a recording of using this test script to orient the robot to a desired angle. -->

<!-- https://user-images.githubusercontent.com/3543536/129284802-5163f635-9c84-4db9-bc42-5d20af02a0e7.mp4 -->

<!-- Arguments in **target=** mode are 0, 90, -90 and 180. -->

# Dependencies installation
sisko@rosbox:~/catkin_ws$ clear && rosdep install --from-paths src/
