# AWS Warehouse TB3

A ROS package that has only a launch file to run aws-robomaker-small-warehouse-world with Turtlebot 3

## Steps

1. Install Turtlebot 3 packages following this [official tutorial](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/). Click on building `TurtleBot3 package from source` to install them as ROS packages in catkin_ws, instead of installing them as system packages.

**NOTE:** If you install packages using Github, make sure to select the right branch based on ROS version

2. Clone [aws-robomaker-small-warehouse-world](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world) 
```bash
cd ~/catkin_ws/src
git clone -b ros1 https://github.com/aws-robotics/aws-robomaker-small-warehouse-world
cd ~/catkin_ws
catkin build # or catkin_make 
```

3. Clone this repo
```
cd ~/catkin_ws/src
git clone <repo-link>
cd ~/catkin_ws
catkin build # or catkin_make 
```

4. Run simulation

```bash
roslaunch aws_warehouse_tb3 aws_warehouse_tb3.launch 
```

5. Run TB3 navigation

```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=~/catkin_ws/src/aws-robomaker-small-warehouse-world/maps/005/map.yaml
```
