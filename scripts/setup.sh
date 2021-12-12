# Cloning TB3 packages
if [ ! -d "${HOME}/catkin_ws/src/DynamixelSDK" ]; then
    echo "Didn't find DynamixelSDK. Cloning it..."
    cd ${HOME}/catkin_ws/src/
    git clone -b melodic-devel https://github.com/AnasMK/DynamixelSDK.git
fi

if [ ! -d "${HOME}/catkin_ws/src/turtlebot3_msgs" ]; then
    echo "Didn't find turtlebot3_msgs. Cloning it..."
    cd ${HOME}/catkin_ws/src/
    git clone -b melodic-devel https://github.com/AnasMK/turtlebot3_msgs.git
fi

if [ ! -d "${HOME}/catkin_ws/src/turtlebot3" ]; then
    echo "Didn't find turtlebot3. Cloning it..."
    cd ${HOME}/catkin_ws/src/
    git clone -b melodic-devel https://github.com/AnasMK/turtlebot3.git
fi

echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc

if [ ! -d "${HOME}/catkin_ws/src/aws-robomaker-small-warehouse-world" ]; then
    echo "Didn't find aws-robomaker-small-warehouse-world. Cloning it..."
    cd ${HOME}/catkin_ws/src/
    git clone -b ros1 https://github.com/aws-robotics/aws-robomaker-small-warehouse-world
fi

. ${HOME}/.bashrc
. /opt/ros/melodic/setup.bash && cd ${HOME}/catkin_ws && catkin_make

