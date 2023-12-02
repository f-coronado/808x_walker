# Overview
This ros 2 project contains a simple obstacle avoidance algorithm that guides the turtlebot3 burger 

## Build and run steps
Clone the project into your ros2_ws/src/
```sh
cd ~/ros2_ws/src/
https://github.com/f-coronado/walker_808x.git
```

Navigate into your ros2 workspace
```sh
cd ~/ros2_ws
```

Build the package
```sh
colcon build --packages-select walker_808x
```
Source your setup.bash
```sh
source /path/to/your/ros2_ws/install/setup.bash
```


## Recording and playing back bag file
Use launch_roomba.py launch file to run nodes and record automatically
```sh
ros2 launch walker_808x launch_roomba.py launch_roomba.py:=true 
```
You can disable bag recording by using the following
```sh
ros2 launch walker_808x launch_roomba.py launch_roomba.py:=false 
```
To play the bag file, navigate to bag recording folder and play the bag file
```sh
cd ~/your_ros2_workspace/rosbagX_202X_XX_XX-XX_XX_XX
ros2 bag play bag_recordings_0.db3
```
You can see more information about the bag recording using the following
```sh
ros2 bag info rosbag2_202X_XX_XX-XX_XX_XX_X.db3
```


## Assumptions and Dependencies
This project was built on ros2 humble, see below to install
https://docs.ros.org/en/humble/Installation.html
The steps below should be completed in your ros2_ws

This package uses rclcpp, sensors_msgs and geometry_msgs. It is good practice to check for missing dependencies before building
```sh
rosdep install -i --from-path src --rosdistro humble -y 
```
If some dependencies are missing, update your workspace
```sh
sudo apt update
sudo apt full-upgrade
```

Make sure the required lines below are in your bashrc script
```sh
gedit ~/.bashrc
```
Add the following to the bottom of your bashrc
```sh
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```