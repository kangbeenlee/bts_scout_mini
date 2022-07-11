# BTS PROJECT for Scout mini Simulation in Gazebo
## 1.	Introduction of Function Package

```
├── scout_base
├── scout_bringup
├── scout_msgs
└── ugv_gazebo_sim
```

*	scout_base：The folder is scout mini odometry package

*	scout_bringup: The folder is a gmapping_slam and navigation package

* scout_msgs: The folder is a package of scout messages

* ugv_gazebo_sim: The folder is gazebo simulation package



## 2.	Environment

### 2-1. Development Environment

​ ubuntu 18.04 + [ROS Melodic desktop full](http://wiki.ros.org/melodic/Installation/Ubuntu)

### 2-2. Download and install required function package

​ Download and install ros-control function package, ros-control is the robot control middleware provided by ROS

```
sudo apt-get install libasio-dev ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-gazebo-ros ros-melodic-gazebo-ros-control ros-melodic-joint-state-publisher-gui ros-melodic-teleop-twist-keyboard ros-melodic-navigation ros-melodic-gmapping
```



## 3.	About Usage

### 3-1.	Create workspace, download simulation model function package and compile

​ Open a new terminal and create a workspace named scout_ws, enter in the terminal:

```
mkdir -p ~/scout_ws/src
```

​ Enter the scout_ws/src folder

```
cd ~/scout_ws/src
```

​ Download simulation model function package

```
git clone https://github.com/kangbeenlee/bts_scout_mini.git
```

​ Enter the scout_ws folder

```
cd scout_ws
```

​ Confirm whether the dependency of the function package is installed

```
rosdep install --from-paths src --ignore-src -r -y 
```

​ Compile

```
catkin_make
```



### 3-2.	Start the gazebo simulation environment of scout_mini and control scout_v2 and scout_mini movement in the gazebo

​ Enter the scout_ws folder

```
cd scout_ws
```

​ Declare the environment variable

```
source devel/setup.bash
```

​ Start the simulation environment of scout_mini

```
roslaunch scout_gazebo_sim scout_mini_playpen.launch
```

​ Control by keyboard, scout_mini can be controlled to move forward, left, right and backward

```
roslaunch scout_teleop scout_teleop_key.launch
```

​ or

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```



### 4.	Start the gazebo simulation environment of scout_mini and gmapping in the gazebo
​ Start the simulation environment of scout_mini

```
roslaunch scout_slam scout_slam.launch
```

​ You can save the map by

```
roslaunch scout_slam gmapping_save.launch
```

### 5. Start the gazebo simulation environment of scout_mini and navigation in the gazebo
​ Start the simulation environment of scout_mini

```
roslaunch scout_navigation scout_navigation.launch
```
