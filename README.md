# Install

1. Create a catkin workspace

```
source /opt/ros/jade/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace 
```

1. Download this repository

```
cd ~/catkin_ws/src
hg clone https://bitbucket.org/osrf/frc2016_competition
```

1. Compile

```
cd ~/catkin_ws
catkin_make install
```

1. Setup

```
source install/setup.sh
source install/share/frc2016_competition/setup.sh
```

1. Run

```
roslaunch frc2016_competition frc2016.world
```
