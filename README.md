# Dependencies

1. Gazebo 7.1

    You can use the `gazebo_markers` branch for visual markers.

1. ROS (Indigo, Jade, Kinetic), or ROS2

1. Ignition Transport, Math, Msgs

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
roslaunch frc2016_competition frc2016.launch
```

# The Competition

## Rules

1. Each team may build up to 3 robots

    3 different robots, or use 3 identical robots

1. The goal is to earn points by achieving the following:

    1. Drive over a centerfield barricade
        1. First time: 3 pts
        1. Second + time: 5 pts
    1. Throw a ball into opponent’s tower
        1. Bottom tower: 10 pts
        1. Top tower: 20 pts

1. 6 balls are lined up at centerfield when match begins, three balls are
   located in each castle

1. Additional balls are added to the field from alternating sides at
   1-minute intervals, only if balls are in the castle

1. Teams may teleoperate their robots, or opt for autonomy

1. The team with the most points at the end of the 10-minute game wins

## Procedure

The final competition will be run on a single computer in the cafe. This
repository will be used to run the competition by launching:

```
roslaunch frc2016_competition frc2016_compete.launch
```

Each team will be allowed one directory on the cafe machine in which to
upload any models, plugins, and code. The two directories will be
`~/red_team` and `~/blue_team`. A `setup.sh` in each directory will be run
prior to launch. Use this script to add paths to environment variables so
that Gazebo and ROS can find your models, plugins, and packages.


The `frc2016_compete.launch` script will run two launch files:

1. red_team spawn.launch
2. blue_team spawn.launch.

Each `spawn.launch` script should spawn your team's robots into Gazebo.


A ROS message will be sent that signals the start of the competion.

    Topic: /frc2016/game_control
    Message Type: std_msgs:String
    Data: 'start'

At this point your robots may move and score points. You may use any number
of laptops or other devices. Only the game directory may physically interact
with the PC running the competition.

We will assume everyone is playing nicely, which means:

1. Don't try to sabatoge the other team.
1. Don't listen to topics like Gazebo's ground truth.

Competition will end when another ROS message is sent:

    Topic: /frc2016/game_control
    Message Type: std_msgs:String
    Data: 'stop'

Score will be sent on another ROS topic:

    Topic: /frc2016/score
    Message Type: std_msgs:UInt32MultiArray
    Data: [0] = Red Score, [1] = Blue score 

## Winning

The field is not symmetric. The competition will be run six times. A team
will start on each side three times.  The team with the largest average
score will be the winner.