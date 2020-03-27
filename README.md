# Dependencies

1. Gazebo 7.1

    You can use the `gazebo_markers` branch for visual markers.

1. ROS (Indigo, Jade, Kinetic), or ROS2

1. `gazebo-ros-pkgs` for Gazebo 7 (`sudo apt-get install ros-<distro>-gazebo7-ros-pkgs`)

1. Ignition Transport, Math, Msgs (`sudo apt-get install libignition-msgs-dev libignition-math-dev libignition-transport-dev`)

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

1. There are 4 possible starting locations
    1. Three positions along the center line
    1. One position on the opposing team's side (spy bot).

1. The goal is to earn points by achieving the following:

    1. Drive over a centerfield barricade
        1. First time: 5 pts
        1. Second time: 10 pts
        1. Drive over four barricades, two times each: 20pts
    1. Throw a ball into opponent’s tower
        1. Bottom tower: 5 pts
        1. Top tower: 15 pts

1. 6 balls are lined up at centerfield when match begins, three balls are
   located in each castle

1. Additional balls are added to the field from alternating sides at
   1-minute intervals, only if balls are in the castle

1. Teams may tele-operate their robots, or opt for autonomy

1. The team with the most points at the end of the 10-minute game wins

1. Cameras

    1. Each team may have a total of two cameras.
    1. A camera may be attached to a robot.
        1. The camera can be attached to a pan-tilt unit, but must remain
           fixed to the model.
    1. A camera may be placed behind their team's castle
        1. The camera's Z position must be between 0.5 and 2.0 meters above
           the ground plane.
        1. The camera's X position must be between -4.0 and 4.0 meteres in
           the world coordinate frame
        1. Red: Camera's Y position must be less than -8.3 meters in the world coordinate frame
        1. Blue: Camera's Y position must be greater than 8.3 meters in the world coordinate frame.
        1. A camera behind a team's castle can be floating and movable
    1. It is not possible to add or remove cameras during competition.
    1. It is not possible to swap castle and robot cameras.

## Procedure

The final competition will be run on a single computer in the cafe. This
repository will be used to run the competition by launching:

```
roslaunch frc2016_competition frc2016_compete.launch
```

Each team will be allowed one directory on the cafe machine in which to
upload any models, plugins, and code. The two directories will be
`~/red_team` and `~/blue_team`.

The `frc2016_compete.launch` script will run two launch files:

1. frc2016_competition/launch/red_spawn[1-6].launch
2. frc2016_competition/launch/blue_spawn[1-6].launch

Six trials will be run. The first trial will run `spawn1.launch`, the second
`spawn2.launch` and so on.

Each spawn launch script should spawn your team's robots into Gazebo. The
possible starting locations for each team are:

    1. Red
        1. x:-2 y:-1
        1. x:-1 y:-1
        1. x:0 y:-1
        1. x:1 y:-1
        1. x:2 y:-1
        1. x:-3 y:7
    1. Blue
        1. x:-2 y:1
        1. x:-1 y:1
        1. x:0 y:1
        1. x:1 y:1
        1. x:2 y:1
        1. x:3 y:-7

An Ignition message will be sent that signals the start of the competion.

    Topic: /frc2016/game_control
    Message Type: ignition::msgs:StringMsg
    Data: 'start'

At this point your robots may move and score points. You may use any number
of laptops or other devices. Only the game directory may physically interact
with the PC running the competition.

We will assume everyone is playing nicely, which means:

1. Don't try to sabatoge the other team.
1. Don't listen to topics like Gazebo's ground truth.

Competition will end when another Ignition message is sent:

    Topic: /frc2016/game_control
    Message Type: ignition::msgs:StringMsg
    Data: 'stop'

Score will be sent on another Ignition transport topic:

    Topic: /frc2016/score
    Message Type: ignition::msgs:Int32_V
    Data: [0] = Blue Score, [1] = Red score

## Winning

The field is not symmetric. The competition will be run six times. A team
will start on each side three times.  The team with the largest average
score will be the winner.
