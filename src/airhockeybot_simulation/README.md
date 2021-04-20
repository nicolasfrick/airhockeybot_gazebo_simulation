# Airhockeybot Simulation

## Usage:

### Install:

```bash
$ cd airhockeybot/src
$ git clone git@gitlab.com:projectir/airhockeybot.git
$ cd ..
$ catkin build
```

```bash
$ source /opt/ros/melodic/setup.bash
$ source devel/setup.bash
```

### Run:

```bash
$ roslaunch airhockeybot_gazebo airhockeybot_gazebo.launch
$ rosrun image_view image_view image:=/camera/color/image_raw
```

### Control:
'8', '2', '6', '4', # player bat/ puk orthogonal (up, down, right, left)
'9', '7', '1', '3', # player bat/ puk diagonal (up left, up right, down right, down left)
'w', 'x', 'd', 'a', # robot (up, down, right, left)
'r', 'p', '0'       # random bot, init puk, switch player/puk control

### Topics:

/airhockeybot/joint_states
/airhockeybot/joint_x_position/command
/airhockeybot/joint_y_position/command
/airhockeybot/player_joint_x_position/command
/airhockeybot/player_joint_y_position/command

/camera/color/camera_info
/camera/color/image_raw
/camera/color/image_raw/compressed
/camera/color/image_raw/compressed/parameter_descriptions
/camera/color/image_raw/compressed/parameter_updates