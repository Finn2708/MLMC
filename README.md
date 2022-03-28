# MLMC

## Setup
### Without custom rosserial messages
```
git clone # this repo
cd catkin_ws
catkin_make clean
catkin_make install
source devel/setup.bash

```

### For building custom messages
```
cd your/arduino/libraries
rosrun rosserial_arduino make_libraries . mlmc_msgs
```

## Usage
To launch `rosserial_python serial_node.py`, `mlmc test_run_server.py` and `mlmc brain.py` all at once, run:

```
roscore
roslaunch mlmc mlmc.launch
```

Training will start immediately with params defined in `brain.py`.
