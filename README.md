# gym-crumb



# Installation of CRUMB

1. Create workspapce (for example catkin_ws);

2. Gitclone into catkin_ws/src https://github.com/CRUMBproject/ROS; https://github.com/vanadiumlabs/arbotix_ros.git

3. Add to CmakeList:
```bash
add_compile_options(-std=c++11)
add_compile_options(-std=gnu++11)
```

4. In file roboticsgroup_gazebo_plugins/mimic_joint_plugin.cpp replace SetMaxForce with SetParam("max_force", 0,max_effort_) and SetAngle with SetParam("angle", 0, math::Angle(angle));

5. Intall: ros-kinetic-position-controllers, ros-kinetic-effort-controllers, ros-kinetic-joint-state-controller;

6. Build package with catkin_make (I had got message told me to use "find . -iname "*.xacro" | xargs sed -i 's#<\([/]\?\)\(if\|unless\|include\|arg\|property\|macro\|insert_block\)#<\1xacro:\2#g' /home/airan/kinetic/src/Crumb/crumb_description/urdf/crumb.xacro" and after using this command CRUMB had built)

7. There are some worlds in crumb_gazebo that uses camera. You should replace path in this file.world with path, in which you wnat to save the images.
	

# Installation of gym-crumb

```bash
cd gym-crumb
pip install -e .
```

# Example

Start playgroun1.world
```bash
roslaunch crumb_gazebo crumb_pick_place.launch world_file:=/.../playground1.world
```
reset() - reset environment to initial state

step(arm_index, angle) - environment publishes angle on a topic
```bash
import gym
import gym_crumb
from math import radians
env = gym.make("crumb-v0")  
ACTION_SPACE = {
0: radians(90),
1: radians(45),
2: radians(0),
3: radians(-45),
4: radians(-90),
}
env.step([0,ACTION_SPACE[3]])
```


# Agents

Start gazebo and use notebook.

