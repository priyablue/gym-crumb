# gym-crumb



# Installation

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
