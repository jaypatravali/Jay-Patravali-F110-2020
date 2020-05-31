## Instructions

1. Check the params.yaml to change the wall following mode, currently its default DIRECTION: 'center'
2. To set KP, KD again check params.yaml
3. To change default velocity check params.yaml.
4. Demo videos are provided in Videos/
5. Make sure you have the f110-skeletons-spring2020 race simulator also compiled in the catkin_ws
6. [Update]: in report I mention changing pid values for Center following. Its now improved and no need of changing KP, KD values.


##Run after compiling workspace

``  $roslaunch patravali_wall_following wall_following_sim.launch
    press n