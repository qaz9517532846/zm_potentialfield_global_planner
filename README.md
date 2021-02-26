# zm_potentialfield_global_planner

- zm_potentialfield_global_planner is a global planner algorithm under ROS using Potential Field for AGV using mecanum wheel motion.

- start pose to goal pose x-y 2-D motion(x-y), finally rotation z angle to goal pose(x-y-theta).

- Software : Robot Operating System.

- OS: Linux Ubuntu.

- Step1. Download zm_potentialfield_global_planner github link.

``` bash
$ cd <catkin_ws>/src
```

``` bash
$ git clone https://github.com/qaz9517532846/zm_potentialfield_global_planner.git
```

``` bash
$ cd ..
```

``` bash
$ catkin_make
```

- Step2. zm_potentialfield_global_planner add to move_base.launch file.

``` bash
  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <param name="base_global_planner" value="zm_potentialfield_global_planner/zmPotentialFieldGlobalPlanner" />
    <param name="base_local_planner" value="dwa_local_planner/DWAPlannerROS" />
    <rosparam file="$(find zm_robot_navigation)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find zm_robot_navigation)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find zm_robot_navigation)/param/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find zm_robot_navigation)/param/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find zm_robot_navigation)/param/move_base_params.yaml" command="load" />
    <rosparam file="$(find zm_robot_navigation)/param/dwa_local_planner_params.yaml" command="load" />
    <remap from="cmd_vel" to="$(arg cmd_vel_topic)"/>
    <remap from="odom" to="$(arg odom_topic)"/>
    <param name="DWAPlannerROS/min_vel_x" value="0.0" if="$(arg move_forward_only)" />
  </node>
```

- Potential Field global planner and A* global planner path result.

![image](https://github.com/qaz9517532846/zm_potentialfield_global_planner/blob/main/image/result.png)

## Reference

[1]. Potential_Field_GlobalPlanner_ROS. https://github.com/gonzalesMK/Potential_Field_GlobalPlanner_ROS

------

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2020 ZM Robotics Software Laboratory.
