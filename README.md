# CDRM

ROS packages for Contact Dynamic Roadmaps (CDRM). This repository contains the following packages:

* `cdrm` - an implementation of the CDRM data structure.
* `cdrm_legged` - legged planning using CDRMs.
* `cdrm_msgs` - messages for the `cdrm` package.
* `cdrm_welding` - a welding tool path planner using CDRMs.
* `cdrm_welding_msgs` - messages for the `cdrm_welding` package.
* `cdrm_welding_tutorial` - an example work cell which uses `cdrm_welding` to plan welds.
* `example_quadruped_description` - a basic quadruped URDF.
* `example_quadruped_leg_kinematics` - an IKFast generated solver for an individual leg.
* `example_quadruped_moveit_config` - a MoveIt configuration for the quadruped.

## Requirements

* ROS Melodic

## License

* MIT

## Author

* Andrew Short (University of Wollongong) <andrewjshort@gmail.com>

## Usage

### Legged Planning

Run the RViz demo with:

```
roslaunch example_quadruped_moveit_config demo.launch
```

### Weld Planning

To run the weld planning example, first generate the CDRM for the work cell:

```
$ roslaunch cdrm_welding_tutorial test_m10ia_on_gantry.launch
$ rosrun cdrm_welding cdrm_welding_node
$ rosrun cdrm_welding_tutorial generate_welding_cdrm.py
```

Then run the plan weld script and view it in RViz:

```
$ roslaunch cdrm_welding_tutorial test_m10ia_on_gantry.launch
$ rosrun cdrm_welding cdrm_welding_node
$ rosrun cdrm_welding_tutorial plan_weld.py
```
