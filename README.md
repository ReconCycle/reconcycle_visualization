# reconcycle_visualization/reconcycle_description
Holds URDF descriptions and CAD models (under meshes) of the ReconCycle. Also includes launch files folder.

Package covers the visualization of ReconCycle cell in RVIZ.

# Install

* If ROS workspace is not yet created, please refer to `http://wiki.ros.org/catkin/Tutorials/create_a_workspace` for more inforation on how to create a ROS workspace.

* cd to your ROS workspace:
```
$ cd /workspace_folder_name/src
```

* Clone the repository to your ROS workspace (example: /ros_ws/src folder):

`$ git clone https://github.com/ReconCycle/reconcycle_visualization.git`

* Catkin build:
* move to the /workspace_folder_name and build packages in your workspace (example: /ros_ws folder). 
```
$ cd ..
$ catkin build
```

# Usage

```
$ roslaunch reconcycle_visualization display_sim.launch
```

# Tricks and tips

