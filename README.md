# Software Development Project
## Direct Base Controller (DBC)

### Direct Base Controller: Merged Nodes

1. Clone the repo in your `catkin_workspace/src`.
2.  Disable following packages
	* mir_direct_base_controller
	* mcr_direct_base_controller
	* mcr_common_converters
	* mcr_manipulation_measurers

	Use following command to disable a package
	```
	roscd package_name
	touch CATKIN_IGNORE
	```
3.  Delete old build files before new build. Go to the *ros*  directory and execute.
	```
	catkin build
	source ~/.bashrc
	```
4.  Use *group3_direct_base_controller* package to launch.
	```
	roslaunch group3_direct_base_controller direct_base_controller.launch
	roslaunch group3_direct_base_controller pose_mock_up_gui.launch
	roslaunch group3_direct_base_controller event_gui.launch
	```

##### To Do: Merge Nodes

+ ~~mcr_direct_base_controller:direct_base_controller_coordinator_node~~

+ mcr_geometric_relation_monitors:component_wise_pose_error_monitor_node
+ mcr_twist_controller:twist_controller_node
+ mcr_twist_limiter:twist_limiter_node
+ mcr_twist_synchronizer:twist_synchronizer_node

##### Nodes communication
To check how nodes communicate with each other, run the simulation and run `rqt_graph`.
The GUI will show you a graph of how nodes talk to each other.
