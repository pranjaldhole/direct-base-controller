### Direct Base Controller: Merged Nodes

##### Activating DBC from repo
In order to deactive `direct_base_controller` from `mas_industrial_robotics`,
add `CATKIN_IGNORE` as empty file in the `mas_industrial_robotics/mit_navigation/direct_base_controller/`
and rebuild the catkin workspace with `catkin build`

##### To Do: Merge Nodes

+ ~~mcr_direct_base_controller:direct_base_controller_coordinator_node~~
+ mcr_common_converters:transform_to_pose_converter_node
+ mcr_manipulation_measurers:component_wise_pose_error_calculator_node
+ mcr_geometric_relation_monitors:component_wise_pose_error_monitor_node
+ mcr_twist_controller:twist_controller_node
+ mcr_twist_limiter:twist_limiter_node
+ mcr_twist_synchronizer:twist_synchronizer_node

##### Nodes communication
To check how nodes communicate with each other, run the simulation and run `rqt_graph`.
The GUI will show you a graph of how nodes talk to each other.

<!--- End of script --->