# deoxys_ros
ExecuteTrajectoryAction server/client for executing MoveIt trajectories with deoxys_control

## Setup
1 - The Franka Panda must be on and connected (blue/green light)  
2 - The NUC should be running Deoxys  
3 - FCI should be enabled
4 - Roscore running  
5 - ```roslaunch launch/custom_move_group.launch```  

## Confirm Setup
Executing ```rostopic list``` should give you a sizeable number of topics  
Executing ```rostopic echo /joint_states``` should give nothing  

## Running the scripts
With the rosrpl environment activated run scripts.deoxys_server.py, this will create an action server waiting for requests. The deoxys_client is an example of how to query the moveit_commander and send the trajectory execution request to the server.

## Todo
[] Add environment.yaml for rosrpl  
[] Add a guide for enabling uniform sampling on the motion planner  
