# deoxys_ros
ExecuteTrajectoryAction server/client for executing MoveIt trajectories with deoxys_control

## Installation
1. ```git clone git@github.com:UT-Austin-RobIn/deoxys_ros.git```  
2. create a new conda environement (rosrpl) from environment.yaml and activate it  
3. in the deoxys_ros folder run ```catkin_make```    
4. Follow the instructions at https://github.com/UT-Austin-RPL/deoxys_control to install Deoxys  
5. After deoxys is installed ```cd ~/deoxys_control/deoxys``` and ```pip install -e .``` to make deoxys available to python (make sure you are in the rosrpl environment while doing this)  
6. Set up uniform sampling by going to ```$(rospack find panda_moveit_config)/launch/chomp_planning_pipeline.launch.xml``` and adding ```default_planner_request_adapters/AddTimeParameterization``` to the top of the planning adapters list  
7. Note, it may be necessary to remove something, as of yet it is unclear.

## Setup
1. The Franka Panda must be on and connected (blue/green light)  
2. The NUC should be running Deoxys  
3. FCI should be enabled
4. ```roslaunch launch/custom_move_group.launch```  

## Confirm Setup
Executing ```rostopic list``` should give you a sizeable number of topics  
Executing ```rostopic echo /joint_states``` should give nothing  

## Running the scripts
With the rosrpl environment activated run scripts/deoxys_server.py, this will create an action server waiting for requests. The deoxys_client is an example of how to query the moveit_commander and send the trajectory execution request to the server.  

After running deoxys_server.py executing ```rostopic echo /joint_states``` should start displaying the real time joint states of the Franka Panda.  

## Todo
[X] Add environment.yaml for rosrpl  
[X] Add a guide for enabling uniform sampling on the motion planner  
[X] Remove the walls from the planning scene  
[ ] Have someone test setting it up from scratch  
[ ] Cleanup commander.py and deoxys_server.py
