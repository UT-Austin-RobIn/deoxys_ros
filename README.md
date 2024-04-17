# deoxys_ros
ExecuteTrajectoryAction server/client for executing MoveIt trajectories with deoxys_control

## Installation
1. Make a ros workspace, cd src
2. ```git clone git@github.com:UT-Austin-RobIn/deoxys_ros.git```
3. ```git clone git@github.com:UT-Austin-RobIn/panda_moveit_config.git```
4. ...
5. create a new conda environement (rosrpl) from environment.yaml and activate it  
6. in the deoxys_ros folder run ```catkin_make```, and ```source devel/setup.bash```           
7. Follow the instructions at https://github.com/UT-Austin-RPL/deoxys_control to install Deoxys  
8. After deoxys is installed ```cd ~/deoxys_control/deoxys``` and ```pip install -e .``` to make deoxys available to python (make sure you are in the rosrpl environment while doing this)  

## Setup
1. The Franka Panda must be on and connected (blue/green light)  
2. The NUC should be running Deoxys  
3. FCI should be enabled
4. ```roslaunch launch/deoxys_ros.launch```  

## Confirm Setup
Executing ```rostopic list``` should give you a sizeable number of topics  
Executing ```rostopic echo /joint_states``` should give nothing  

## Planning in your own scripts
With the rosrpl environment activated and dexos_ros.launch launded there should be an action server for you to interface with.   
The interactive_client.py file demonstrates an example of how to use the provided commander.py file as a python interface.

## Todo
[X] Add environment.yaml for rosrpl  
[X] Add a guide for enabling uniform sampling on the motion planner  
[X] Remove the walls from the planning scene  
[ ] Have someone test setting it up from scratch  
[ ] Cleanup commander.py and deoxys_server.py
