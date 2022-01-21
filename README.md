# Marine Group
Delezenne Quentin - Duchiron Benjamin

This repository contains all the work of the group in the course LARM.

# In **challenge3**
This branch contains the work done to achieve the third challenge.  

First, this part contains all the work done in the challenge2 : the HSV detection and the Haar detection.

In the challenge 3, we worked to improve how the robot is moving in its environment. The system is based on a multiplexer that will receive move ordrers from a navigation system, and a collision avoidance system (or CAS). Il will then send the order to the robot by prioritizing the orders of the CAS.

The CAS uses the Hokuyo laser to examine the environment and determine the range and the position of the dangerous obstacles. It then send move order to avoid collision, taking in account the range of the closest obstacle and the occupancy on both its left and right sides.

The navigation system has to parts, but only the first has been succesfully implemented.  
The first one uses headings to goals and the orientation of the robot to make it turn until the goal is in front of him. It will then move forward until it reaches it.  
The second part uses A* Algorithm. It uses the map provided by Gmapping, with a little treatment to fill gaps and remove inregularities. It would allow the robot to determine the best path to reach his goal in the format of several waypoints, that he could follow using the first part of the navigation system.

The file `navplan.py` is the file in which the A* is coded, and has an executable code to check its behaviour. To run it, uncomment the last lines and make sure that a "map.png" file has been put in the directory. It will run and export the path and nodes into a `path.png` file and display the map with the path using openCV.

## How to use

To launch the code of all the project on a turtlebot, run :
`roslaunch grp-marine challenge3_tbot.launch`

To launch the code in the simualtion, without the vision part, run :
`roslaunch grp-marine challenge3_simulation.launch`