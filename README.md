# Marine Group
Delezenne Quentin - Duchiron Benjamin

This repository contains all the work of the group in the course LARM.

# In **master**
Ths master branch contains all the work, inculding the tutorials and the drafts.

# In **challenge1**
This branch contain the work done to achieve the first challenge, and the needed code to make is run (the mb6-tbot package is included).  
The behaviour coded will observe the obstcles and decide to stup and turn for a prolonged period of time to take another direction. It will decide the direction of the turn by choosing the way where it sees the less obstacles.  
The whole program is based on a single node. It is subscribed to the data from the laser csan, and publishes to the commands of the robot.

To launch the simulation in Gazebo with the given situation, use the command :  
`roslaunch grp-color challenge1_simulation.launch`

To launch the turtlebot, after connecting both the bot and the laser, use the command :  
`roslaunch grp-color challenge1_turtlebot.launch`

In both cases, the launch file will run all the code required and will display the Rviz rendering of the laser scan.