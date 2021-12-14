# MY_TELEOP Package

This package is designed to send messages to move the turlebot in the stage simulation.

It is composed of :

`my_keyboard_teleop.launch` : a launch file to have live commands on the turtle bot.

`turtlebot_teleop_key.py` : a code file to receive keyboard keys and use them to move the turtle (used with my_keyboard_teleop launch file)

`read_my.py` : a code file to create a node that send messages on the topic to make move the tutle. The moves will repeat until the node is killed.