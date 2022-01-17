# Marine Group
Delezenne Quentin - Duchiron Benjamin

This repository contains all the work of the group in the course LARM.

# In **master**
This master branch contains all the work, including the tutorials and the drafts.

# In **challenge2**
This branch contains the work done to achieve the second challenge.  
The behavior coded will observe the environement and try to identify both orange and black bottles using two different methods.
Orange bottles are detected using HSV filter and contours research. A stream of the scans are displayed in `detect_orange` with blue circles.
Orange bottles are detected using Harr learning detection. A stream of the scans are displayed in `detect_black` with blue rectangles.
Rviz will be launched automatically and will display :
- The pose of the tbot with a red arrow
- The gmapping Map
- The `detect_orange` stream in the bottom right
- The `detect_black` stream in the bottom right

The Harr detection is not 100% accurate and the choice have been made to make it more strict to avoid noise. It allows us to avoid many false results, but it also prevent us of detectingall the black bottles. A consequence is the lack of detection in case of bottle laying on the side.

Overview of a result :  
![Overview](https://github.com/Waramer/LARM_Repo/tree/challenge2/Overview.png)


To launch the code with the minimal code (orange bottle only), run :
`roslaunch grp-marine challenge2.launch`

To launch the code with all the nodes (both detections), run :
`roslaunch grp-color challenge12prime.launch`