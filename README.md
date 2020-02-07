# Human robot collaboration
Here is brief documentation about the gesture detection project work and how to set up the enviroment

## Gesture Detection software

https://github.com/athena15/project_kojak

Repository above was used in order to implement the gesture detection software. Clone the above repository and copy-paste talker.py from under the gesture_detection folder of this repository. The script (talker.py) is built on Python 2.7 and you need to pip install the requirements for the script. Some of the requirements are listed in the project kojak repository but there is some excess requirements aswell (smart home stuff). In addition you must have atleast have http://wiki.ros.org/rospy installed in order to publish signals in the topics.

## Franka Emika Panda controller software

https://github.com/frankaemika/franka_ros

Above repository was used in creating the software for the Panda controller. In https://github.com/betrri/roboprojectwork repository please copy the files under panda_controller to following directories in franka_ros repository in order to make it work. 

franka_example_controllers.yaml ->  franka_ros/franka_example_controllers/config/

gesture_movement.cpp  -> franka_ros/franka_example_controllers/src/

gesture_movement.h  -> franka_ros/franka_example_controllers/include/ 

gesture_movement.launch  -> franka_ros/franka_example_controllers/launch/ 

## Launching the code 
 
First the network should be tested and the flow of gestures from detection unit to controller confirmed and be sure that it is configured as shown in Figure 2. 
Robot controller is launched by running the custom roslaunch file gesture_movement.launch by typing in terminal: 
roslaunch franka_example_controllers gesture_movement.launch. 
This launch finds the correct ros parameters from the franka_example_controllers.yaml, starts the gesture_movement.cpp controller and starts the rviz visualization.  
Gesture detection software is started by running the talker.py script. 
