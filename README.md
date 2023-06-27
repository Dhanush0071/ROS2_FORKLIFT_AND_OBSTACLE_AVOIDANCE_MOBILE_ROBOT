# Ros2_forklift_obstacle_avoiding-robot_gazebo


 -->  Clone this github repo 'vision' to your ros2 workspace
 -->  Install all the required softwares
 Ubuntu 22.04 ROS2 Humble

$1 our first project is 
-A mobile robot which projects 10 samples of lidar lasers out and rotates until an obstacle is detected and moves towards the detected object and when the robot reaches close enough to the object a vacuum force is excreted on the object which pulls the object near the forks  and we can also see the object with the 360 degrees camera and if an obstacle is precious enough we can click on the key “C” on the keyboard performs the fork lift on the object


$2 one is 
-A mobile robot which projects 10 samples of lidar lasers out and moves linearly broadcasting a 360 degree camera stream by the cameras mounted to all three sides of the body of the robot and if any obstacle comes in the path of the robot it stops and rotates and searches for a free space and if found it again starts moving linearly and the process is fully automatic 
And as per users wish the robot can be manually driven using the “A,W,S,D” keys on your keyboard

demo are included in rep


#OBJ.PY  - OBSTACLE AVOIDING ROBOT FILE
#OBJ1.PY - DETECT AND FORK LIFT ROBOT FILE
