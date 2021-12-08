# Planning Project 16782 - Fall '21
## Motion Planning for Precise Scanning of Additively Manufactured Parts

Instructions to run the code:

Clone the repo into the \src folder of your catkin_ws
Go to the folder of your workspace where you can see \src folder and run:
```
  catkin build
```
Open 2 different terminal windows and source them
```
  source devel/setup.bash
```
In 1st terminal run:
```
  roslaunch scan_pro_moveit_config demo_gazebo.launch
```
In the second terminal run: (build a roadmap)
```
  rosrun planning_prm planning_prm_node _param=buildprm _file=waypt1.txt
```
In the second terminal run: (run A* search on an existing roadmap)
```
  rosrun planning_prm planning_prm_node _param=loadprm _file=waypt1.txt
```
