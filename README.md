# scanningPRM
Planning Project 16782

Gazebo Environment:
  
    roslaunch planning_prm scan_pro_robot_gazebo_control.launch
  
    rostopic pub /scan_pro_robot/arm_controller/command trajectory_msgs/JointTrajectory '{joint_names: ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"], points: [{positions: [0.5,0.5,0.02,0.5,0.8,1.0], time_from_start: [1,0]}]}' -1
