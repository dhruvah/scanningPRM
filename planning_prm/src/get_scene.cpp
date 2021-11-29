/*=================================================================
 *
 * get_scene.cpp
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <time.h>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <stack>
#include <iostream>
#include <sstream>

#include <ros/ros.h>

#include <moveit_msgs/PositionIKRequest.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/PlanningSceneComponents.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/GetStateValidityRequest.h>
#include <moveit_msgs/GetStateValidityResponse.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <tf/tf.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <std_msgs/String.h>


#define PI 3.141592654

using namespace std;

class PlanningScene:
{
    
}