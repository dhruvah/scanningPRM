/*=================================================================
 *
 * planner.cpp
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
#include <ros/console.h>

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

#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#define PI 3.141592654

using namespace std;

// need to figure out exactly how this is going to work with ROS- compiling executable and running seperate times wont work,
// will need to save data elsewhere in that case
struct Node;
unordered_map<int, Node*> vertices;
unordered_map<int, vector<int>> components;
static bool mapGenerated;
static int numVertices = 0;
static int maxNeighbors = 10;
static double epsilon = PI/4; //2 <<<<<<<<< parameter tuning
static double i_step = (PI/4)/2;
string PLANNING_GROUP = "arm";
vector<double*> valid_states;

// void init_scene(const moveit::core::RobotModelPtr& kinematic_model)
// {
// 	planning_scene::PlanningScene planning_scene(kinematic_model);

// 	collision_detection::CollisionRequest collision_request;
//   	collision_detection::CollisionResult collision_result;

// 	moveit::core::RobotState copied_state = planning_scene.getCurrentState();
// 	// moveit::core::RobotStatePtr copied_state(new moveit::core::RobotState(kinematic_model));
// 	collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
// 	copied_state.setJointGroupPositions(PLANNING_GROUP, q_check);

// 	planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
// 	return;
// }

// in tatti memory of k.....
void is_valid_K(planning_scene::PlanningScene* planning_scene, vector<double> q_check)
{
	moveit::core::RobotState copied_state = planning_scene->getCurrentState();
	// ********need to change so that array is converted to vector - do const& (for setJointGroupPositions)
	copied_state.setJointGroupPositions(PLANNING_GROUP, q_check);

	planning_scene->isStateColliding(copied_state);
	// cout << "Testing is valid of planning scene: " << planning_scene->isStateColliding(copied_state) << endl;
	bool k = false;
	return;
}

double euclidDist(double *v1, double *v2, int numOfDOFs)
{
	double result = 0;
	for (int i = 0; i < numOfDOFs; i++)
	{
		result += (v1[i] - v2[i])*(v1[i] - v2[i]);
	}
	return sqrt(result);
}

struct Node {
	int id;
	int compId;
	double *angles;
	vector<int> neighbors;
	double f;
	double g;
	Node *parent;

	Node(int idIn, int compIdIn, double *anglesIn)
	{
		this->id = idIn;
		this->compId = compIdIn;
		this->angles = anglesIn;
		this->g = INT16_MAX;
	}

	void addEdge(Node *n)
	{
		this->neighbors.push_back(n->id);
		n->neighbors.push_back(this->id);
	}

	bool setAstarParams(Node *newParent, double *goalAngles, int numOfDOFs)
	{
		double new_g = newParent->g + euclidDist(newParent->angles, this->angles, numOfDOFs);
		if (new_g < this->g)
		{
			this->parent = newParent;
			this->g = newParent->g + euclidDist(newParent->angles, this->angles, numOfDOFs);
			this->f = this->g + euclidDist(this->angles, goalAngles, numOfDOFs);
			return true;
		}
		return false;
	}

	void resetAstarParams()
	{
		this->f = 0;
		this->g = INT16_MAX;
		this->parent = 0;
	}

};

struct NodeCompare
{
    bool operator()(Node *n1, Node *n2)
    {
        return n1->f > n2->f;
    }
};


double randomAngleGen(int i) {
	double randAngle;
	static bool init = false;
	if (!init)
	{
		srand(time(NULL));
		init = true;
	}
	if (i == 0)
	{
		randAngle = -2.96706 + ((double)rand() / RAND_MAX)*(2.96706 - (-2.96706)); 
	}
	else if (i == 1)
	{
		randAngle = -1.745329 + ((double)rand() / RAND_MAX)*(2.356194 - (-1.745329)); 
	}
	else if (i == 2)
	{
		randAngle = -2.076942 + ((double)rand() / RAND_MAX)*(2.949606 - (-2.076942)); 
	}
	else if (i == 3)
	{
		randAngle = 0.0; 
	}
	else if (i == 4)
	{
		randAngle = 0.0;
	}
	else
	{
		randAngle = 0.0; 
	}
	// randAngle = ((double)rand() / RAND_MAX) * 2 * PI; //generates random double between 0 and 2*PI
	return (randAngle);
}

void randomSample(double *angles, int numOfDOFs, double *map, int xSize, int ySize) {
	for (int i = 0; i < numOfDOFs; i++)
	{
		angles[i] = randomAngleGen(i);
	}
	if (true) // need to change this to collision check, use fcl library
	{
		return;
	}
	return randomSample(angles, numOfDOFs, map, xSize, ySize);
}

void printAngles(double *angles, int size) {
    string temp = "[";
    for (int i = 0; i < size; i++) {
        temp += to_string(angles[i]) + ", ";
    }
    temp = temp.substr(0,temp.length() - 2) + "]\n";
    cout << temp;
}

void update_component_ids(unordered_map<int, Node*> &vertices, unordered_map<int, vector<int>> &components, int oldId, int newId)
{
	for (int id : components[oldId])
	{
		vertices[id]->compId = newId;
		components[newId].push_back(id);
	}
	components[oldId].clear();
}

Node* integrate_with_graph(double *angles, unordered_map<int, Node*> &vertices, unordered_map<int, vector<int>> &components, int maxNeighbors, int idx, double epsilon, int numOfDOFs, double *map, int xSize, int ySize)
{
	Node *q = new Node(idx, idx, angles);
	vertices[idx] = q;
	components[idx].push_back(q->id); 
	int newId;
	double dist;

	for (auto &it : vertices)
	{
		if (it.second == q)
		{
			continue;
		}
		dist = euclidDist(angles, it.second->angles, numOfDOFs);
		if (dist < epsilon)
		{
			if (q->neighbors.size() <= maxNeighbors)  // could also be (q->comp_id != it.second->comp_id) if only connecting components once (this was a problem before though)
			{
				if (true)	// need to implement obstacleFree function here (interpolate and check for collisions between configs)
				{
					newId = it.second->compId;
					q->addEdge(it.second);
					if (q->compId != it.second->compId)
					{
						update_component_ids(vertices, components, q->compId, newId);
					}
				}
			}
		}
	}
	return q;
}

void removeFromGraph(Node *&n, unordered_map<int, Node*> &vertices, unordered_map<int, vector<int>> &components)
{
	for (int id : n->neighbors)
	{
		vertices[id]->neighbors.pop_back();
	}
	vertices.erase(n->id);
	components[n->compId].pop_back();
	delete n;
}

void deletePointers(unordered_map<int, Node*> &vertices) {
	Node *q;
	double *a;
	for (auto& kv: vertices) {
		q = kv.second;
		a = q->angles;
		delete q, a;
	}
}

void printNumComponents(unordered_map<int, vector<int>> &components) {
	int numComponents = 0;
	for (auto& it : components)
	{
		if (!it.second.empty())
		{
			numComponents++;
		}
	}
	cout << "num of components: " << numComponents << endl;
}

void clearOldData(unordered_map<int, Node*> &vertices, unordered_map<int, vector<int>> &components)
{
	vertices.clear();
	components.clear();
}

void resetNodeAstarParams(unordered_map<int, Node*> &vertices)
{
	for (auto &it : vertices)
	{
		it.second->resetAstarParams();
	}
}

void printPlan(double **&plan, int planLength, int numOfDOFs) {
	if (plan == 0){
		return;
	}

	for (int i = 0; i < planLength; i++) {
		printAngles(plan[i], numOfDOFs);
	}
	cout << "Plan Length: " << planLength << endl;
}

void backtrackPRM(Node *goal, int numOfDOFs, double **&plan, int &planLength)
{
	cout << "Plan euclidean distance: " << goal->g << endl;
	Node *temp = goal;
	int numSteps;
	stack<double*> path;
	while (temp->parent != 0)
	{
		numSteps = (int)(euclidDist(temp->angles, temp->parent->angles, numOfDOFs) / i_step);
		numSteps = (numSteps > 0) ? numSteps : 1;
		for (int step = 0; step < numSteps; step++)
		{
			double *newAngles = new double[numOfDOFs];
			for (int i = 0; i < numOfDOFs; i++)
			{
				newAngles[i] = temp->angles[i] + (temp->parent->angles[i] - temp->angles[i]) * ((double)step / (double)numSteps);
			}
			path.push(newAngles);
		}
		temp = temp->parent;
	}
	path.push(temp->angles); // start angles
	planLength = path.size();
	plan = (double **)malloc(planLength * sizeof(double *));
	for (int j = 0; j < planLength; j++)
	{
		plan[j] = (double *)malloc(numOfDOFs * sizeof(double));
		plan[j] = path.top();
		path.pop();
	}
}

void aStarSearch(unordered_map<int, Node*> &vertices, Node *start, Node *goal, int numOfDOFs, double **&plan, int &planLength)
{
	cout << "finding optimal path...\n";
	priority_queue<Node *, vector<Node *>, NodeCompare> OPEN;
    unordered_set<int> CLOSED;
	OPEN.push(start);
	start->g = 0;
	start->f = 0;
	start->parent = 0;
	int index;
	bool goalFound = false;
	double distBetween;
	Node *best;
	Node *child;
	while (!OPEN.empty())
	{
		best = OPEN.top();
		OPEN.pop();
		index = best->id; 
		if (CLOSED.count(index))
		{
			continue;
		}
		CLOSED.insert(index);
		// check if goal
		if (best == goal)
		{
			goalFound = true;
			break;
		}
		for (int id : best->neighbors)
		{
			if (!CLOSED.count(id))
			{
				child = vertices[id];
				distBetween = euclidDist(best->angles, child->angles, numOfDOFs);
				if (child->setAstarParams(best, goal->angles, numOfDOFs))
				{
					OPEN.push(child);
				}
			}
		}
	}
	if (goalFound)
	{
		cout << "Goal found. Now Backtracking\n";
		backtrackPRM(best, numOfDOFs, plan, planLength);
		cout << "Path found" << endl;
	}
	else
	{
		cout << "No possible path\n";
	}
}

void plannerPRM(int numOfDOFs, double *startAngles, double *goalAngles, double **&plan, int &planLength) {
	if (!mapGenerated) {
		// pre-processing
		mapGenerated = true;
		deletePointers(vertices);
		clearOldData(vertices, components);

		Node *q;
		int tf = 5;
		// int K = 20000;
		int K = valid_states.size();
		clock_t prmStartTime = clock();
		// while (((double)(clock() - prmStartTime) / CLOCKS_PER_SEC) < tf)
		for (int k = 0; k < K; k++)
		{
			double *s = new double[numOfDOFs];
			s = valid_states[k];
			// randomSample(s, numOfDOFs, 0, 0, 0);   // need to add inputs necessary for collision check inside randomSample
			q = integrate_with_graph(s, vertices, components, maxNeighbors, numVertices++, epsilon, numOfDOFs, 0, 0, 0);   // need to add inputs necessary for collision check
		}
	}

	// query
	Node *qGoal = integrate_with_graph(goalAngles, vertices, components, maxNeighbors, numVertices, epsilon, numOfDOFs, 0, 0, 0);  // need to add inputs necessary for collision check
	Node *qStart = integrate_with_graph(startAngles, vertices, components, maxNeighbors, ++numVertices, epsilon, numOfDOFs, 0, 0, 0);  // need to add inputs necessary for collision check
	numVertices++;

	cout << "num of vertices: " << numVertices << endl;
	printNumComponents(components);

	cout << "goal comp id: " << qGoal->compId << endl <<  "start comp id: " << qStart->compId << endl;

	if (qGoal->compId == qStart->compId)
	{
		aStarSearch(vertices, qStart, qGoal, numOfDOFs, plan, planLength);
	}
	else
	{
		cout << "no viable path exists\n";
	}

	// cout << "Aaj" << endl;
	// removeFromGraph(qGoal, vertices, components);
	// cout << "Gaand" << endl;
	// removeFromGraph(qStart, vertices, components);
	// cout << "lag" << endl;
	// resetNodeAstarParams(vertices);
	// cout << "rahii" << endl;
	// qGoal, qStart = nullptr;
	// cout << "haiiii" << endl;
	// numVertices -= 2;

	// deletePointers(vertices);  // double check this to make sure all pointers deleted
	// when should pointers be deleted? will need to keep them for accessing map on successive query executions
}

// void jointCallback(const sensor_msgs::JointState::ConstPtr& js)
// {
//     string temp = "[";
//     for (double a : js->position) {
//         temp += to_string(a) + ", ";
//     }
//     temp = temp.substr(0, temp.length() - 2) + "]";
//     ROS_INFO("I heard: %s", temp);
// 	// ROS_ERROR("I heard");
// }


int main(int argc, char **argv) {
	clock_t startTime = clock();
    int numOfDOFs = 6;
	// double qStart[numOfDOFs] = {PI/2, PI/4, 0, -PI/4, 0, 0};
	// double qGoal[numOfDOFs]  = {PI/4, 0, PI/2, 0, -PI/4, 0};
	// double qStart[numOfDOFs] = {2.042, -1.047, -0.785, 0, -1.309, -1.099};
	// double qGoal[numOfDOFs]  = {-2.1118, 0.855, 1.797, -3.14, -0.488, -2.1118};
	// double qStart[numOfDOFs] = {0, -0.78, 0, 0, 0, 0};
	// double qGoal[numOfDOFs]  = {0, 0.78, 0, 0, 0, 0};
	double qStart[numOfDOFs] = {-120*PI/180, 60*PI/180, 90*PI/180, 0, 0, 0};
	double qGoal[numOfDOFs]  = {-60*PI/180, 60*PI/180, 90*PI/180, 0, 0, 0};
	double **plan = 0;
	int planLength;

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
	
	// static const std::string PLANNING_GROUP = "arm";

	// // The :move_group_interface:`MoveGroupInterface` class can be easily
	// // setup using just the name of the planning group you would like to control and plan for.
	// moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	// ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
	
	// string j1 = "arm"; 

	// vector<double> q_check = {1.66, -0.9, -1.06, -3.14, -1.99, 1.48};
	// vector<double> q_check = {-1.52,0.226,1.85,3.07,-0.38,-1.47};
	vector<double> q_check = {0,3.3,0.0,0.0,0.0,0.0};
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
	// planning_scene::PlanningScene planning_scene(kinematic_model);
	

	collision_detection::CollisionRequest collision_request;
  	collision_detection::CollisionResult collision_result;

	// possible bug
	// moveit::core::RobotState copied_state = planning_scene.getCurrentState();
	// moveit::core::RobotState copied_state;
	// moveit::core::RobotStatePtr copied_state(new moveit::core::RobotState(kinematic_model));
	// collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
	// copied_state.setJointGroupPositions(PLANNING_GROUP, q_check);

	planning_scene::PlanningScene* planning_scene = new planning_scene::PlanningScene(kinematic_model);
	is_valid_K(planning_scene, q_check);
	delete planning_scene;

	int n_samples = 20000;
	// vector<double*> valid_states;
	// for (int k = 0; k < n_samples; k++)
	// {
	// 	// cout << "debuggg: "<< k << endl;
	// 	double *s = new double[numOfDOFs];
	// 	randomSample(s, numOfDOFs, 0, 0, 0);
	// 	copied_state.setJointGroupPositions(PLANNING_GROUP, s);
	// 	// planning_scene.isStateValid(copied_state);
	// 	// planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);

	// 	// if (!collision_result.collision)
	// 	if(planning_scene.isStateColliding(copied_state) == 0)
	// 	// if (true)
	// 	{
	// 		valid_states.push_back(s);
	// 	}
	// }
	// cout << "Size check::::::" << valid_states.size() << endl;

	std::ofstream outFile("example.txt");
    // std::copy(valid_states.begin(), valid_states.end(), output_iterator);
	for (const auto &e : valid_states) 
	{
		outFile << *e << " " << *(e+1) << " " << *(e+2) << " " << *(e+3) << " " << *(e+4) << " " << *(e+5) << " " << "\n";
	}

	// planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);



	// init_scene(kinematic_model);
	// moveit::core::RobotState copied_state2 = planning_scene.getCurrentState();
	// copied_state2.setJointGroupPositions(PLANNING_GROUP, q_check);
	// bool validity = planning_scene.isStateValid(copied_state, PLANNING_GROUP);
	// ROS_INFO_STREAM("Test 7: is state valid - Current state is " << (validity ? "in" : "not in") << " collision");

	// ......................PLANNER........................
	// plannerPRM(numOfDOFs, qStart, qGoal, plan, planLength);
	// cout << "Runtime: " << (float)(clock() - startTime)/ CLOCKS_PER_SEC << endl;
	// if (planLength <= 2)
	// {
	// 	cout << "Exiting..." << endl;
	// 	return 0;
	// }
	// cout << "Should output plan details " << endl;
	// printPlan(plan, planLength, numOfDOFs);

	// vector<double> q_check1 = {1.57,-1.57,0.0,0.0,0.0,0.0};

	// moveit_msgs::PlanningScene ps;
	// ps.robot_state.joint_state.position = q_check1;
	// planning_scene.setPlanningSceneMsg(ps);
	// cout << "Check planning scene: " << ps.robot_model_name << endl;

	// moveit_msgs::GetPlanningScene getScene;
	// moveit_msgs::RobotState scanpro_robot_state;
	// moveit_msgs::PlanningSceneComponents ps_components;
	// moveit_msgs::GetStateValidity isValid;
	// moveit_msgs::GetStateValidityRequest req;
	// moveit_msgs::GetStateValidityResponse res;
	// scanpro_robot_state.joint_state.position = q_check1;

	// ros::ServiceClient get_planning_scene = n.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
	// ros::ServiceClient check_val_service = n.serviceClient<moveit_msgs::GetStateValidity>("check_state_validity");
	// check_val_service.waitForExistence();

	// getScene.request.components = ps_components;
	// get_planning_scene.call(getScene);

	// isValid.request.robot_state = scanpro_robot_state;
	// // isValid.request.robot_state = ps.robot_state;
	// isValid.request.group_name = PLANNING_GROUP;

	// check_val_service.call(isValid);
	// // isValid.response = res;

	// cout << "Checking validity: " << isValid.response << endl;

	// sensor_msgs::JointState js;
	trajectory_msgs::JointTrajectory jt;
	
	// // ros::Publisher js_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
	int i,j = 0;
	double dt = 0.01;
	ros::Duration time;

	ros::Publisher jt_pub = n.advertise<trajectory_msgs::JointTrajectory>("/scan_pro_robot/arm_controller/command", 100);
	ros::Duration(1,0).sleep();
	jt.joint_names =  {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
	
	ros::Rate rate(1);
	jt.header.stamp = ros::Time::now();

	// .....................IMPORTANT..............................
	// jt.points.resize(planLength);

	// vector<vector<double>> store_plan;
	// cout << "Publishing plan to Gazebo" << endl;
	// cout << planLength << endl;
	// cout << plan[planLength-1][0] << endl;
	// while (i < planLength)
	// {
	// 	vector<double> q_plan;
	// 	// // cout << "Here!" << endl;	
	// 	cout << "ros plan check: " << plan[0] << endl;
	// 	for (int j = 0; j < numOfDOFs; j++)
	// 	{
	// 		// cout << plan[i][j] << endl;
	// 		q_plan.push_back(plan[i][j]);
	// 	}
	// 	store_plan.push_back(q_plan);
		
	// 	jt.points[i].positions = q_plan;
	// 	jt.points[i].time_from_start = {i+1,0};
		
	// 	i++;
	// 	cout << "There!" << endl;
	// }
	// .....................IMPORTANT..............................

	// while (ros::ok())
	// {
	// 	// vector<double> q_test = {0.5,0.5,0.02,0.5,0.8,1.0,0.0};
	// 	// for (int i = 0; i < numOfDOFs; i++) 
	// 	// {
	// 	// 	q_test.push_back(plan[0][i]);
	// 	// }
	// 	// q_test.push_back(0);
	// 	// js.position = q_test;
	
	// .....................TESTING!!!..............................
	// jt.points.resize(2);
	// jt.points[0].positions = {0.5,0.5,3.3,0.5,0.8,1.0};
	// jt.points[1].positions = {0.5,0.5,1.2,0.8,0.8,1.0};
	// jt.points[0].positions = {0, -0.78, 0, 0, 0, 0};
	// jt.points[1].positions = {0, 0.78, 0, 0, 0, 0};
	// jt.points[0].positions = {0.0,0.0,3.3,0.0,0.0,0.0};

	// jt.points[0].positions = {0,0,0,0,0,0};
	// jt.points[1].positions = {-2.1118, 0.855, 1.797, -3.14, -0.488, -2.1118};
	// jt.points[2].positions = {2.042, -1.047, -0.785, 0, -1.309, -1.099};
	
	// // 	// jt.points[0].positions = {0.0,0.0,3.3,0.0,0.0,0.0};
	// // 	// trajectory_msgs::JointTrajectoryPoint jtp;
		
	// // 	// jtp.positions = store_plan[0];
	// // 	// jtp.time_from_start = {1,0};
	// // 	// jt.points[j].positions = store_plan[j];
	
		
	// jt.points[0].time_from_start = {1,0};
	// jt.points[1].time_from_start = {5,0};
	// jt.points[2].time_from_start = {10,0};
	// .....................TESTING!!!..............................

	// 	// jt.points.push_back(jtp);
	// 	// jt.points[i] = jtp;
	jt_pub.publish(jt);
		
		// ros::spinOnce();
	// 	rate.sleep();
	// 	// j++;
	// }

	// jt_pub.publish(jt);
	
    // ros::Subscriber sub = n.subscribe("/joint_states", 10, jointCallback);
    // ros::spin();
	// loop_rate.sleep();

    return 0;
}

