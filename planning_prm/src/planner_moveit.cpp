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
#include <fstream>

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

#define PARSE_ID 0
#define PARSE_COMP_ID 1
#define PARSE_ANGLES 2
#define PARSE_NEIGHBORS 3

using namespace std;

// static int planTime = 5;
static int K = 15000;

const double jointMin[] = {-170*PI/180, -100*PI/180, -119*PI/180, -190*PI/180, -120*PI/180};
const double jointMax[] = {170*PI/180, 135*PI/180, 169*PI/180, 190*PI/180, 120*PI/180};
const double numPlanningJoints = 3;

int numVertices = 0;
const int maxNeighbors = 20;
const double epsilon = PI/4; //2 <<<<<<<<< parameter tuning
const double i_step = PI/64;

bool is_valid_K(const planning_scene::PlanningScene* planning_scene, double* angles, int numOfDOFs)
{
	const string PLANNING_GROUP = "arm";
	vector<double> q_check;
	for (int i = 0; i < numOfDOFs; i++) {
		q_check.push_back(angles[i]);
	}

	moveit::core::RobotState copied_state = planning_scene->getCurrentState();
	copied_state.setJointGroupPositions(PLANNING_GROUP, q_check);
	
	return !planning_scene->isStateColliding(copied_state);
}

double euclidDist(const double *v1, const double *v2, int numOfDOFs)
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

	Node(){}

	Node(int idIn, int compIdIn, double *anglesIn)
	{
		this->id = idIn;
		this->compId = compIdIn;
		this->angles = anglesIn;
		this->g = INT16_MAX;
	}

	string toString(int numOfDOFs) {
		string s = "id: " + to_string(id) + "\n";
		s += "compId: " + to_string(compId) + "\n";
		s += "angles: ";
		for (int i = 0; i < numOfDOFs; i++) {
			s += to_string(angles[i]) + ", ";
		}
		s = s.substr(0, s.length() - 2) + "\n";
		s += "neighbors: ";
		for (int n : neighbors) {
			s += to_string(n) + ", ";
		}
		s = s.substr(0, s.length() - 2) + "\n";
		return s;
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

	void writeData(ofstream &f, int numOfDOFs) {
		f << id << endl;
		f << compId << endl;

		for (int i = 0; i < numOfDOFs; i++) {
			f << angles[i] << " ";
		}
		f << endl;

		for (int n : neighbors) {
			f << n << " ";
		}
		f << endl << endl;
	}

	void readData(ifstream &f, string &line, int parser, unordered_map<int, Node*> &vertices, int numOfDOFs) {
		if (parser == PARSE_ID) {
			this->id = stoi(line);
		} else if (parser == PARSE_COMP_ID) {
			this->compId = stoi(line);
		} else if (parser == PARSE_ANGLES) {
			this->angles = (double *)malloc(numOfDOFs*sizeof(double));
			int spacePos;
			for (int i = 0; i < numOfDOFs; i++) {
				spacePos = line.find(" ");
				this->angles[i] = stod(line.substr(0, spacePos));
				line.erase(0, spacePos + 1);
			}
		} else if (parser == PARSE_NEIGHBORS) {
			int spacePos;
			while ((spacePos = line.find(" ")) != string::npos) {
				this->neighbors.push_back(stoi(line.substr(0, spacePos)));
				line.erase(0, spacePos + 1);
			}
			// cout << this->toString(numOfDOFs);
		}
		this->resetAstarParams();
		vertices[this->id] = this;
	}

};

struct NodeCompare
{
    bool operator()(Node *n1, Node *n2)
    {
        return n1->f > n2->f;
    }
};

double randomAngleGen() {
	double randAngle;
	static bool init = false;
	if (!init)
	{
		srand(time(NULL));
		init = true;
	}
	randAngle = ((double)rand() / RAND_MAX) * 2 * PI; //generates random double between 0 and 2*PI
	return randAngle;
}

double constrainedRandomAngleGen(int i) {
	double randAngle;
	static bool init = false;
	if (!init)
	{
		srand(time(NULL));
		init = true;
	}
	randAngle = jointMin[i] + ((double)rand() / RAND_MAX)*(jointMax[i] - jointMin[i]); 
	return (randAngle);
}

void randomSample(double *angles, int numOfDOFs, const planning_scene::PlanningScene* planning_scene) {
	for (int i = 0; i < numOfDOFs; i++)
	{
		if (i >= numPlanningJoints) {
			angles[i] = 0;
			continue;
		}
		angles[i] = constrainedRandomAngleGen(i);
	}
	if (is_valid_K(planning_scene, angles, numOfDOFs)) // need to change this to collision check, use fcl library
	{
		return;
	}
	return randomSample(angles, numOfDOFs, planning_scene);
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
	components.erase(oldId);
	// components[oldId].clear();
}

bool obstacleFree(const double* q1, const double *q2, int numOfDOFs, const planning_scene::PlanningScene* planning_scene) {
	int numSteps = (int)(euclidDist(q1, q2, numOfDOFs) / i_step);
	numSteps = (numSteps > 0) ? numSteps : 1;
	double temp[numOfDOFs];
	for (int step = 0; step < numSteps; step++)
	{
		for (int i = 0; i < numOfDOFs; i++)
		{
			temp[i] = q1[i] + (q2[i] - q1[i]) * ((double)step / (double)numSteps);
		}
		if (!is_valid_K(planning_scene, temp, numOfDOFs)) {
			// cout << "collision\n";
			return false;
		}
	}
	return true;
}

Node* integrate_with_graph(double *angles, unordered_map<int, Node*> &vertices, unordered_map<int, vector<int>> &components, int idx, int numOfDOFs, const planning_scene::PlanningScene* planning_scene)
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
			if (q->neighbors.size() < maxNeighbors)  // could also be (q->comp_id != it.second->comp_id) if only connecting components once (this was a problem before though)
			{
				if (obstacleFree(it.second->angles, angles, numOfDOFs, planning_scene))	// need to implement obstacleFree function here (interpolate and check for collisions between configs)
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

// void removeFromGraph(Node *&n, unordered_map<int, Node*> &vertices, unordered_map<int, vector<int>> &components)
// {
// 	for (int id : n->neighbors)
// 	{
// 		vertices[id]->neighbors.pop_back();
// 	}
// 	vertices.erase(n->id);
// 	components[n->compId].pop_back();
// 	delete n;
// }

void deletePointers(unordered_map<int, Node*> &vertices) {
	Node *q;
	double *a;
	for (auto& kv: vertices) {
		q = kv.second;
		a = q->angles;
		delete q, a;
	}
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
		path.push(temp->angles);
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

void saveData(unordered_map<int, Node*> &vertices, unordered_map<int, vector<int>> &components, int numOfDOFs) {
	ofstream fv;
	fv.open("ros_vertices_data.txt", ios::out);
    for(const auto& it : vertices){
        it.second->writeData(fv, numOfDOFs);
    }
    fv.close();

	// ofstream fc;
	// fc.open("components_data.txt", ios::out);
    // for(const auto& it : components){
    //     fc << it.first << endl;
    //     for (int n : it.second) {
	// 		fc << n << " ";
	// 	}
	// 	fc << endl << endl;
    // }
    // fc.close();
}

void loadData(unordered_map<int, Node*> &vertices, unordered_map<int, vector<int>> &components, int numOfDOFs) {

    ifstream fv;
	fv.open("ros_vertices_data.txt", ios::in);
    string line;
	int parser = PARSE_ID;
    Node* n = new Node();
	if (fv.is_open()) {
		while (getline(fv, line)) {
			// cout << "parser: " << parser << endl;
			// cout << line << endl;
			if (line.empty()) {
				n = new Node();
				parser = PARSE_ID;
			} else {
				n->readData(fv, line, parser++, vertices, numOfDOFs);
			}
		}
	}
	else {
		cout << "could not open file\n";
	}
	fv.close();

	// ifstream fc;
	// fc.open("components_data.txt", ios::in);
	// if (fc.is_open()) {
	// 	int spacePos;
	// 	int nodeId;
	// 	while (getline(fc, line)) {
	// 		if (line.empty()) {
	// 			continue;
	// 		}
	// 		if (spacePos = line.find(" ") == string::npos) {
	// 			nodeId = stoi(line);
	// 		} else {
	// 			while ((spacePos = line.find(" ")) != string::npos) {
	// 				components[nodeId].push_back(stoi(line.substr(0, spacePos)));
	// 				line.erase(0, spacePos + 1);
	// 			}				
	// 		}
	// 	}
	// }
	// else {
	// 	cout << "could not open file\n";
	// }
	// fc.close();    
}

void plannerPRM(unordered_map<int, Node*> &vertices, unordered_map<int, vector<int>> &components, bool isPrmBuild, int numOfDOFs, double *startAngles, double *goalAngles, double **&plan, int &planLength, const planning_scene::PlanningScene* planning_scene) {
	// if (isPrmBuild) {
	// 	// pre-processing
	// 	Node *q;
	// 	clock_t prmStartTime = clock();
	// 	// while (((double)(clock() - prmStartTime) / CLOCKS_PER_SEC) < planTime)
	// 	for (int k = 0; k < K; k++)
	// 	{
	// 		double *s = new double[numOfDOFs];
	// 		randomSample(s, numOfDOFs, planning_scene);   // need to add inputs necessary for collision check inside randomSample
	// 		q = integrate_with_graph(s, vertices, components, numVertices++, numOfDOFs, planning_scene);   // need to add inputs necessary for collision check
	// 	}
	// 	saveData(vertices, components, numOfDOFs);
	// }

	// // query
	// Node *qGoal = integrate_with_graph(goalAngles, vertices, components, numVertices, numOfDOFs, planning_scene);  // need to add inputs necessary for collision check
	// Node *qStart = integrate_with_graph(startAngles, vertices, components, ++numVertices, numOfDOFs, planning_scene);  // need to add inputs necessary for collision check
	// numVertices++;

	// cout << "num of vertices: " << numVertices << endl;
	// cout << "num of components: " << components.size() << endl;

	// cout << "goal comp id: " << qGoal->compId << endl <<  "start comp id: " << qStart->compId << endl;

	// if (qGoal->compId == qStart->compId)
	// {
	// 	aStarSearch(vertices, qStart, qGoal, numOfDOFs, plan, planLength);
	// }
	// else
	// {
	// 	cout << "no viable path exists\n";
	// }

	// int sum = 0;
	// for (const auto& it:vertices) {
	// 	sum += it.second->neighbors.size();
	// }
	// cout << "avg # of neighbors: " << (double)sum/numVertices << endl;

	// deletePointers(vertices);
}

void buildPRM(unordered_map<int, Node*> &vertices, unordered_map<int, vector<int>> &components, int numOfDOFs, const planning_scene::PlanningScene* planning_scene, bool save) {
	// pre-processing
	Node *q;
	clock_t prmStartTime = clock();
	// while (((double)(clock() - prmStartTime) / CLOCKS_PER_SEC) < planTime)
	for (int k = 0; k < K; k++)
	{
		double *s = new double[numOfDOFs];
		randomSample(s, numOfDOFs, planning_scene);   // need to add inputs necessary for collision check inside randomSample
		q = integrate_with_graph(s, vertices, components, numVertices++, numOfDOFs, planning_scene);   // need to add inputs necessary for collision check
	}
	if (save) {
		saveData(vertices, components, numOfDOFs);
	}

	cout << "num of vertices: " << numVertices << endl;
	cout << "num of components: " << components.size() << endl;

	int sum = 0;
	for (const auto& it:vertices) {
		sum += it.second->neighbors.size();
	}
	cout << "avg # of neighbors: " << (double)sum/numVertices << endl;
}

void waypointsPath(unordered_map<int, Node*> &vertices, unordered_map<int, vector<int>> &components, int numOfDOFs, double **waypoints, int numOfWaypoints, double **&globalPlan, int &globalPlanLength, const planning_scene::PlanningScene* planning_scene) {
	// query
	double *goalAngles;
	double *startAngles;
	for (int i = 0; i < numOfWaypoints - 1; i++) {
		startAngles = waypoints[i];
		goalAngles = waypoints[i+1];
		Node *qGoal = integrate_with_graph(goalAngles, vertices, components, numVertices, numOfDOFs, planning_scene);  // need to add inputs necessary for collision check
		Node *qStart = integrate_with_graph(startAngles, vertices, components, ++numVertices, numOfDOFs, planning_scene);  // need to add inputs necessary for collision check
		numVertices++;

		cout << "goal comp id: " << qGoal->compId << endl <<  "start comp id: " << qStart->compId << endl;

		double **localPlan = 0;
		int localPlanLength = 0;

		if (qGoal->compId == qStart->compId)
		{
			aStarSearch(vertices, qStart, qGoal, numOfDOFs, localPlan, localPlanLength);
			// printPlan(localPlan, localPlanLength, numOfDOFs);
			globalPlan = (double**)realloc(globalPlan, (globalPlanLength+localPlanLength)*sizeof(double**));
			for (int j = 0; j < localPlanLength; j++) {
				globalPlan[globalPlanLength + j] = (double*)malloc(numOfDOFs*sizeof(double*));
				globalPlan[globalPlanLength + j] = localPlan[j];
			}

			globalPlanLength += localPlanLength;
			localPlanLength = 0;
			localPlanLength = 0;
			resetNodeAstarParams(vertices);
		}
		else
		{
			cout << "found incomplete path\n";
			break;
		}
	}

	deletePointers(vertices);
}

int main(int argc, char **argv) {
	clock_t startTime = clock();
	unordered_map<int, Node*> vertices;
	unordered_map<int, vector<int>> components;
    int numOfDOFs = 6;
	int numOfWaypoints;
	double **waypoints;

	string line;
	ifstream myfile;
	myfile.open("waypt5.txt", ios::in);
	if (myfile.is_open())
	{
		int spacePos;
		int wp = 0;
		while (getline(myfile,line))
		{
			if (line.find(" ") == string::npos) {
				numOfWaypoints = stoi(line);
				waypoints = (double**)malloc(numOfWaypoints*sizeof(double**));
				continue;
			}
			waypoints[wp] = (double*)malloc(numOfDOFs*sizeof(double*));
			for (int i = 0; i < numOfDOFs; i++) {
				spacePos = line.find(", ");
				waypoints[wp][i] = PI*stoi(line.substr(0, spacePos))/180;
				line.erase(0, spacePos + 2);
			}
			wp++;
		}
		myfile.close();
	}
	else {
		cout << "cannot open file\n";
	}

	// double q1[numOfDOFs] = {-120*PI/180, 60*PI/180, 90*PI/180, 0, 0, 0};
	// double q2[numOfDOFs]  = {-90*PI/180, 20*PI/180, 150*PI/180, 0, 0, 0};
	// double q3[numOfDOFs]  = {-90*PI/180, 0*PI/180, 90*PI/180, 0, 0, 0};
	// double q4[numOfDOFs]  = {-60*PI/180, 60*PI/180, 90*PI/180, 0, 0, 0};

	// waypoints[0] = q1;
	// waypoints[1] = q2;
	// waypoints[2] = q3;
	// waypoints[3] = q4;

	double **plan = 0;
	int planLength = 0;
	bool isPrmBuild = true;
	// cout << "isPrmBuild: " << isPrmBuild << endl;

	// ....................CREATE SCENE.....................
    ros::init(argc, argv, "talker");
    ros::NodeHandle n("~");
	string check;
	n.getParam("param", check);
	ROS_INFO("Got parameter: %s", check.c_str());

	if (check.compare("loadmap") == 0) {
		cout << "loading map...\n";
		isPrmBuild = false;
		clock_t build_time = clock();
		loadData(vertices, components, numOfDOFs);
		numVertices = vertices.size();
		cout << "build runtime: " << (float)(clock() - build_time)/ CLOCKS_PER_SEC << endl;
	}
	else if(check.compare("buildmap") == 0) {
		cout << "building map...\n";
	}

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

	planning_scene::PlanningScene* planning_scene = new planning_scene::PlanningScene(kinematic_model);

	// ......................PLANNER........................
	// plannerPRM(vertices, components, isPrmBuild, numOfDOFs, qStart, qGoal, plan, planLength, planning_scene);
	if (isPrmBuild) {
		buildPRM(vertices, components, numOfDOFs, planning_scene, true);
	}
	waypointsPath(vertices, components, numOfDOFs, waypoints, numOfWaypoints, plan, planLength, planning_scene);
	delete planning_scene; // DO NOT DELETE THIS LINE
	cout << "Runtime: " << (float)(clock() - startTime)/ CLOCKS_PER_SEC << endl;
	if (planLength < 2)
	{
		cout << "Exiting..." << endl;
		return 0;
	}
	cout << "Should output plan details " << endl;
	printPlan(plan, planLength, numOfDOFs);

	trajectory_msgs::JointTrajectory jt;

	int i = 0,j = 0;
	double dt = 0.01;
	// ros::Duration time;

	ros::Publisher jt_pub = n.advertise<trajectory_msgs::JointTrajectory>("/scan_pro_robot/arm_controller/command", 100);
	ros::Duration(1,0).sleep();
	jt.joint_names =  {"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
	
	ros::Rate rate(1);
	jt.header.stamp = ros::Time::now();

	// .....................COPY PLAN AND PUBLISH..............................
	jt.points.resize(planLength);


	vector<vector<double>> store_plan;
	cout << "Publishing plan to Gazebo" << endl;
	while (i < planLength)
	{
		vector<double> q_plan;
		// // cout << "Here!" << endl;	
		// cout << "ros plan check: " << plan[0] << endl;
		for (int j = 0; j < numOfDOFs; j++)
		{
			// cout << plan[i][j] << endl;
			q_plan.push_back(plan[i][j]);
		}
		store_plan.push_back(q_plan);
		
		jt.points[i].positions = q_plan;
		jt.points[i].time_from_start = ros::Duration(i + 1);
		
		i++;
		// cout << "There!" << endl;
	}

	// for (trajectory_msgs::JointTrajectoryPoint p : jt.points) {
	// 	cout << "time: " << p.time_from_start << endl;
	// }

	jt_pub.publish(jt);

    return 0;
}

