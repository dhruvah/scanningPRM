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
static int planTime = 5;
static int K = 10000;

struct Node;
unordered_map<int, Node*> vertices;
unordered_map<int, vector<int>> components;
bool mapGenerated;
int numVertices = 0;
const int maxNeighbors = 20;
const double epsilon = PI/4; //2 <<<<<<<<< parameter tuning
const double i_step = PI/16;
const string PLANNING_GROUP = "arm";

bool is_valid_K(const planning_scene::PlanningScene* planning_scene, double* angles, int numOfDOFs)
{
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

	// void serialize(FILE* f, bool bWrite, int numOfDOFs) {
	// 	this->resetAstarParams();

	// 	if (bWrite) {
	// 		fwrite(&id, sizeof(id), 1, f);
	// 		fwrite(&compId, sizeof(compId), 1, f);
	// 		fwrite(&angles[0], sizeof(double), numOfDOFs, f);
	// 		fwrite(&neighbors[0], sizeof(int), neighbors.size(), f);
	// 	}
	// 	else {
	// 		fread(&id, sizeof(id), 1, f);
	// 		fread(&compId, sizeof(compId), 1, f);
	// 		fread(&angles[0], sizeof(double), numOfDOFs, f);
	// 		fread(&neighbors[0], sizeof(int), neighbors.size(), f);
	// 	}
	// }

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
	return (randAngle);
}

void randomSample(double *angles, int numOfDOFs, const planning_scene::PlanningScene* planning_scene) {
	for (int i = 0; i < numOfDOFs; i++)
	{
		if (i >= 3) {
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
	components[oldId].clear();
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

Node* integrate_with_graph(	double *angles, int idx, int numOfDOFs, const planning_scene::PlanningScene* planning_scene)
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

// void saveData(int numOfDOFs) {
// 	FILE *fv = fopen("vertices_data", "wb");
//     for(const auto& it : vertices){
//         fwrite(&(it.first), sizeof(int), 1, fv);
//         it.second->serialize(fv, true, numOfDOFs);
//     }
//     fclose(fv);

// 	FILE *fc = fopen("components_data", "wb");
//     for(const auto& it : components){
//         fwrite(&(it.first), sizeof(int), 1, fc);
//         fwrite(&(it.second)[0], sizeof(int), it.second.size(), fc);
//     }
//     fclose(fc);
// }

// void loadData() {
//     FILE *fv = fopen("vertices_data", "rb");
//     int keyv;
//     Node* valv;
//     while(fread(&keyv, 8, 1, fv)){
//         fread(&valv, 1, 1, fv);
//         vertices[keyv] = valv;
//     }
//     fclose(fv);	

//     FILE *fc = fopen("components_data", "rb");
//     int keyc;
//     vector<int> valc;
//     while(fread(&keyc, 8, 1, fc)){
//         fread(&valc, 1, 1, fc);
//         components[keyc] = valc;
//     }
//     fclose(fc);
// }

void plannerPRM(int numOfDOFs, double *startAngles, double *goalAngles, double **&plan, int &planLength, const planning_scene::PlanningScene* planning_scene) {
	if (!mapGenerated) {
		// pre-processing
		mapGenerated = true;
		deletePointers(vertices);
		clearOldData(vertices, components);

		Node *q;
		clock_t prmStartTime = clock();
		// while (((double)(clock() - prmStartTime) / CLOCKS_PER_SEC) < planTime)
		for (int k = 0; k < K; k++)
		{
			double *s = new double[numOfDOFs];
			randomSample(s, numOfDOFs, planning_scene);   // need to add inputs necessary for collision check inside randomSample
			q = integrate_with_graph(s, numVertices++, numOfDOFs, planning_scene);   // need to add inputs necessary for collision check
		}
	}

	// query
	Node *qGoal = integrate_with_graph(goalAngles, numVertices, numOfDOFs, planning_scene);  // need to add inputs necessary for collision check
	Node *qStart = integrate_with_graph(startAngles, ++numVertices, numOfDOFs, planning_scene);  // need to add inputs necessary for collision check
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

	int sum = 0;
	for (const auto& it:vertices) {
		sum += it.second->neighbors.size();
	}
	cout << "avg # of neighbors: " << (double)sum/numVertices << endl;

	// cout << "Aaj" << endl;
	removeFromGraph(qGoal, vertices, components);
	// cout << "Gaand" << endl;
	removeFromGraph(qStart, vertices, components);
	// cout << "lag" << endl;
	resetNodeAstarParams(vertices);
	// cout << "rahii" << endl;
	qGoal, qStart = nullptr;
	// cout << "haiiii" << endl;
	numVertices -= 2;

	// deletePointers(vertices);  // double check this to make sure all pointers deleted
	// when should pointers be deleted? will need to keep them for accessing map on successive query executions
}


int main(int argc, char **argv) {
	clock_t startTime = clock();
    int numOfDOFs = 6;
	int numOfWaypoints = 10;
	double qGoal[numOfDOFs] = {-120*PI/180, 60*PI/180, 90*PI/180, 0, 0, 0};
	double qStart[numOfDOFs]  = {-60*PI/180, 60*PI/180, 90*PI/180, 0, 0, 0};
	double **plan = 0;
	int planLength = 0;

	// ....................CREATE SCENE.....................
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

	planning_scene::PlanningScene* planning_scene = new planning_scene::PlanningScene(kinematic_model);

	// ......................PLANNER........................
	plannerPRM(numOfDOFs, qStart, qGoal, plan, planLength, planning_scene);
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
	deletePointers(vertices);

    return 0;
}

