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

#include <ros/ros.h>

#include <moveit_msgs/PositionIKRequest.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/GetStateValidity.h>
#include <tf/tf.h>

#include <sensor_msgs/JointState.h>

#include <std_msgs/String.h>

#define PI 3.141592654

using namespace std;

// need to figure out exactly how this is going to work with ROS- compiling executable and running seperate times wont work,
// will need to save data elsewhere in that case
struct Node;
static unordered_map<int, Node*> vertices;
static unordered_map<int, vector<int>> components;
static bool mapGenerated;
static int numVertices = 0;
static int maxNeighbors = 10;
static int epsilon = 2;
static double i_step = 0.05;


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


double randomAngleGen() {
	double randAngle;
	static bool init = false;
	if (!init)
	{
		srand(time(NULL));
		init = true;
	}
	randAngle = ((double)rand() / RAND_MAX) * 2 * PI; //generates random double between 0 and 2*PI
	return (randAngle);
}

void randomSample(double *angles, int numOfDOFs, double *map, int xSize, int ySize) {
	for (int i = 0; i < numOfDOFs; i++)
	{
		angles[i] = randomAngleGen();
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
		cout << "Path found\n";
		backtrackPRM(best, numOfDOFs, plan, planLength);
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
		int K = 20000;
		clock_t prmStartTime = clock();
		// while (((double)(clock() - prmStartTime) / CLOCKS_PER_SEC) < tf)
		for (int k = 0; k < K; k++)
		{
			double *s = new double[numOfDOFs];
			randomSample(s, numOfDOFs, 0, 0, 0);   // need to add inputs necessary for collision check inside randomSample
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

	removeFromGraph(qGoal, vertices, components);
	removeFromGraph(qStart, vertices, components);
	resetNodeAstarParams(vertices);
	qGoal, qStart = nullptr;
	numVertices -= 2;

	// deletePointers(vertices);  // double check this to make sure all pointers deleted
	// when should pointers be deleted? will need to keep them for accessing map on successive query executions
}

void jointCallback(const sensor_msgs::JointState::ConstPtr& js)
{
    string temp = "[";
    for (double a : js->position) {
        temp += to_string(a) + ", ";
    }
    temp = temp.substr(0, temp.length() - 2) + "]";
    ROS_INFO("I heard: %s", temp);
	// ROS_ERROR("I heard");
}

int main(int argc, char **argv) {
	clock_t startTime = clock();
    int numOfDOFs = 5;
	double qStart[numOfDOFs] = {PI/2, PI/4, 0, -PI/4, 0};
	double qGoal[numOfDOFs]  = {PI/4, 0, PI/2, 0, -PI/4};
	double **plan = 0;
	int planLength;

	plannerPRM(numOfDOFs, qStart, qGoal, plan, planLength);
	printPlan(plan, planLength, numOfDOFs);

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
	sensor_msgs::JointState js;
	ros::Publisher js_pub = n.advertise<sensor_msgs::JointState>("joint_state", 1);
	for (int i = 0; i < numOfDOFs; i++) {
		js.position.push_back(plan[0][i]);
	}
	js_pub.publish(js);
    // ros::Subscriber sub = n.subscribe("/joint_states", 10, jointCallback);
    // ros::spin();    

	cout << "Runtime: " << (float)(clock() - startTime)/ CLOCKS_PER_SEC << endl;

    return 0;
}

