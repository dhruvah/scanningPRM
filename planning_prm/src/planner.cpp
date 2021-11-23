/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <time.h>
#include <unordered_map>
#include <queue>
#include <stack>
#include <iostream>

#define PI 3.141592654

using namespace std;

double euclidDist(double *v1, double *v2, int numofDOFs)
{
	double result = 0;
	for (int i = 0; i < numofDOFs; i++)
	{
		result += (v1[i] - v2[i])*(v1[i] - v2[i]);
	}
	return sqrt(result);
}

struct Node {
	int id;
	int comdId;
	double *angles;
	vector<int>* neighbors;
	double f;
	double g;
	Node *parent;
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

void randomSample(double *angles, int numOfDOFs, double *map, int x_size, int y_size) {
	for (int i = 0; i < numOfDOFs; i++)
	{
		angles[i] = randomAngleGen();
	}
	if (true) // need to change this to collision check, use fcl library
	{
		return;
	}
	return randomSample(angles, numOfDOFs, map, x_size, y_size);
}

void printAngles(double *angles, int size) {
    string temp = "[";
    for (int i = 0; i < size; i++) {
        temp += to_string(angles[i]) + ", ";
    }
    temp = temp.substr(0,temp.length() - 2) + "]\n";
    cout << temp;
}

void plannerPRM(int numOfDOFS) {

}

int main() {
    int numDOFs = 6;
    double s[numDOFs];
    for (int i = 0; i < 10; i++){
        randomSample(s, numDOFs, 0, 0, 0);	// need to add inputs necessary for collision check inside randomSample
        printAngles(s, numDOFs);
    }
    return 0;
}
