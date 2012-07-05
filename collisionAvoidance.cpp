// Standard C++ Headers
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <list>
#include <map>
#include <algorithm>

// Needed ROS headers
#include "ros/ros.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/GoToWaypoint.h"
#include "AU_UAV_ROS/RequestWaypointInfo.h"

using namespace std;

int the_count;
ros::ServiceClient findGoal; // used for requesting waypoint information
ros::ServiceClient client;

//General environment information
const int NUM_PLANES = 8; //number of planes
const int NUM_WAYPOINTS = 50; //number of way points
const int FIELD_SIZE = 500; //size of field
const double VELOCITY = 11.176; //velocity of UAV
const double MAX_TURN = 0.3927; //maximum turn radius
const double FIELD_LONGITUDE = -85.490363; //longitude of upper-left corner of field
const double FIELD_LATITUDE = 32.606573; //latitude of upper-left corner of field
const double DEG_TO_MET_LAT = 110897.21; //m/deg latitude 
const double DEG_TO_MET_LONG = 93880.49; //m/deg longitude

//Store information about the UAVS
double **information; //used to create a multidimensional array of information about the planes
//position in array is plane ID
const int PLANE_X = 0; //x-coordinate of plane
const int PLANE_Y = 1; //y-coordinate of plane
const int PLANE_BEARING = 2; //bearing of plane
const int WAYPOINT_X = 3; //x-coordinate of current waypoint
const int WAYPOINT_Y = 4; //y-coordinate of current waypoint
const int PREVIOUS_X = 5;
const int PREVIOUS_Y = 6;
const int WAYPOINT_BEARING = 7;

//Special algorithm variables and constants
double zone; //distance to search for nearby planes
double separation_requirement;
vector<int>* nearby; //stores nearby planes
list<int> collisionsImpossible;
list<int> collisionsPossible;
vector<int> planesInDanger;
const double PI = 3.14159;
const double RAD = PI/180;

/* Purpose: Allocate memory for table storing information on UAVs and waypoints
   Input: None
   Output: Dynamically allocates space for table of size NUM_PLANES x 5
*/
void setupInformationTable();

/* Purpose: Populate information table
   Input: Pointer to a message
   Output: Populates information table with plane and waypoint information
*/
void populateInformationTable(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg);

/* Purpose: Print information table for testing purposes
   Input: None
   Output: Prints information table with plane and waypoint information
*/
void printInformationTable();

/* Purpose: Convert a longitudinal coordinate to an x-coordinate
   Input: coordComp is the longitudinal component
   Output: Returns equivalent x-component as double
*/
double convertLongitude(double coordComp);

/* Purpose: Convert a latitudinal coordinate to an y-coordinate
   Input: coordComp is the latitudinal component
   Output: Returns equivalent y-component as double
*/
double convertLatitude(double coordComp);

/* Purpose: Calculate turn angle in relation to north
   Input:
   ~ planeID is the ID of the plane whose angle is being manipulated
   ~ angle is the turn angle in relation to the forward facing plane
   Output: Returns turn angle in relation to north as a double
*/
double angle_i(int planeID, double angle);

/* Purpose: Calculate the x_coordinate of the position of a plane after one second of travel
   Input:
   ~ planeID is the ID of the plane whose next position is being calculated
   ~ angle is the turn angle in relation to the forward facing plane
   Output: Returns the x-coordinate of the next position of the plane as a double
*/
double projectedPosition_x(int planeID, double angle);

/* Purpose: Calculate the y_coordinate of the position of a plane after one second of travel
   Input:
   ~ planeID is the ID of the plane whose next position is being calculated
   ~ angle is the turn angle in relation to the forward facing plane
   Output: Returns the y-coordinate of the next position of the plane as a double
*/
double projectedPosition_y(int planeID, double angle);

/* Purpose: Calculate the distance from the target waypoint to a projected position of the focus plane
   Input:
   ~ planeID is the ID of the plane who's projected distance from the target waypoint is being calculated
   ~ angle is the turn angle in relation to the forward facing plane
   Output: Returns the distance from the target waypoint to the next position of the plane as a double
*/
double waypointDistance(int planeID, double angle);

double velocity_x(int planeID, double angle);

double velocity_y(int planeID, double angle);

/* Purpose: Calculate the distance from an aggressor plane to a projected position of the focus plane
   Input:
   ~ planeID is the ID of the focus plane who's projected distance from the aggressor plane is being calculated
   ~ aggressorID is the ID of the aggressor plane
   ~ angle is the turn angle in relation to the forward facing focus plane
   Output: Returns the distance from the aggressor plane to the next position of the plane as a double
*/
double dangerDistance(int planeID, int aggressorID, double angle);

/* Purpose: Calculate the danger factor of the focus plane with a specific turn angle
   Input:
   ~ planeID is the ID of the focus plane
   ~ angle is the turn angle in relation to the forward facing focus plane
   Output: Returns the danger factor as a double
*/
double dangerFactor(int planeID, double angle);

/* Purpose: Calculate the value of the cost function of the focus plane with a specific turn angle
   Input:
   ~ planeID is the ID of the focus plane
   ~ angle is the turn angle in relation to the forward facing focus plane
   Output: Returns the value of the evaluated cost function as a double
*/
double costFunction(int planeID, double angle);

/* Purpose: Find planes near the focus plane
   Input: planeID is the ID of the focus plane
   Output: nearby is a vector that holds a list of nearby planes
*/
void generateNearby(int planeID);

/* Purpose: Set an intermediary waypoint based on a turn angle
   Input:
   ~ planeID is the ID of the focus plane
   ~ angle is the turn angle in relation to the forward facing focus plane
   Output: Sets a waypoint at a distance equal in magnitude as the velocity of the plane in relation to the turn angle
*/
void setWaypoint(int planeID, double angle);

/* Purpose: Calculate x^y
   Input:
   ~ x is a double and the base of the power function
   ~ y is an int representing the exponent of the power function
   Output: Returns x^y as a double
*/
double powNew(double x, int y);

/* Purpose: Find the safest, most efficient turn angle
   Input: planeID is ID of the focus plane
   Output: Sets a waypoint using the calculated angle
*/
void findNoCollisionsAngle();

void simulatedAnnealing(int numPlanesInDanger);

void findCollisionsAngle();

void determineNextState();

void setupProperties();

bool paccept(double diff, int temperature, int probCost);

void loopCorrect();

bool okToStart;

const int TEMPERATURE = 40;
const int BASE_ITERATIONS = 3750;
const int NORM_CONSTANT = 1/(FIELD_SIZE * 16 * sqrt(2));

void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
  the_count++;
  populateInformationTable(msg);
  if(msg->planeID == NUM_PLANES - 1){
    okToStart = true;
    if(okToStart == true){
      determineNextState();
      cout<<"ok"<<endl;
    }
  }
}

int main(int argc, char **argv)
{
  setupInformationTable();
  setupProperties();
  
  //standard ROS startup
  ros::init(argc, argv, "collisionAvoidance");
  ros::NodeHandle n;
  
  //subscribe to telemetry outputs and create client for the avoid collision service and the goal giving service
  ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
  
  //create a service client for requesting waypoint information
  findGoal = n.serviceClient<AU_UAV_ROS::RequestWaypointInfo>("request_waypoint_info");

  client = n.serviceClient<AU_UAV_ROS::GoToWaypoint>("go_to_waypoint");

  //initialize counting
  srand(time(NULL));
  the_count = 0;

  //needed for ROS to wait for callbacks
  ros::spin();
  return 0;
}

//FUNCTIONS FOR SETTING UP INFORMATION TABLE

void setupInformationTable(){
  nearby = new vector<int>[NUM_PLANES];
  information = new double*[NUM_PLANES];
  for(int i = 0; i < NUM_PLANES; i++){
    information[i] = new double[8];
    information[i][PLANE_BEARING] = 0;
    information[i][PREVIOUS_X] = 0;
    information[i][PREVIOUS_Y] = 0;
  }
}

void setupProperties(){
  switch(NUM_PLANES){
  case 4:
    if(FIELD_SIZE == 500){
      zone = 60;
      separation_requirement = 24;
    }
    else{
      zone = 60;
      separation_requirement = 24;
    }
    break;
  case 8:
    if(FIELD_SIZE == 500){
      zone = 72;
      separation_requirement = 30;
    }
    else{
      zone = 3*VELOCITY;
      separation_requirement = 12;
    }
    break;
  case 16:
    if(FIELD_SIZE == 500){
      zone = 65;
      separation_requirement = 24;
    }
    else{
      zone = 5*VELOCITY;
      separation_requirement = 24;
    }
    break;
  case 32:
    if(FIELD_SIZE == 500){
      zone = 40;
      separation_requirement = 14;
    }
    else{
      zone = 5*VELOCITY;
      separation_requirement = 24;
    }
    break;
  }
}

void populateInformationTable(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg){
  int ID = msg->planeID;
  if(okToStart){
    information[ID][PREVIOUS_X] = information[ID][PLANE_X];
    information[ID][PREVIOUS_Y] = information[ID][PLANE_Y];
  }
  information[ID][PLANE_X] = convertLongitude(msg->currentLongitude);
  information[ID][PLANE_Y] = convertLatitude(msg->currentLatitude);
  if(okToStart){
    int deltaX = information[ID][PLANE_X] - information[ID][PREVIOUS_X];
    int deltaY = information[ID][PLANE_Y] - information[ID][PREVIOUS_Y];
    information[ID][PLANE_BEARING] = atan2(deltaX, deltaY); 
  }
  AU_UAV_ROS::RequestWaypointInfo goalSrv;
  goalSrv.request.planeID = msg->planeID;
  goalSrv.request.isAvoidanceWaypoint = false;
  goalSrv.request.positionInQueue = 0;
  if(findGoal.call(goalSrv)){
    information[ID][WAYPOINT_X] = convertLongitude(goalSrv.response.longitude);
    information[ID][WAYPOINT_Y] = convertLatitude(goalSrv.response.latitude);
    information[ID][WAYPOINT_BEARING] = (msg->targetBearing)*RAD;
  }
  //cout<<ID<<"   "<<information[ID][PLANE_BEARING]<<"   "<<information[ID][WAYPOINT_BEARING]<<endl;
}

void printInformationTable(){
  for(int i = 0; i < NUM_PLANES; i++){
    cout<<i<<"   "<<information[i][PLANE_X]<<"   "<<information[i][PLANE_Y]<<"   "<<information[i][WAYPOINT_X]<<"   "<<information[i][WAYPOINT_Y]<<"   "<<information[i][PLANE_BEARING]<<endl;
  }
}

double convertLongitude(double coordComp){
  return (coordComp - FIELD_LONGITUDE) * DEG_TO_MET_LONG;
}

double convertLatitude(double coordComp){
  return (coordComp - FIELD_LATITUDE) * DEG_TO_MET_LAT;
}

//FUNCTIONS FOR COMPUTING COST FUNCTION

double projectedPosition_x(int planeID, double angle){
  return VELOCITY * sin(information[planeID][PLANE_BEARING] + angle) + information[planeID][PLANE_X];
}

double projectedPosition_y(int planeID, double angle){
  return VELOCITY * cos(information[planeID][PLANE_BEARING] + angle) + information[planeID][PLANE_Y];
}

double waypointDistance(int planeID, double angle){
  double x = information[planeID][WAYPOINT_X] - projectedPosition_x(planeID,angle);
  double y = information[planeID][WAYPOINT_Y] - projectedPosition_y(planeID,angle);
  return sqrt(x * x + y * y);
}

double velocity_x(int planeID, double angle){
  return VELOCITY * sin(information[planeID][PLANE_BEARING] + angle);
}

double velocity_y(int planeID, double angle){
  return VELOCITY * cos(information[planeID][PLANE_BEARING] + angle);
}

double dangerDistance(int planeID, int aggressorID, map<int,double> solution){
  double LOS_x = information[aggressorID][PLANE_X] - information[planeID][PLANE_X];
  double LOS_y = information[aggressorID][PLANE_Y] - information[planeID][PLANE_Y];
  double LOS_angle = atan(LOS_x / LOS_y);
  double LOS_mag = sqrt(LOS_x * LOS_x + LOS_y * LOS_y);
  double relativeVelocityX = velocity_x(planeID,solution[planeID]) - velocity_x(aggressorID,solution[aggressorID]);
  double relativeVelocityY = velocity_y(planeID,solution[planeID]) - velocity_y(aggressorID,solution[aggressorID]);
  //double relativeVelocityMag = sqrt(powNew(relativeVelocityX,2) + powNew(relativeVelocityY,2));
  double relativeVelocityAngle = atan(relativeVelocityX / relativeVelocityY);
  double minDistance;
  double angleDifference = relativeVelocityAngle - LOS_angle;
  if(abs(angleDifference) < 90){
    minDistance = abs(LOS_mag*sin(angleDifference));
  }
  else{
    minDistance = LOS_mag;
  }
  return minDistance;
}

double dangerFactor(int planeID, map<int,double> solution){
  double factor = 0.0;
  for(int i = 0; i < (int)nearby[planeID].size(); i++){
    int aggressorID = nearby[planeID][i];
    factor += 1 / (1 + exp((dangerDistance(planeID,aggressorID,solution) - separation_requirement)));
  }
  return factor;
}

double costFunction(map<int,double> solution){
  double value = 0.0;
  for(int i = 0; i < (int)planesInDanger.size(); i++){
    value += dangerFactor(planesInDanger[i],solution);// + waypointDistance(planesInDanger[i],solution[planesInDanger[i]]) / NORM_CONSTANT;
  }
  return value;
}

void determineNextState(){
  collisionsImpossible.clear();
  collisionsPossible.clear();
  for(int i = 0; i < NUM_PLANES; i++){
    generateNearby(i);
    if(!nearby[i].empty())
      collisionsPossible.push_back(i);
    else
      loopCorrect();
  }
  //findNoCollisionsAngle();
  findCollisionsAngle();
}

void generateNearby(int planeID){
  nearby[planeID].clear();
  int tempX = information[planeID][PLANE_X];
  int tempY = information[planeID][PLANE_Y];
  for(int i = 0; i < NUM_PLANES; i++){
    if((i != planeID) && (information[i][PLANE_X] < tempX + zone) && (information[i][PLANE_X] > tempX - zone) && (information[i][PLANE_Y] < tempY + zone) && (information[i][PLANE_Y] > tempY - zone))
      nearby[planeID].push_back(i);
  }
}

void setWaypoint(int planeID, double angle){
  AU_UAV_ROS::GoToWaypoint srv;
    srv.request.planeID = planeID;
    srv.request.latitude = projectedPosition_y(planeID,angle) / DEG_TO_MET_LAT + FIELD_LATITUDE;
    srv.request.longitude = projectedPosition_x(planeID,angle) / DEG_TO_MET_LONG + FIELD_LONGITUDE;
    
    //these settings mean it is an avoidance maneuver waypoint AND to clear the avoidance queue
    srv.request.isAvoidanceManeuver = true;
    srv.request.isNewQueue = true;
    
    //check to make sure the client call worked (regardless of return values from service)
    client.call(srv);
}

//FUNCTIONS FOR SPECIALIZED MATH

/*
double powNew(double x, int it){
  double y = 1;
  for(int i = 0; i < it; i++)
    y *= x;
  return y;
}
*/

/*void findNoCollisionsAngle(){
  while(!collisionsImpossible.empty()){
    int planeID = collisionsImpossible.front();
    collisionsImpossible.pop_front();
    //cout<<planeID<<endl;
    double min = waypointDistance(planeID,-MAX_TURN);
    double minAngle = -MAX_TURN;
    for(double i = -MAX_TURN; i <= MAX_TURN; i+=.1){
      double temp = waypointDistance(planeID,i);
      if(temp <= min){
	min = temp;
	minAngle = i;
      }
    }
    setWaypoint(planeID,minAngle);
  }
}
*/

void findCollisionsAngle(){
  int numPlanesInDanger = (int)collisionsPossible.size();
  while(!collisionsPossible.empty()){
    planesInDanger.clear();
    int planeID = collisionsPossible.front();
    int count = 0;
    planesInDanger.push_back(planeID);
    collisionsPossible.pop_front();
    while(count != (int)planesInDanger.size()){
      for(int i = 0; i < (int)nearby[planeID].size(); i++){
	int temp = nearby[planeID][i];
	if(find(planesInDanger.begin(),planesInDanger.end(),temp) == planesInDanger.end()){
	  planesInDanger.push_back(temp);
	  collisionsPossible.remove(temp);
	}
      }
    count++;
    }
    /*for(int i = 0; i < (int)planesInDanger.size(); i++)
      cout<<planesInDanger[i]<<endl;
      cout<<endl;*/
    simulatedAnnealing(numPlanesInDanger);
  }
}

void simulatedAnnealing(int numPlanesInDanger){
  map<int,double> candidateSolution;
  map<int,double> currentSolution;
  double candidateSolutionValue;
  double currentSolutionValue;
  int sign;
  int temperature = TEMPERATURE;
  int probConst = TEMPERATURE * 2;
  int numVariables = (int)planesInDanger.size();
  int num_iterations_sa = (BASE_ITERATIONS / numPlanesInDanger) * numVariables;
  for(int i = 0; i < numVariables; i++){
    candidateSolution[planesInDanger[i]] = MAX_TURN;
  }
  currentSolution = candidateSolution;
  currentSolutionValue = costFunction(currentSolution);
  while(temperature != 0){
    for(int i = 0; i < num_iterations_sa; i++){
      if(rand() % 2 == 0)
	sign = 1;
      else
	sign = -1;
      int stateChangeNum = rand() % (numVariables + 1);
      double temp = currentSolution[planesInDanger[stateChangeNum]] + sign * .002;
      if((temp > -MAX_TURN && sign == -1 && temp <= MAX_TURN) || (temp >= -MAX_TURN && sign == 1 && temp < MAX_TURN))
	candidateSolution[planesInDanger[stateChangeNum]] = temp;
      candidateSolutionValue = costFunction(candidateSolution);
      double costDifference = currentSolutionValue - candidateSolutionValue;
      if((candidateSolutionValue <= currentSolutionValue) || paccept(costDifference,temperature,probConst)){
	currentSolution = candidateSolution;
	currentSolutionValue = candidateSolutionValue;
      }
    }
    temperature -= 1;
  }
  for(int i = 0; i < numVariables; i++){
    setWaypoint(planesInDanger[i],currentSolution[planesInDanger[i]]);
  }
  //cout<<currentSolutionValue<<endl;
}

bool paccept(double diff, int temperature, int probCost){
  double randomValue = (rand() % probCost + 1);
  double value = temperature * exp(diff);
  return randomValue <= value;
}

void loopCorrect(){
  for(int i = 0; i < NUM_PLANES; i++){
    //angle of actual UAV bearing in degrees w/ respect to cartesian coordinates
    double planeBearing = information[i][PLANE_BEARING];
    double x = information[i][WAYPOINT_X] - information[i][PLANE_X];
    double y = information[i][WAYPOINT_Y] - information[i][PLANE_Y];
    double distanceToTarget = sqrt(x * x + y * y);
    double targetBearing = information[i][WAYPOINT_BEARING];
    double deltaBearing = targetBearing - planeBearing;
    
    //Check to see if attempting a maximum turn
    if (deltaBearing > MAX_TURN || deltaBearing < -MAX_TURN){
      if (distanceToTarget < 24){
	setWaypoint(i,0);
      }
    }
  }
}
