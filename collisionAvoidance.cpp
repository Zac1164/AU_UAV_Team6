#include "ros/ros.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/GoToWaypoint.h"
#include "AU_UAV_ROS/RequestWaypointInfo.h"

#include<iostream>
#include<cmath>
#include<time.h>
#include<vector>
#include<map>
#include<algorithm>

using namespace std;

int NUM_PLANES;
double FIELD_SIZE;
const double VELOCITY = 11.176;
const double MAX_TURN = 0.3927;
const double FIELD_LONGITUDE = -85.490363;
const double FIELD_LATITUDE = 32.606573;
const double DEG_TO_MET_LAT = 110897.21;
const double DEG_TO_MET_LONG = 93880.49;

vector< vector<double> > information;
const int PLANE_X = 0;
const int PLANE_Y = 1;
const int PLANE_BEARING = 2;
const int WAYPOINT_X = 3;
const int WAYPOINT_Y = 4;
const int WAYPOINT_BEARING = 5;
const int PREVIOUS_X = 6;
const int PREVIOUS_Y = 7;
const int LOOP_SET = 8;

const double PI = 3.141592;
const double RAD = PI/180;

const double TEMPERATURE = 60;
int num_iterations_sa;

bool okToStart;
double zone;
double zoneCoefficient;
double separation_requirement;
ros::ServiceClient findGoal;
ros::ServiceClient client;

void setupInformationTable();
void setupProperties();
void populateInformationTable(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg, int ID);
double convertLongitude(double coordComp);
double convertLatitude(double coordComp);
void determineNextState();
void generateNearby(int planeID, vector< vector<int> >& nearby);
bool loopCheck(int planeID, int requiredDist);
void setWaypoint(int planeID, double angle);
double projectedPosition_x(int planeID, double angle);
double projectedPosition_y(int planeID, double angle);
void findCollisionsAngle(list<int>& collisionsPossible, vector< vector<int> >& nearby);
void simulatedAnnealing(vector<int>& planesInDanger, int numCollisionsPossible, vector< vector <int> >& nearby);
bool paccept(double diff, int temperature, int probCost);
double costFunction(map<int,double> &solution, vector< vector<int> >& nearby, vector<int>& planesInDanger);
double dangerFactor(int planeID, map<int,double>& solution, vector< vector<int> >& nearby);
double dangerDistance(int planeID, int aggressorID, map<int,double>& solution);
double velocity_x(int planeID, double angle);
double velocity_y(int planeID, double angle);
double waypointDistance(int planeID, double angle);
double waypointDistance(int planeID);
double distanceFromEnemy(int planeID, int aggressorID);

void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
  int ID = msg->planeID;
  populateInformationTable(msg,ID);
  if(ID == NUM_PLANES - 1){
    determineNextState();
    cout<<"ok"<<endl;
    okToStart = true;
  }
}

int main(int argc, char **argv)
{
  setupInformationTable();
  setupProperties();

  ros::init(argc, argv, "collisionAvoidance");
  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);

  findGoal = n.serviceClient<AU_UAV_ROS::RequestWaypointInfo>("request_waypoint_info");

  client = n.serviceClient<AU_UAV_ROS::GoToWaypoint>("go_to_waypoint");

  srand(time(NULL));

  ros::spin();
  return 0;
}

void setupInformationTable(){
  information.resize(NUM_PLANES);
  for(int i = 0; i < NUM_PLANES; i++){
    information[i].resize(9);
    information[i][PLANE_BEARING] = 0;
    information[i][PLANE_X] = 0;
    information[i][PLANE_Y] = 0;
    information[i][LOOP_SET] = 0;
  }
}

void setupProperties(){
  okToStart = false;
  separation_requirement = 12;
  switch(NUM_PLANES){
  case 4:
    num_iterations_sa = 1000;
    if(FIELD_SIZE == 500)
      zoneCoefficient = 1;
    else
      zoneCoefficient = 1.75;
    break;
  case 8:
    num_iterations_sa = 1000;
    if(FIELD_SIZE == 500)
      zoneCoefficient = 1.75;
    else
      zoneCoefficient = 1.5;
    break;
  case 16:
    num_iterations_sa = 500;
    if(FIELD_SIZE == 500)
      zoneCoefficient = 2.0;
    else
      zoneCoefficient = 1.75;
    break;
  case 32:
    if(FIELD_SIZE == 500){
      num_iterations_sa = 250;
      zoneCoefficient = 2.25;
    }
    else{
      num_iterations_sa = 500;
      zoneCoefficient = 2.0;
    }
    break;
  }
  zone = zoneCoefficient * separation_requirement / sin(MAX_TURN);
}

void populateInformationTable(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg, int ID){
  double deltaX, deltaY;
  AU_UAV_ROS::RequestWaypointInfo goalSrv;
  information[ID][PREVIOUS_X] = information[ID][PLANE_X];
  information[ID][PREVIOUS_Y] = information[ID][PLANE_Y];
  information[ID][PLANE_X] = convertLongitude(msg->currentLongitude);
  information[ID][PLANE_Y] = convertLatitude(msg->currentLatitude);
  deltaX = information[ID][PLANE_X] - information[ID][PREVIOUS_X];
  deltaY = information[ID][PLANE_Y] - information[ID][PREVIOUS_Y];
  if(okToStart)
    information[ID][PLANE_BEARING] = atan2(deltaX, deltaY); 
  goalSrv.request.planeID = ID;
  goalSrv.request.isAvoidanceWaypoint = false;
  goalSrv.request.positionInQueue = 0;
  if(findGoal.call(goalSrv)){
    information[ID][WAYPOINT_X] = convertLongitude(goalSrv.response.longitude);
    information[ID][WAYPOINT_Y] = convertLatitude(goalSrv.response.latitude);
    information[ID][WAYPOINT_BEARING] = (msg->targetBearing) * RAD;
  }
}

double convertLongitude(double coordComp){
  return (coordComp - FIELD_LONGITUDE) * DEG_TO_MET_LONG;
}

double convertLatitude(double coordComp){
  return (coordComp - FIELD_LATITUDE) * DEG_TO_MET_LAT;
}

void determineNextState(){
  list<int> collisionsPossible;
  vector< vector<int> > nearby;
  nearby.resize(NUM_PLANES);
  for(int i = 0; i < NUM_PLANES; i++){
    generateNearby(i,nearby);
    if(!nearby[i].empty())
      collisionsPossible.push_back(i);
    else{
      loopSet = information[i][LOOP_SET];
      if(loopCheck(i,36) && loopSet == 0){
	setWaypoint(i,-information[i][PLANE_BEARING]);
	information[i][LOOP_SET]++;
      }
      if(loopCheck(i,36) && loopSet == 1){
	setWaypoint(i,0);
	information[i][LOOP_SET]--;
      }
    }
  }
  findCollisionsAngle(collisionsPossible,nearby);
}

void generateNearby(int planeID, vector< vector<int> >& nearby){
  double tempX = information[planeID][PLANE_X];
  double tempY = information[planeID][PLANE_Y];
  for(int i = 0; i < NUM_PLANES; i++){
    if((planeID != i) && (information[i][PLANE_X] < tempX + zone) && (information[i][PLANE_X] > tempX - zone) && (information[i][PLANE_Y] < tempY + zone) && (information[i][PLANE_Y] > tempY - zone)){
      nearby[planeID].push_back(i);
    }
  }
}

bool loopCheck(int i, int requiredDist){
  double planeBearing, x, y, distanceToTarget, targetBearing,deltaBearing;
  planeBearing = information[i][PLANE_BEARING];
  x = information[i][WAYPOINT_X] - information[i][PLANE_X];
  y = information[i][WAYPOINT_Y] - information[i][PLANE_Y];
  distanceToTarget = sqrt(x * x + y * y);
  targetBearing = information[i][WAYPOINT_BEARING];
  deltaBearing = targetBearing - planeBearing;
  if((deltaBearing > MAX_TURN || deltaBearing < -MAX_TURN) && distanceToTarget < requiredDist){
     return true;
  }
  return false;
}

void setWaypoint(int planeID, double angle){
  AU_UAV_ROS::GoToWaypoint srv;
  srv.request.planeID = planeID;
  srv.request.latitude = projectedPosition_y(planeID,angle) / DEG_TO_MET_LAT + FIELD_LATITUDE;
  srv.request.longitude = projectedPosition_x(planeID,angle) / DEG_TO_MET_LONG + FIELD_LONGITUDE;
  srv.request.isAvoidanceManeuver = true;
  srv.request.isNewQueue = true;
  client.call(srv);
}

double projectedPosition_x(int planeID, double angle){
  return VELOCITY * sin(information[planeID][PLANE_BEARING] + angle) + information[planeID][PLANE_X];
}

double projectedPosition_y(int planeID, double angle){
  return VELOCITY * cos(information[planeID][PLANE_BEARING] + angle) + information[planeID][PLANE_Y];
}

void findCollisionsAngle(list<int>& collisionsPossible, vector< vector<int> >& nearby){
  vector<int> planesInDanger;
  int numCollisionsPossible = (int)collisionsPossible.size();
  while(!collisionsPossible.empty()){
    planesInDanger.clear();
    planesInDanger.push_back(collisionsPossible.front());
    collisionsPossible.pop_front();
    int count = 0;
    while(count < (int)planesInDanger.size()){
      int planeID = planesInDanger[count];
      for(int i = 0; i < (int)nearby[planeID].size(); i++){
	int temp = nearby[planeID][i];
	if(find(planesInDanger.begin(),planesInDanger.end(),temp)==planesInDanger.end()){
	  planesInDanger.push_back(temp);
	  collisionsPossible.remove(temp);
	}
      }
      count++;
    }
    int planeSize = (int)planesInDanger.size();
    //cout<<planeSize<<endl;
    simulatedAnnealing(planesInDanger,numCollisionsPossible, nearby);
  }
}

void simulatedAnnealing(vector<int>& planesInDanger, int numCollisionsPossible, vector< vector<int> >& nearby){
  map<int,double> candidateSolution, currentSolution;
  double candidateSolutionValue, currentSolutionValue;
  int sign;
  int numVariables = (int)planesInDanger.size();
  int temperature = TEMPERATURE;
  int probConst = TEMPERATURE * 2;
  double turnMax = MAX_TURN - 0.0017;
  for(int i = 0; i < numVariables; i++){
    candidateSolution[planesInDanger[i]] = turnMax;
  }
  currentSolution = candidateSolution;
  currentSolutionValue = costFunction(currentSolution,nearby,planesInDanger);
  while(temperature != 0){
    for(int i = 0; i < num_iterations_sa; i++){
      if(rand() % 2 == 0)
	sign = 1;
      else
	sign = -1;
      int stateChangeNum = rand() % numVariables;
      double temp = currentSolution[planesInDanger[stateChangeNum]] + sign * .0017;
      if((sign == -1 && temp >= -turnMax) || (temp <= turnMax && sign == 1))
	candidateSolution[planesInDanger[stateChangeNum]] = temp;
      candidateSolutionValue = costFunction(candidateSolution,nearby,planesInDanger);
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
}

bool paccept(double diff, int temperature, int probCost){
  double randomValue = (rand() % probCost + 1);
  double value = temperature * exp(diff);
  return randomValue <= value;
}

double costFunction(map<int,double> &solution, vector< vector<int> >& nearby, vector<int>& planesInDanger){
  int numPlanesInDanger = (int)planesInDanger.size();
  double value = 0.0;
  for(int i = 0; i < numPlanesInDanger; i++){
    value += dangerFactor(planesInDanger[i],solution,nearby);
  }
  return value;
}

double dangerFactor(int planeID, map<int,double>& solution, vector< vector <int> >& nearby){
  double factor = 0.0;
  for(int i = 0; i < (int)nearby[planeID].size(); i++){
    int aggressorID = nearby[planeID][i];
    factor += 2 / (1 + exp((dangerDistance(planeID,aggressorID,solution) - separation_requirement)));
  }
  return factor;
}

double dangerDistance(int planeID, int aggressorID, map<int,double>& solution){
  double LOS_x, LOS_y, LOS_angle, LOS_mag, relativeVelocityX, relativeVelocityY, relativeVelocityAngle, minDistance, angleDifference, relativeVelocityMag;
  LOS_x = information[aggressorID][PLANE_X] - information[planeID][PLANE_X];
  LOS_y = information[aggressorID][PLANE_Y] - information[planeID][PLANE_Y];
  LOS_angle = atan2(LOS_x, LOS_y);
  LOS_mag = sqrt(LOS_x * LOS_x + LOS_y * LOS_y);
  relativeVelocityX = velocity_x(planeID,solution[planeID]) - velocity_x(aggressorID,solution[aggressorID]);
  relativeVelocityY = velocity_y(planeID,solution[planeID]) - velocity_y(aggressorID,solution[aggressorID]);
  //relativeVelocityMag = sqrt(relativeVelocityX * relativeVelocityX + relativeVelocityY * relativeVelocityY);
  relativeVelocityAngle = atan2(relativeVelocityX, relativeVelocityY);
  angleDifference = abs(relativeVelocityAngle - LOS_angle);
  if(angleDifference < PI / 2.0)
    minDistance = LOS_mag*sin(angleDifference);
  else
    minDistance = LOS_mag;
  return minDistance;
}

double velocity_x(int planeID, double angle){
  return VELOCITY * sin(information[planeID][PLANE_BEARING] + angle);
}

double velocity_y(int planeID, double angle){
  return VELOCITY * cos(information[planeID][PLANE_BEARING] + angle);
}

double waypointDistance(int planeID, double angle){
  double x = information[planeID][WAYPOINT_X] - projectedPosition_x(planeID,angle);
  double y = information[planeID][WAYPOINT_Y] - projectedPosition_y(planeID,angle);
  return sqrt(x * x + y * y);
}

double waypointDistance(int planeID){
  double x = information[planeID][WAYPOINT_X];
  double y = information[planeID][WAYPOINT_Y];
  return sqrt(x * x + y * y);
}

double distanceFromEnemy(int planeID, int aggressorID){
  double x1 = information[planeID][PLANE_X];
  double y1 = information[planeID][PLANE_Y];
  double x2 = information[aggressorID][PLANE_X];
  double y2 = information[aggressorID][PLANE_Y];
  double x = x2 - x1;
  double y = y2 - y1;
  return sqrt(x * x + y * y);
}
