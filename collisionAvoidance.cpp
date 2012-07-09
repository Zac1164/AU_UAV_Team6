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

const int NUM_PLANES = 32;
const double FIELD_SIZE = 500;
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

const double PI = 3.141592;
const double RAD = PI/180;

double TEMPERATURE;
double BASE_ITERATIONS;
const int NORM_CONSTANT = 1/(FIELD_SIZE * 4  * sqrt(2));

bool okToStart;
double zone;
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
void loopCorrect(int planeID);
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
    information[i].resize(8);
    information[i][PLANE_BEARING] = 0;
    information[i][PLANE_X] = 0;
    information[i][PLANE_Y] = 0;
  }
}

void setupProperties(){
  okToStart = false;
  switch(NUM_PLANES){
  case 4:
    if(FIELD_SIZE == 500){
      zone = 60;
      separation_requirement = 24;
      BASE_ITERATIONS = 4250;
      TEMPERATURE = 40;
    }
    else{
      zone = 60;
      separation_requirement = 24;
      BASE_ITERATIONS = 4250;
      TEMPERATURE = 40;
    }
    break;
  case 8:
    if(FIELD_SIZE == 500){
      /*zone = 72;
      separation_requirement = 30;
      BASE_ITERATIONS = 3500;
      TEMPERATURE = 40;*/
      zone = 100;
      separation_requirement = 40;
      BASE_ITERATIONS = 3500;
      TEMPERATURE = 40;
    }
    else{
      zone = 72;
      separation_requirement = 30;
      BASE_ITERATIONS = 4000;
      TEMPERATURE = 40;
    }
    break;
  case 16:
    if(FIELD_SIZE == 500){
      zone = 72;
      separation_requirement = 12;
      BASE_ITERATIONS = 3000;
      TEMPERATURE = 75;
    }
    else{
      zone = 5*VELOCITY;
      separation_requirement = 24;
      BASE_ITERATIONS = 3000;
      TEMPERATURE = 40;
    }
    break;
  case 32:
    if(FIELD_SIZE == 500){
      separation_requirement = 18;
      zone = 1.25 * separation_requirement / sin(MAX_TURN);
      TEMPERATURE = 40;
    }
    else{
      zone = 5*VELOCITY;
      separation_requirement = 24;
      BASE_ITERATIONS = 2000;
      TEMPERATURE = 40;
    }
    break;
  }
}

void populateInformationTable(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg, int ID){
  int deltaX, deltaY;
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
    else
      loopCorrect(i);
  }
  /*std::list<int>::iterator pos;
  for (pos=collisionsPossible.begin(); pos!=collisionsPossible.end(); ++pos) {
    cout<<*pos<<endl;
    }*/
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

void loopCorrect(int i){
  double planeBearing, x, y, distanceToTarget, targetBearing,deltaBearing;
  //angle of actual UAV bearing in degrees w/ respect to cartesian coordinates
  planeBearing = information[i][PLANE_BEARING];
  x = information[i][WAYPOINT_X] - information[i][PLANE_X];
  y = information[i][WAYPOINT_Y] - information[i][PLANE_Y];
  distanceToTarget = sqrt(x * x + y * y);
  targetBearing = information[i][WAYPOINT_BEARING];
  deltaBearing = targetBearing - planeBearing;
  //Check to see if attempting a maximum turn
  if (deltaBearing > MAX_TURN || deltaBearing < -MAX_TURN){
    if (distanceToTarget < 24){
      setWaypoint(i,0);
    }
  }
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
    /*std::list<int>::iterator pos;
    for (pos=collisionsPossible.begin(); pos!=collisionsPossible.end(); ++pos) {
    cout<<*pos<<" ";
    }
    cout<<endl;*/
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
    //cout<<(int)planesInDanger.size()<<endl;
    /*for(int i = 0; i < (int)planesInDanger.size(); i++)
      cout<<planesInDanger[i]<<" ";
      cout<<endl;*/
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
  int num_iterations_sa = 500;
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
      if((temp <= turnMax && sign == -1) && (temp >= -turnMax && sign == 1))
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
  //cout<<currentSolutionValue<<endl;
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
  // double standardizingConstant = 1/FIELD_SIZE;
  for(int i = 0; i < numPlanesInDanger; i++){
    value += dangerFactor(planesInDanger[i],solution,nearby);// + waypointDistance(i,solution[i]) * standardizingConstant;
  }
  //cout<<value<<endl;
  /*if(numPlanesInDanger == 2){
    value += waypointDistance(0,solution[0]) + waypointDistance(1,solution[1]);
    }*/
  return value;
}

double dangerFactor(int planeID, map<int,double>& solution, vector< vector <int> >& nearby){
  double factor = 0.0;
  for(int i = 0; i < (int)nearby[planeID].size(); i++){
    int aggressorID = nearby[planeID][i];
    factor += 2 / (1 + exp((dangerDistance(planeID,aggressorID,solution) - separation_requirement)));
    /*double dangerLevel = dangerDistance(planeID,aggressorID,solution);
    double standardizingConstant = 1/FIELD_SIZE;
    if(dangerLevel <= separation_requirement)
      factor += 2 / (1 + exp(dangerLevel - separation_requirement));
    else
      factor += waypointDistance(i,solution[i]) * standardizingConstant;
    */
  }
  /*factor += 1 / (1 + exp(abs(solution[planeID] - information[planeID][WAYPOINT_BEARING])));
  if(solution[planeID] > 0.4)
  cout<<solution[planeID]<<endl;*/
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
  relativeVelocityMag = sqrt(relativeVelocityX * relativeVelocityX + relativeVelocityY * relativeVelocityY);
  relativeVelocityAngle = atan2(relativeVelocityX, relativeVelocityY);
  angleDifference = abs(relativeVelocityAngle - LOS_angle);
  if(angleDifference < PI / 2.0){
    if(LOS_mag - relativeVelocityMag * cos(angleDifference) <= separation_requirement){
      minDistance = 0;
      //cout<<"crash"<<endl;
    }
    else
      minDistance = LOS_mag*sin(angleDifference);
  }
  else{
    minDistance = LOS_mag;
  }
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
