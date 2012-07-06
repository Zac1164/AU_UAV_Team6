#include "ros/ros.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/GoToWaypoint.h"
#include "AU_UAV_ROS/RequestWaypointInfo.h"

#include<omp.h>
#include<iostream>
#include<cmath>
#include<time.h>
#include<vector>
#include<map>
#include<algorithm>

using namespace std;

const int NUM_PLANES = 8;
const int FIELD_SIZE = 500;
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
void loopCorrect();
void setWaypoint(int planeID, double angle);
double projectedPosition_x(int planeID, double angle);
double projectedPosition_y(int planeID, double angle);
void findCollisionsAngle(list<int>& collisionsPossible, vector< vector<int> >& nearby);

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
#pragma omp parallel for
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
    information[ID][WAYPOINT_BEARING] = (msg->targetBearing)*RAD;
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
#pragma omp parallel for
  for(int i = 0; i < NUM_PLANES; i++){
    generateNearby(i,nearby);
    if(!nearby[i].empty())
      collisionsPossible.push_back(i);
    else
      loopCorrect();
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

void loopCorrect(){
  double planeBearing, x, y, distanceToTarget, targetBearing,deltaBearing;
  for(int i = 0; i < NUM_PLANES; i++){
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
    /*for(int i = 0; i < (int)planesInDanger.size(); i++)
      cout<<planesInDanger[i]<<" ";
      cout<<endl;*/
    
  }
}
