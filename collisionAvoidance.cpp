// Standard C++ Headers
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <limits>

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
const int NUM_PLANES = 32; //number of planes
const int NUM_WAYPOINTS = 50; //number of way points
const int FIELD_SIZE = 500; //size of field
const double VELOCITY = 11.176; //velocity of UAV
const double MAX_TURN = 22.5; //maximum turn radius
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

//Special algorithm variables and constants
const double ZONE = 4*VELOCITY; //distance to search for nearby planes
vector<int> nearby; //stores nearby planes
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
void findMin(int planeID);

void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
  the_count++;
  populateInformationTable(msg);
  //print information table if last plane   
  if(msg->planeID == NUM_PLANES-1){
    for(int i = 0; i < NUM_PLANES; i++){
       findMin(i);
    }
  }
}

int main(int argc, char **argv)
{
  setupInformationTable();
  
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
  information = new double*[NUM_PLANES];
  for(int i = 0; i < NUM_PLANES; i++)
    information[i] = new double[5];
}

void populateInformationTable(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg){
  int ID = msg->planeID;
  information[ID][PLANE_X] = convertLongitude(msg->currentLongitude);
  information[ID][PLANE_Y] = convertLatitude(msg->currentLatitude);
  information[ID][PLANE_BEARING] = msg->targetBearing;
  AU_UAV_ROS::RequestWaypointInfo goalSrv;
  goalSrv.request.planeID = msg->planeID;
  goalSrv.request.isAvoidanceWaypoint = false;
  goalSrv.request.positionInQueue = 0;
  if(findGoal.call(goalSrv)){
    information[ID][WAYPOINT_X] = convertLongitude(goalSrv.response.longitude);
    information[ID][WAYPOINT_Y] = convertLatitude(goalSrv.response.latitude);
  }
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

double angle_i(int planeID, double angle){
  return RAD * (information[planeID][PLANE_BEARING] + angle);
}

double projectedPosition_x(int planeID, double angle){
  return VELOCITY * sin(angle_i(planeID,angle)) + information[planeID][PLANE_X];
}

double projectedPosition_y(int planeID, double angle){
  return VELOCITY * cos(angle_i(planeID,angle)) + information[planeID][PLANE_Y];
}

double waypointDistance(int planeID, double angle){
  double x = information[planeID][WAYPOINT_X] - projectedPosition_x(planeID,angle);
  double y = information[planeID][WAYPOINT_Y] - projectedPosition_y(planeID,angle);
  return sqrt(powNew(x,2) + powNew(y,2));
}

double dangerDistance(int planeID, int aggressorID, double angle){
  double x = information[aggressorID][PLANE_X] - projectedPosition_x(planeID,angle);
  double y = information[aggressorID][PLANE_Y] - projectedPosition_y(planeID,angle);
  return sqrt(powNew(x,2) + powNew(y,2));
}

double dangerFactor(int planeID, double angle){
  generateNearby(planeID);
  int factor = 1;
  for(int i = 0; i < (int)nearby.size(); i++){
    int aggressorID = nearby.back();
    factor += 1 / (1 + exp(dangerDistance(planeID,aggressorID,angle) - 2 * VELOCITY));
    nearby.pop_back();
  }
  return factor;
}

double costFunction(int planeID, double angle){
  return waypointDistance(planeID, angle) * dangerFactor(planeID, angle);
}

void generateNearby(int planeID){
  nearby.clear();
  int tempX = information[planeID][PLANE_X];
  int tempY = information[planeID][PLANE_Y];
  for(int i = 0; i < NUM_PLANES; i++){
    if((i != planeID) && (information[i][PLANE_X] < tempX + ZONE) && (information[i][PLANE_X] > tempX - ZONE) && (information[i][PLANE_Y] < tempY + ZONE) && (information[i][PLANE_Y] > tempY - ZONE))
      nearby.push_back(i);
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
    if(client.call(srv))
    {
      information[planeID][PLANE_Y] = projectedPosition_y(planeID,angle) / DEG_TO_MET_LAT + FIELD_LATITUDE;
      information[planeID][PLANE_X] = projectedPosition_x(planeID,angle) / DEG_TO_MET_LONG + FIELD_LONGITUDE;
      
    }
    else
    {
    ROS_ERROR("Did not receive response");
    }
}

//FUNCTIONS FOR SPECIALIZED MATH

double powNew(double x, int it){
  double y = 1;
  for(int i = 0; i < it; i++)
    y *= x;
  return y;
}


void findMin(int planeID){
  double min = costFunction(planeID,-MAX_TURN);
  double minAngle = -MAX_TURN;
  for(double i = -MAX_TURN; i <= MAX_TURN; i+=.05){
    double temp = costFunction(planeID,i);
    if(temp <= min){
      min = temp;
      minAngle = i;
    }
  }
  setWaypoint(planeID,minAngle);
}

/*
void findMin(int planeID){
  int itMax = 100;
  double eps = numeric_limits<double>::epsilon();
  double a = -MAX_TURN;
  double b = MAX_TURN;
  double c = MAX_TURN;
  double d,e,fc,p,q,r,s,tol1,xm;
  double fa = costFunction(planeID,a);
  double fb = costFunction(planeID,b);
  if((fa > 0.0 && fb > 0.0) || (fa < 0.0 && fb < 0.0)){
    cerr<<"findMin(): Root must be bracketed."<<endl;
    exit(1);
  }
  fc = fb;
  for(int i = 0; i < itMax; i++){
    if((fb > 0.0 && fc > 0.0) || (fb < 0.0 && fc < 0.0)){
      c = a;
      fc = fa;
      d = b-a;
      e =d;
    }
    if(abs(fc) < abs(fb)){
      a = b;
      b = c;
      c = a;
      fa = fb;
      fb = fc;
      fc = fa;
    }
    tol1 = 2.0 * eps * abs(b) + 0.5 * 0.001;
    xm = 0.5 * (c-b);
    if(abs(xm) <= tol1 || fb == 0.0){ 
      setWaypoint(planeID,b);
      return;
    }
    if(abs(e) >= tol1 && abs(fa) > abs(fb)){
      s = fb/fa;
      if(a == c){
	p = 2.0 * xm * s;
	q = 1.0 - s;
      }
      else{
	q = fa / fc;
	r = fb / fc;
	p = s * (2.0 * xm * q * (q-r) - (b-a) * (r -1.0));
	q = (q - 1.0) * (r - 1.0) * (s - 1.0);
      }
      if(p > 0.0)
	q = -q;
      p = abs(p);
      setWaypoint(planeID,b);
      double min1 = 3.0 * xm * q - abs(tol1 * q);
      double min2 = abs(e * q);
      if(2.0 * p < (min1 < min2 ? min1 : min2)){
	e = d;
	d = p / q;
      }
      else{
	d = xm;
	e = d;
      }
    }
    else{
      d = xm;
      e = d;
    }
    a = b;
    fa = fb;
    if(abs(d) > tol1)
      b += d;
    else{
      b += ((xm) >= 0.0 ? fabs(tol1) : -fabs(tol1));
      fb =costFunction(planeID,b);
    }
  }
  cerr<<"findMin(): Max number of iterations exceeded."<<endl;
  exit(1);
}
*/
