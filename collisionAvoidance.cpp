// Standard C++ Headers
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// Needed ROS headers
#include "ros/ros.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/GoToWaypoint.h"
#include "AU_UAV_ROS/RequestWaypointInfo.h"

using namespace std;

int the_count;
ros::ServiceClient findGoal; // used for requesting waypoint information

//General environment information
const int NUM_PLANES = 32; //number of planes
const int NUM_WAYPOINTS = 50; //number of way points
const int FIELD_SIZE = 500; //size of field
const int VELOCITY = 11.176; //velocity of UAV
const int MAX_TURN = 22.5; //maximum turn radius
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

void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
  the_count++;
  populateInformationTable(msg);
  //print information table if last plane 
  /*if(msg->planeID == NUM_PLANES-1)
    printInformationTable();*/
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

  //initialize counting
  srand(time(NULL));
  the_count = 0;

  //needed for ROS to wait for callbacks
  ros::spin();
  return 0;
}

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
  return (coordComp - FIELD_LONGITUDE)*DEG_TO_MET_LONG;
}

double convertLatitude(double coordComp){
  return (coordComp - FIELD_LATITUDE)*DEG_TO_MET_LAT;
}
