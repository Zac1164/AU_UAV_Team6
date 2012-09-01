/*
Authors: Zachary Daniels and Lacey Wright
Title: Collision Avoidance of Multiple UAS Using a Collision Cone-Based Cost Function
Purpose: Heuristic for collision avoidance of multiple UAS using collision cone and simulated annealing
Date of Creation: June to August 2012
Funded by: NSF Grant #0851960 and the United States Department of Defense for the 2012 NSF REU on Smart Unmanned Aerial Vehicles at Auburn University
Contributions and Advisement by James Holt, Saad Biaz, and Gilbert Crouse
*/

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

/*Course Information*/
int num_planes; //Number of UAS in run
double field_size; //Width (or height) of field in meters
const double VELOCITY = 11.176; //Velocity of UAS in m/s
const double MAX_TURN = 0.3927; //Maximum turn angle of UAS in radians
const double FIELD_LONGITUDE = -85.490363; //Longitude of western edge of field
const double FIELD_LATITUDE = 32.606573; //Latitude of northern edge of field
const double DEG_TO_MET_LAT = 110897.21; //Conversion constant for degrees latitude to meters
const double DEG_TO_MET_LONG = 93880.49; //Conversion constant for degrees longitude to meters

/*Plane Information*/
vector< vector<double> > information; //Vector stores information about each UAS
const int PLANE_X = 0; //X-position of UAS in meters with respect to origin at Northwest corner
const int PLANE_Y = 1; //Y-position of UAS in meters with respect to orgiin at Northwest corner
const int PLANE_BEARING = 2; //Bearing of UAS from North
const int WAYPOINT_X = 3; //X-position of current goal waypoint
const int WAYPOINT_Y = 4; //Y-position of current goal waypoint
const int WAYPOINT_BEARING = 5; //Bearing from North to current goal waypoint
const int PREVIOUS_X = 6; //Former x-position
const int PREVIOUS_Y = 7; //Former y-position
const int LOOP_SET = 8; //Used for correcting looping
vector<bool> okToStart; //Stores whether or not it ok to start populating certain information for each UAS 

/*Mathematical Constants*/
const double PI = 3.141592; //Constant to for pi
const double RAD = PI/180; //Constant for a single radian

/*Algorithm Parameters*/
double temperature; //Temperature to be used with simulated annealing
int num_iterations_sa; //Number of iterations to be used with simulated annealing
double zone; //Danger zone requirement in meters
double zoneCoefficient; //Expansion coefficient for danger zone calculations
const double separation_requirement = 12; //Separation requirement for collision cone

/*ROS Services*/
ros::ServiceClient findGoal; //Get waypoint information
ros::ServiceClient client; //Go to waypoint service

/* Purpose: Set up the okToStart vector
   Pre-conditions: None
   Post-conditions: okToStart is resized to 32 and every value is set to false
   Returns: Nothing
*/
void setupTable();

/* Purpose: Set up the course properties and algorithm parameters depending plane count and field size 
   Pre-conditions: num_planes is set and field_size is set
   Post-conditions: num_iterations, zoneCoefficient, temperature, and zone may be adjusted
   Returns: Nothing
*/
void setupProperties();

/* Purpose: Populate the UAS information table
   Pre-conditions: 
   * Must occur during telemetry update
   * ID represents the UAS ID and is passed as an int
   Post-conditions: information vector is filled in with UAS and waypoint information for the upcoming time step
   Returns: Nothing
*/
void populateInformationTable(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg, int ID);

/* Purpose: Convert from degrees longitude to an x-coordinate in meters with origin at the Northwest field corner
   Pre-conditions: coordComp is a longitude component of a coordinate passed by value
   Post-conditions: No data is changed
   Returns: A double representing an x-component of position in meters with origin at the Northwest field corner is returned
*/
double convertLongitude(double coordComp);

/* Purpose: Convert from degrees latitude to a y-coordinate in meters with origin at the Northwest field corner
   Pre-conditions: coordComp is a latitude component of a coordinate and is passed by value
   Post-conditions: No data is changed
   Returns: A double representing a y-component of position in meters with origin at the Northwest field corner is returned
*/
double convertLatitude(double coordComp);

/* Purpose: Determine the actions to be taken by each UAS before the next time step
   Pre-conditions:
   * num_planes stores the number of UAS on the field
   * information[][LOOP_SET] for each UAS is set to either 0 or 1 
   Post-conditions:
   * collisionsPossible is created. collisionsPossible stores a list of UAS that fall within some other UAS' danger zone
   * nearby is created. nearby is a two-dimensional vector that stores a vector of all UAS within a UAS' danger zone for each UAS
   * information[][LOOP_SET] may change if a UAS is determined to be in a loop
   Returns: Nothing
*/
void determineNextState();

/* Purpose: Generate a vector of all UAS within a certain UAS' danger zone
   Pre-conditions:
   * planeID holds the ID of the UAS whose danger zone is being checked and is passed by value
   * nearby is a two-dimensional vector which stores vectors for each UAS of the other UAS in its danger zone and is passed by reference
   * information is populated
   Post-conditions: The vector of all UAS within a certain UAS' danger zone is stored in nearby[planeID]
   Returns: Nothing
*/
void generateNearby(int planeID, vector< vector<int> >& nearby);

/* Purpose: Check whether or not a UAS is looping around its waypoint
   Pre-conditions: 
   * planeID stores the ID of the UAS that is being checked for looping and is passed by value
   * requiredDistance stores the distance a UAS must be away from a waypoint for looping to occur and is passed by value
   * information is populated
   Post-conditions: No data is changed
   Returns: true is returned if the UAS is looping; otherwise, false is returned
*/
bool loopCheck(int planeID, int requiredDist);

/* Purpose: Sets an avoidance waypoint of a UAS
   Pre-conditions: 
   * planeID stores the ID of the UAS whose avoidance waypoint is being set and is passed by value
   * angle stores the angle of turn to adjust the UAS by and is passed by value
   * client has been initialized
   Post-conditions: No data is changed
   Returns: Nothing
*/
void setWaypoint(int planeID, double angle);

/* Purpose: Gives the x-coordinate of the next position of the UAS
   Pre-conditions:
   * planeID is the ID of the plane whose next position is being calculated
   * angle stores the angle of turn to adjust the UAS by and is passed by value
   Post-conditions: No data is changed
   Returns: x-coordinate of next position of UAS as double
*/
double projectedPosition_x(int planeID, double angle);

/* Purpose: Gives the y-coordinate of the next position of the UAS
   Pre-conditions:
   * planeID is the ID of the plane whose next position is being calculated
   * angle stores the angle of turn to adjust the UAS by and is passed by value
   Post-conditions: No data is changed
   Returns: y-coordinate of next position of UAS as double
*/
double projectedPosition_y(int planeID, double angle);

/* Purpose: Finds 'best' angles of all planes at risk of colliding soon
   Pre-conditions:
   * collisionsPossible is passed by reference and stores a list of all UAS at risk with colliding with other UAS.
   * nearby is a vector of vectors passed by reference that stores information on which UAS are near other UAS
   Post-conditions:
   * collisionsPossible is emptyed
   * simulatedAnnealing() is called
   Returns: Nothing
*/
void findCollisionsAngle(list<int>& collisionsPossible, vector< vector<int> >& nearby);

/* Purpose: Use simulated annealing to estimate optimal angles of turn for UAs in danger
   Pre-conditions:
   * planesInDanger is a vector passed by reference that stores which UAS are in danger of collision
   * numCollisionsPossible stores the number of collisions possible and is passed by value
   * nearby is a vector of vectors passed by reference that stores information on which UAS are near other UAS
   Post-conditions:
   * the angle of turn for each plane in danger is adjusted
   Returns: Nothing
*/
void simulatedAnnealing(vector<int>& planesInDanger, int numCollisionsPossible, vector< vector <int> >& nearby);

/* Purpose: Acceptance function for simulated annealing
   Pre-conditions:
   * diff is the difference between a current solution and candidate solution passed as a double by value
   * temp_current is the current temperature used in simulated annealing passed as an int by value
   * probCost is used to adjust the probability of accepting a candidate solution and is passed as an int by value
   * temperature must be set
   * num_iterations_sa must be set
   Post-conditions: No data is changed
   Returns: A bool that if true, accepts the candidate solution, or if false, rejects the solution
*/
bool paccept(double diff, int temp_current, int probCost);

/* Purpose: Cost function to be used with simulated annealing
   Pre-conditions:
   * solution is a map of the candidate solution with "UAS ID", "next angle" pairs passed by reference
   * nearby is a vector of vectors passed by reference that stores information on which UAS are near other UAS
   * planesInDanger is a vector passed by reference that stores which UAS are in danger of collision
   Post-conditions: No data changes
   Returns: The value of the cost function of the candidate solution as a double
*/
double costFunction(map<int,double> &solution, vector< vector<int> >& nearby, vector<int>& planesInDanger);

/*
  Purpose: Calculate the danger factor for a given UAS
  Pre-conditions:
  * planeID is ID of the UAS of focus and passed by value
  * solution is a map of the candidate solution with "UAS ID", "next angle" pairs passed by reference
  * nearby is a vector of vectors passed by reference that stores information on which UAS are near other UAS
  * separation_requirement must be set
  Post-conditions: No data is changed
  Returns: the danger factor as a double
*/
double dangerFactor(int planeID, map<int,double>& solution, vector< vector<int> >& nearby);

/*
  Purpose: Returns the distance of minimum separation between a UAS and an aggressor UAS
  Pre-conditions:
  * planeID is the ID of the UAS of focus and passed by value
  * aggressorID is the ID of the aggressor UAS
  * solution is a map of the candidate solution with "UAS ID", "next angle" pairs passed by reference
  Post-conditions: No data is changed
  Returns: The minimum distance of separation between a UAS and an aggressor UAS as a double
*/
double dangerDistance(int planeID, int aggressorID, map<int,double>& solution);

/*
  Purpose: Find the x-component of a velocity vector given a new angle of turn
  Pre-conditions:
  * planeID is the ID of the UAS of focus and passed by value
  * angle stores the angle of turn to adjust the UAS by and is passed by value
  Post-conditions: No data is changed
  Returns: x-component of a velocity vector given a new angle of turn as a double
*/
double velocity_x(int planeID, double angle);

/*
  Purpose: Find the y-component of a velocity vector given a new angle of turn
  Pre-conditions:
  * planeID is the ID of the UAS of focus and passed by value
  * angle stores the angle of turn to adjust the UAS by and is passed by value
  Post-conditions: No data is changed
  Returns: y-component of a velocity vector given a new angle of turn as a double
*/
double velocity_y(int planeID, double angle);

/*
  Purpose: Calculates the distance away from a waypoint given a projected position of a UAS
  Pre-conditions:
  * planeID is the ID of the UAS of focus and passed by value
  * angle stores the angle of turn to adjust the UAS by and is passed by value
  Post-conditions: No data is changed
  Returns: distance away from a waypoint given a projected position of a UAS as a double
*/
double waypointDistance(int planeID, double angle);

/*
  Purpose: Calculates the distance away from a waypoint of a UAS
  Pre-conditions:
  * planeID is the ID of the UAS of focus and passed by value
  Post-conditions: No data is changed
  Returns: distance away from a waypoint of a UAS as a double
*/
double waypointDistance(int planeID);

/*
  Purpose: Calculates the distance away of a UAS from an aggressor UAS
  Pre-conditions:
  * planeID is the ID of the UAS of focus and passed by value
  * aggressorID is the ID of the aggressor UAS and passed by value
  Post-conditions: No data is changed
  Returns: distance away of a UAS from an aggressor UAS as a double
*/
double distanceFromEnemy(int planeID, int aggressorID);

void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
  if(msg->currentWaypointIndex != -1){
    int ID = msg->planeID;
    populateInformationTable(msg,ID);
    if(ID == 5 && num_planes == 4)
      num_planes = 8;
    else if(ID == 9 && num_planes == 8)
      num_planes = 16;
    else if(ID == 17 && num_planes == 16)
      num_planes = 32;
    if(ID == num_planes - 1){
      determineNextState();
      cout<<"ok"<<endl;
    }
  }
}

int main(int argc, char **argv)
{
  num_planes = 4;
  field_size = 500;
  temperature = 60;

  setupTable();

  ros::init(argc, argv, "collisionAvoidance");
  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);

  findGoal = n.serviceClient<AU_UAV_ROS::RequestWaypointInfo>("request_waypoint_info");

  client = n.serviceClient<AU_UAV_ROS::GoToWaypoint>("go_to_waypoint");

  srand(time(NULL));

  ros::spin();
  return 0;
}

void setupTable(){
  okToStart.resize(32);
  for(int i = 0; i < 32; i++)
    okToStart[i] = false;
}

void setupProperties(){
  switch(num_planes){
  case 4:
    num_iterations_sa = 1000;
    if(field_size == 500)
      zoneCoefficient = 1.75;
    else
      zoneCoefficient = 1.75;
    break;
  case 8:
    num_iterations_sa = 1000;
    if(field_size == 500)
      zoneCoefficient = 2.25;
    else
      zoneCoefficient = 1.75;
    break;
  case 16:
    num_iterations_sa = 500;
    if(field_size == 500)
      zoneCoefficient = 2.25;
    else
      zoneCoefficient = 2.25;
    break;
  case 32:
    if(field_size == 500){
      num_iterations_sa = 250;
      zoneCoefficient = 1.00;
      temperature = 50;
    }
    else{
      num_iterations_sa = 500;
      zoneCoefficient = 2.25;
    }
    break;
  }
  zone = zoneCoefficient * separation_requirement / sin(MAX_TURN);
}

void populateInformationTable(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg, int ID){
  double deltaX, deltaY;
  AU_UAV_ROS::RequestWaypointInfo goalSrv;
  if(!okToStart[ID]){
    information.resize((int)information.size()+1);
    information[ID].resize(9);
    information[ID][PLANE_BEARING] = 0;
    information[ID][PLANE_X] = 0;
    information[ID][PLANE_Y] = 0;
    information[ID][LOOP_SET] = 0;
  }
  information[ID][PREVIOUS_X] = information[ID][PLANE_X];
  information[ID][PREVIOUS_Y] = information[ID][PLANE_Y];
  information[ID][PLANE_X] = convertLongitude(msg->currentLongitude);
  information[ID][PLANE_Y] = convertLatitude(msg->currentLatitude);
  deltaX = information[ID][PLANE_X] - information[ID][PREVIOUS_X];
  deltaY = information[ID][PLANE_Y] - information[ID][PREVIOUS_Y];
  if(okToStart[ID])
    information[ID][PLANE_BEARING] = atan2(deltaX, deltaY);
  goalSrv.request.planeID = ID;
  goalSrv.request.isAvoidanceWaypoint = false;
  goalSrv.request.positionInQueue = 0;
  if(findGoal.call(goalSrv)){
    information[ID][WAYPOINT_X] = convertLongitude(goalSrv.response.longitude);
    if(information[ID][WAYPOINT_X] > 500)
      field_size = 1000;
    information[ID][WAYPOINT_Y] = convertLatitude(goalSrv.response.latitude);
    if(information[ID][WAYPOINT_X] > 500)
      field_size = 1000;
    information[ID][WAYPOINT_BEARING] = (msg->targetBearing) * RAD;
  }
  okToStart[ID] = true;
}

double convertLongitude(double coordComp){
  return (coordComp - FIELD_LONGITUDE) * DEG_TO_MET_LONG;
}

double convertLatitude(double coordComp){
  return (coordComp - FIELD_LATITUDE) * DEG_TO_MET_LAT;
}

void determineNextState(){
  setupProperties();
  list<int> collisionsPossible;
  vector< vector<int> > nearby;
  nearby.resize(num_planes);
  for(int i = 0; i < num_planes; i++){
    generateNearby(i,nearby);
    if(!nearby[i].empty())
      collisionsPossible.push_back(i);
    else{
      int loopSet = information[i][LOOP_SET];
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
  for(int i = 0; i < num_planes; i++){
    if((planeID != i) && (information[i][PLANE_X] < tempX + zone) && (information[i][PLANE_X] > tempX - zone) && (information[i][PLANE_Y] < tempY + zone) && (information[i][PLANE_Y] > tempY - zone)){
      nearby[planeID].push_back(i);
    }
  }
}

bool loopCheck(int planeID, int requiredDist){
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
    simulatedAnnealing(planesInDanger,numCollisionsPossible, nearby);
  }
}

void simulatedAnnealing(vector<int>& planesInDanger, int numCollisionsPossible, vector< vector<int> >& nearby){
  map<int,double> candidateSolution, currentSolution;
  double candidateSolutionValue, currentSolutionValue;
  int sign;
  int numVariables = (int)planesInDanger.size();
  int temp_current = temperature;
  int probConst = temperature * 2;
  double turnMax = MAX_TURN - 0.0017;
  for(int i = 0; i < numVariables; i++){
    candidateSolution[planesInDanger[i]] = turnMax;
  }
  currentSolution = candidateSolution;
  currentSolutionValue = costFunction(currentSolution,nearby,planesInDanger);
  while(temp_current != 0){
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
      if((candidateSolutionValue <= currentSolutionValue) || paccept(costDifference,temp_current,probConst)){
	currentSolution = candidateSolution;
	currentSolutionValue = candidateSolutionValue;
      }
    }
    temp_current -= 1;
  }
  for(int i = 0; i < numVariables; i++){
    setWaypoint(planesInDanger[i],currentSolution[planesInDanger[i]]);
  }
}

bool paccept(double diff, int temp_current, int probCost){
  double randomValue = (rand() % probCost + 1);
  double value = temp_current * exp(diff);
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
