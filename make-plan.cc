/**
 * make-plan.cc
 * 
 * Sample code for a robot that has two front bumpers and a laser, and
 * which is provided with localization data.
 *
 * The code also allows the controller to read and write a "plan", a sequence
 * of location that the robot should move to and to read in a "map", a matrix
 * of 1 and 0 values that can be used as an occupancy grid.
 *
 * Written by: Simon Parsons
 * Date:       4th December 2011
 *  
 **/


#include <iostream>
#include <fstream>
#include <vector>
#include <libplayerc++/playerc++.h>
using namespace PlayerCc;  


const int SIZE = 32; // The number of squares per side of the occupancy grid
                     // (which we assume to be square)

player_pose2d_t readPosition(LocalizeProxy& lp);
void printRobotData(BumperProxy& bp, player_pose2d_t pose);
void printLaserData(LaserProxy& sp);

void readMap(int[SIZE][SIZE]);
void printMap(int[SIZE][SIZE]);
void writeMap(int[SIZE][SIZE]);
void dialateMap(int[SIZE][SIZE]);
std::vector<int> findPath(double, double, double, double, int[SIZE][SIZE]);
void findWaypoints(std::vector <int>&, int[SIZE][SIZE]);

int  readPlanLength(void);
void readPlan(double *, int);
void printPlan(double *,int);  
void writePlan(double *, int);


int main(int argc, char *argv[])
{  

  // Variables
  int counter = 0;
  double speed;            // How fast do we want the robot to go forwards?
  double turnrate;         // How fast do we want the robot to turn?
  player_pose2d_t  pose;   // For handling localization data

  // The occupancy grid

  int oGrid[SIZE][SIZE];

  // The set of coordinates that makes up the plan

  int pLength;
  double *plan;

  // Set up proxies. These are the names we will use to connect to 
  // the interface to the robot.
  PlayerClient    robot("localhost");  
  BumperProxy     bp(&robot,0);  
  Position2dProxy pp(&robot,0);
  LocalizeProxy   lp (&robot, 0);
  LaserProxy      sp (&robot, 0);
  
  // Allow the program to take charge of the motors (take care now)
  pp.SetMotorEnable(true);

  // Map handling
  // The occupancy grid is a square array of integers, each side of
  // which is SIZE elements, in which each element is either 1 or 0. A
  // 1 indicates the square is occupied, an 0 indicates that it is
  // free space.
  readMap(oGrid);   // Read a map in from the file map.txt
  printMap(oGrid);  // Print the map on the screen
  //writeMap(oGrid);  // Write a map out to the file map-out.txt
  
  dialateMap(oGrid);
  printMap(oGrid);
  
  std::vector<int> myNodes = findPath(-6,-6,6.5,6.5,oGrid);
  printMap(oGrid);

  findWaypoints(myNodes, oGrid);
  printMap(oGrid);
  
/*
  // Plan handling
  // 
  // A plan is an integer, n, followed by n doubles (n has to be
  // even). The first and second doubles are the initial x and y
  // (respectively) coordinates of the robot, the third and fourth
  // doubles give the first location that the robot should move to, and
  // so on. The last pair of doubles give the point at which the robot
  // should stop.
  pLength = readPlanLength(); // Find out how long the plan is from plan.txt
  plan = new double[pLength]; // Create enough space to store the plan
  readPlan(plan, pLength);    // Read the plan from the file plan.txt.
  printPlan(plan,pLength);    // Print the plan on the screen
  writePlan(plan, pLength);   // Write the plan to the file plan-out.txt


  // Main control loop
  while(true) 
    {    
      // Update information from the robot.
      robot.Read();
      // Read new information about position
      pose = readPosition(lp);
      // Print data on the robot to the terminal
      printRobotData(bp, pose);
      // Print information about the laser. Check the counter first to stop
      // problems on startup
      if(counter > 2){
	printLaserData(sp);
      }

      // Print data on the robot to the terminal --- turned off for now.
      // printRobotData(bp, pose);
      
      // If either bumper is pressed, stop. Otherwise just go forwards

      if(bp[0] || bp[1]){
	speed= 0;
	turnrate= 0;
      } 
      else {
	speed=.1;
        turnrate = 0;
      }     

      // What are we doing?
      std::cout << "Speed: " << speed << std::endl;      
      std::cout << "Turn rate: " << turnrate << std::endl << std::endl;

      // Send the commands to the robot
      pp.SetSpeed(speed, turnrate);  
      // Count how many times we do this
      counter++;
    } */
  
} // end of main()

/**
 * readMap
 *
 * Reads in the contents of the file map.txt into the array map
 * in such a way that the first element of the last row of the
 * file map.txt is in element [0][0].
 *
 * This means that whatever is in the file looks like the occupancy
 * grid would if you drew it on paper.
 *
 **/

void readMap(int map[SIZE][SIZE])
{
	std::ifstream mapFile;
	mapFile.open("map.txt");

	for(int i = SIZE - 1; i >= 0; i--) {
		for(int j = 0; j < SIZE; j++)
			mapFile >> map[i][j];
	}

  mapFile.close();
} // End of readMap()

/**
 * printMap
 *
 * Print map[][] out on the screen. The first element to be printed
 * is [SIZE][0] so that the result looks the same as the contents of
 * map.txt
 *
 **/

void printMap(int map[SIZE][SIZE]) {
	for(int i = SIZE - 1; i >= 0; i--) {
		for(int j = 0; j < SIZE; j++)
			std::cout << map[i][j] << " ";
		std::cout << std::endl;
	}
	std::cout << std::endl << std::endl;
} // End of printMap()

/**
 * writeMap
 *
 * Write a map into map-out.txt in such a way that the [0][0] element
 * ends up in the bottom left corner of the file (so that the contents
 * of the file look like the relevant occupancy grid.
 *
 **/

void writeMap(int map[SIZE][SIZE])
{
  std::ofstream mapFile;
  mapFile.open("map-out.txt");

  for(int i = SIZE - 1; i >= 0; i--){
    for(int j = 0; j < SIZE; j++)
      {
	mapFile << map[i][j];
      }
    mapFile << std::endl;
  }

  mapFile.close();
}



void dialateMap(int map[SIZE][SIZE]) {
	for(int i = SIZE-1; i >= 0; i--) {
		for(int j = 0; j < SIZE; j++) {
			if (i > 0) {
				if (map[i][j] == 0 && map[i-1][j] == 1)
					map[i][j] = 2;
				else if (map[i][j] == 1 && map[i-1][j] == 0)
					map[i-1][j] = 2;
			}
			if (j < SIZE-1) {
				if (map[i][j] == 0 && map[i][j+1] == 1)
					map[i][j] = 2;
				else if (map[i][j] == 1 && map[i][j+1] == 0)
					map[i][j+1] = 2;
			}
		}
	}
}


std::vector<int> findPath(double startX, double startY, double endX, double endY, int map[SIZE][SIZE]) {
	int nodeX = startX*2+16;
	int nodeY = startY*2+16;
	std::vector<int> closedNodes(1, nodeX*100+nodeY);
	
	endX = endX*2+16;
	endY = endY*2+16;
	
	int minX, minY, minCost = 999, nodeCost = 0;
	map[nodeX][nodeY] = 3;
	
	while (nodeX != endX || nodeY != endY) {
		for (int i = nodeX+1; i > nodeX-2; i--) {
			for (int j = nodeY-1; j < nodeY+2; j++) {
				if (map[i][j] == 0) {
					nodeCost = 1 + std::abs(endX-i) + std::abs(endY-j);
					if (nodeCost < minCost) {
						minCost = nodeCost;
						minX = i;
						minY = j;
					}
				}
			}
		}
		nodeX = minX;
		nodeY = minY;
		map[nodeX][nodeY] = 3;
		closedNodes.push_back(nodeX*100+nodeY);
		minCost = 9999;
	}
	for (int i = 0; i < closedNodes.size(); i++)
		std::cout << closedNodes[i] << std::endl;
	return closedNodes;
}


void findWaypoints (std::vector<int>& myNodes, int map[SIZE][SIZE]) {
	int lastWaypoint = 0, i = 1;
	
	while (i < myNodes.size()-1) {
		std::cout << "\n" << myNodes[i] << " " << i << " " 
				  << myNodes[i-1]/100-myNodes[i]/100 << " " << myNodes[i]/100-myNodes[i+1]/100 << " "
				  << myNodes[i-1]%100-myNodes[i]%100 << " " << myNodes[i]%100-myNodes[i+1]%100 << " ";
		if ((myNodes[i-1]/100-myNodes[i]/100 == myNodes[i]/100-myNodes[i+1]/100) &&
			(myNodes[i-1]%100-myNodes[i]%100 == myNodes[i]%100-myNodes[i+1]%100)) {
			std::cout << "Delete";
			i++;
		}
		else {
			if (i-lastWaypoint > 0) {
				std::cout << myNodes[lastWaypoint+1] << " " << myNodes[i-1];
				myNodes.erase(myNodes.begin()+lastWaypoint+1, myNodes.begin()+i);
			}
			lastWaypoint++;
			i = lastWaypoint+1;
		}	
	}
	myNodes.erase(myNodes.begin()+lastWaypoint+1, myNodes.end()-1);
	std::cout << std::endl << std::endl;
	for (i = 0; i < myNodes.size(); i++) {
		map[myNodes[i]/100][myNodes[i]%100] = 4;
	}
}



/**
 * readPosition()
 *
 * Read the position of the robot from the localization proxy. 
 *
 * The localization proxy gives us a hypothesis, and from that we extract
 * the mean, which is a pose. 
 *
 **/

player_pose2d_t readPosition(LocalizeProxy& lp)
{

  player_localize_hypoth_t hypothesis;
  player_pose2d_t          pose;
  uint32_t                 hCount;

  // Need some messing around to avoid a crash when the proxy is
  // starting up.

  hCount = lp.GetHypothCount();

  if(hCount > 0){
    hypothesis = lp.GetHypoth(0);
    pose       = hypothesis.mean;
  }

  return pose;
} // End of readPosition()


void printLaserData(LaserProxy& sp)
{

  double maxRange, minLeft, minRight, range, bearing;
  int points;

  maxRange  = sp.GetMaxRange();
  minLeft   = sp.MinLeft();
  minRight  = sp.MinRight();
  range     = sp.GetRange(5);
  bearing   = sp.GetBearing(5);
  points    = sp.GetCount();

  //Uncomment this to print out useful laser data
  //std::cout << "Laser says..." << std::endl;
  //std::cout << "Maximum distance I can see: " << maxRange << std::endl;
  //std::cout << "Number of readings I return: " << points << std::endl;
  //std::cout << "Closest thing on left: " << minLeft << std::endl;
  //std::cout << "Closest thing on right: " << minRight << std::endl;
  //std::cout << "Range of a single point: " << range << std::endl;
  //std::cout << "Bearing of a single point: " << bearing << std::endl;

  return;
} // End of printLaserData()

/**
 *  printRobotData
 *
 * Print out data on the state of the bumpers and the current location
 * of the robot.
 *
 **/

void printRobotData(BumperProxy& bp, player_pose2d_t pose)
{

  // Print out what the bumpers tell us:
  std::cout << "Left  bumper: " << bp[0] << std::endl;
  std::cout << "Right bumper: " << bp[1] << std::endl;
  // Can also print the bumpers with:
  //std::cout << bp << std::endl;

  // Print out where we are
  std::cout << "We are at" << std::endl;
  std::cout << "X: " << pose.px << std::endl;
  std::cout << "Y: " << pose.py << std::endl;
  std::cout << "A: " << pose.pa << std::endl;

  
} // End of printRobotData()

/**
 * readPlanLength
 *
 * Open the file plan.txt and read the first element, which should be
 * an even integer, and return it.
 *
 **/

int readPlanLength(void)
{
  int length;

  std::ifstream planFile;
  planFile.open("plan.txt");

  planFile >> length;
  planFile.close();

  // Some minimal error checking
  if((length % 2) != 0){
    std::cout << "The plan has mismatched x and y coordinates" << std::endl;
    exit(1);
  }

  return length;

} // End of readPlanLength

/**
 * readPlan
 *
 * Given the number of coordinates, read them in from plan.txt and put
 * them in the array plan.
 *
 **/

void readPlan(double *plan, int length)
{
  int skip;

  std::ifstream planFile;
  planFile.open("plan.txt");

  planFile >> skip;
  for(int i = 0; i < length; i++){
    planFile >> plan[i];
  }

  planFile.close();

} // End of readPlan

/**
 * printPlan
 *
 * Print the plan on the screen, two coordinates to a line, x then y
 * with a header to remind us which is which.
 *
 **/

void printPlan(double *plan , int length)
{
  std::cout << std::endl;
  std::cout << "   x     y" << std::endl;
  for(int i = 0; i < length; i++){
    std::cout.width(5);
    std::cout << plan[i] << " ";
    if((i > 0) && ((i % 2) != 0)){
      std::cout << std::endl;
    }
  }
  std::cout << std::endl;

} // End of printPlan


/**
 * writePlan
 * 
 * Send the plan to the file plan-out.txt, preceeded by the information
 * about how long it is.
 *
 **/

void writePlan(double *plan , int length)
{
  std::ofstream planFile;
  planFile.open("plan-out.txt");

  planFile << length << " ";
  for(int i = 0; i < length; i++){
    planFile << plan[i] << " ";
  }

  planFile.close();

} // End of writePlan
