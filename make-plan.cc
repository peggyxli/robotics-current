/**
 * Robotics class project #6
 * Modified version of make-plan.cc, originally written by Simon Parsons
 * Creates a path for a Roomba/Create to navigate around obstacles to a specified goal location
 * Reads in and uses a premade occupancy grid (map.txt)
 * Includes navigation code for the Roomba/Create to move along the path
 * Group 5: Randolph Cisneros, Arsenii Lyzenko, Peggy Li
 **/


#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <libplayerc++/playerc++.h>
using namespace PlayerCc;  

const int SIZE = 32; // The number of squares per side of the occupancy grid


struct Node {
	int y;
	int x;
	double distanceFromStart;
	double cost;
	Node* parent;
	Node(): y(0), x(0), distanceFromStart(0), cost(0), parent(NULL) {};
	Node(int a, int b, int endY, int endX, Node* myPointer = NULL): y(a), x(b), parent(myPointer) {
		if (parent != NULL) {	//add distance from parent
			distanceFromStart = parent->distanceFromStart;
			distanceFromStart += sqrt((parent->y-y)*(parent->y-y)+(parent->x-x)*(parent->x-x));	
		}
		else distanceFromStart = 0;
		
		//add distance to goal
		cost = distanceFromStart + sqrt((endY-y)*(endY-y)+(endX-x)*(endX-x));
	}
};

bool operator< (const Node& node1, const Node& node2) {return node1.cost > node2.cost;}
bool operator> (const Node& node1, const Node& node2) {return node1.cost < node2.cost;}					 


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
void writePlan(std::vector<int>);


int main(int argc, char *argv[]) {
	//Set up proxies to connect the interface to the robot
	player_pose2d_t pose;
	PlayerClient robot("localhost");  
	BumperProxy bp(&robot, 0);  
	Position2dProxy pp(&robot, 0);
	LocalizeProxy lp (&robot, 0);
	LaserProxy sp (&robot, 0);
	
	pp.SetMotorEnable(true);
	robot.Read();
	pose = readPosition(lp);
	
	int oGrid[SIZE][SIZE];	//the occupancy grid
	int pLength;
	double *plan;	

	readMap(oGrid);   // Read a map in from the file map.txt
	dialateMap(oGrid);  
	std::vector<int> myNodes = findPath(pose.px,pose.py,6.5,6.5,oGrid);
	findWaypoints(myNodes, oGrid);
	printMap(oGrid);
	writeMap(oGrid);
	writePlan(myNodes);   // Write the plan to the file plan-out.txt

	pLength = readPlanLength(); // Find out how long the plan is from plan.txt
	plan = new double[pLength]; // Create enough space to store the plan
	readPlan(plan, pLength);    // Read the plan from the file plan.txt.
	printPlan(plan,pLength);    // Print the plan on the screen
	
	
	std::cout << "Booting up laser." << std::endl;
	for (int i = 0; i < 3; i++) 
		robot.Read();	//to avoid seg fault while booting up
	double speed, turnrate, diffY, diffX, diffAngle;
	
	
	for (int i = 0; i < pLength; i = i + 2) {	//for each pair of coordinates
		//navigate to waypoint
		while (std::abs(pose.px - plan[i]) > 0.01 || std::abs(pose.py - plan[i + 1]) > 0.01) {
			//update and print information from the robot
			robot.Read();
			pose = readPosition(lp);
			printRobotData(bp, pose);
			
			if (sp.MinLeft() < .5) { //obstacle avoidance
				turnrate = sp.MinLeft() - 2;
				if (bp[0] || bp[1])
					speed = -sp.MinLeft();
				else speed = sp.MinLeft()/2;
				std::cout << "Obstacle avoidance in progress." << std::endl;
			}
			else if (sp.MinRight() < .5) {	//obstacle avoidance
				turnrate = 2 - sp.MinRight();
				if (bp[0] || bp[1])
					speed = -sp.MinRight();
				else speed = sp.MinRight()/2;
				std::cout << "Obstacle avoidance in progress." << std::endl;
			}
			else {	//locate and move towards position
				//calculate angle needed to move to end position
				diffY = plan[i + 1] - pose.py;
				diffX = plan[i] - pose.px;
				diffAngle = atan2(diffY, diffX) - pose.pa;
				
				turnrate = diffAngle;	//set turnrate
				if (std::abs(diffAngle) < 0.01)	//move to position
					speed = sqrt(diffY*diffY+diffX*diffX)/2;
				else speed = 0;	//stay in place and find angle
				
				std::cout << "We are going to\nX: " << plan[i] << "\nY: " << plan[i+1] << std::endl;
			}
			//print info
			std::cout << "Speed: " << speed << std::endl;      
			std::cout << "Turn rate: " << turnrate << std::endl << std::endl;
			pp.SetSpeed(speed, turnrate);
		}
	}
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
void readMap(int map[SIZE][SIZE]) {
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
	for(int i = SIZE; i >= -1; i--) {
		std::cout << "0 ";
		for(int j = 0; j < SIZE; j++)
			if (i == SIZE || i == -1)
				std::cout << "0 ";
			else if (map[i][j] == 0 || map[i][j] == 6)
				std::cout << "  ";
			else
				std::cout << map[i][j] << " ";
		std::cout << "0\n";
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
void writeMap(int map[SIZE][SIZE]) {
	std::ofstream mapFile;
	mapFile.open("map-out.txt");
	
	mapFile << "Occupancy grid/map used for pathfinding and navigation\n\nLegend:"
			<< "\n\t0: Boundary/border\n\t1: Obstacle/occupied square"
			<< "\n\t2: Dialated obstacle locations\n\t3: Expected path"
			<< "\n\t4: Waypoints\n\t5: Starting location\n" << std::endl;

	for(int i = SIZE; i >= -1; i--) {
		mapFile << "0 ";
		for(int j = 0; j < SIZE; j++)
			if (i == SIZE || i == -1)
				mapFile << "0 ";
			else if (map[i][j] == 0 || map[i][j] == 6)
				mapFile << "  ";
			else
				mapFile << map[i][j] << " ";
		mapFile << "0\n"; 
	}
	mapFile << std::endl;
	mapFile.close();
}


/**
 * Dialates an occupancy grid
 * Assumes the grid is marked with 1's for obstacles and 0's for open spaces
 * Done to prevent the robot from creating a path that causes it to run into corners 
 */
void dialateMap(int map[SIZE][SIZE]) {
	for(int i = SIZE-1; i > 0; i--) {
		for(int j = 0; j < SIZE-1; j++) {	//check surrounding nodes
			if (map[i][j] == 1) {	
				if (map[i][j+1] == 0) map[i][j+1] = 2;
				if (map[i-1][j] == 0) map[i-1][j] = 2;
				if (map[i-1][j+1] == 0) map[i-1][j+1] = 2;
			}
			else if (map[i][j+1] == 1) {
				if (map[i][j] == 0) map[i][j] = 2;
				if (map[i-1][j] == 0) map[i-1][j] = 2;
			}
			else if (map[i-1][j] == 1) {
				if (map[i][j] == 0) map[i][j] = 2;
				if (map[i][j+1] == 0) map[i][j+1] = 2; 
			}
			else if (map[i][j] == 0 && map[i-1][j+1] == 1)
				map[i][j] = 2;	
		}
		//end case; needed to prevent segmentation fault
		if (map[i][SIZE-1] == 0 && map[i-1][SIZE-1] == 1)
			map[i][SIZE-1] = 2;
		else if (map[i][SIZE-1] == 1 && map[i-1][SIZE-1] == 0)
			map[i-1][SIZE-1] = 2;
	}
	//end case; needed to prevent segmentation fault
	for(int j = 0; j < SIZE-1; j++) {
		if (map[0][j] == 0 && map[0][j+1] == 1)
			map[0][j] = 2;
		else if (map[0][j] == 1 && map[0][j+1] == 0)
			map[0][j+1] = 2;
	}
}


/**
 * Finds a path around an occupancy grid
 * Assumes the map is marked with 0's for open spaces
 */
std::vector<int> findPath(double startX, double startY, double endX, double endY, int map[SIZE][SIZE]) {
	//convert map coordinate to array locations
	startX = startX*2+16;
	startY = startY*2+16;
	endX = endX*2+16;
	endY = endY*2+16;
	
	
	std::priority_queue<Node, std::vector<Node>,std::less<std::vector<Node>::value_type> > openNodes;
	Node* myNode = new Node(startY, startX, endY, endX);
	std::vector<Node*> closedNodes (1, myNode);
	map[myNode->y][myNode->x] = 5;	//mark starting node
	
	while (myNode->x != endX || myNode->y != endY) {
		//check surrounding squares for closest to goal
		for (int i = myNode->y+1; i > myNode->y-2 && i >= 0; i--) {
			if (i > SIZE-1) i--; //boundary avoidance
			for (int j = myNode->x-1; j < myNode->x+2 && j < SIZE; j++) {
				if (j < 0) j++; //boundary avoidance
				if (map[i][j] == 0)	//if node is open
					openNodes.push(Node(i,j, endY, endX, myNode));
			}
		}
		if (!openNodes.empty()) {
			while (map[openNodes.top().y][openNodes.top().x] != 0)
				openNodes.pop();
			myNode = new Node(openNodes.top());	//get new node
			openNodes.pop(); 
			
			//mark new node as closed
			closedNodes.push_back(myNode);
			map[myNode->y][myNode->x] = 6;
		}
		else {
			std::cout << "No path could be found." << std::endl;
			myNode->y = endY; //to terminate while loop
			myNode->x = endX; //to terminate while loop
		}
	}
	
	
	std::vector<int> myNodes;
	while (myNode->parent != NULL) {	//find path from end to start
		myNodes.push_back(myNode->y*100+myNode->x);
		map[myNode->y][myNode->x] = 3;
		myNode = myNode->parent;
	}
	myNodes.push_back(myNode->y*100+myNode->x);
	std::reverse(myNodes.begin(), myNodes.end());
	
	
	for (int i = 0; i < closedNodes.size(); i++) { //free up memory on heap
		delete closedNodes[i];
	}
	
	return myNodes;
}


/**
 * Finds waypoints from a list of nodes and marks them on an occupancy grid
 */
void findWaypoints (std::vector<int>& myNodes, int map[SIZE][SIZE]) {
	int diffLastY, diffLastX, diffNextY, diffNextX, lastWaypoint = 0;
	
	for (int i = 1; i < myNodes.size()-1; i++) {
		//calculate direction
		diffLastY = myNodes[i-1]/100-myNodes[i]/100;
		diffNextY = myNodes[i]/100-myNodes[i+1]/100;
		diffLastX = myNodes[i-1]%100-myNodes[i]%100;
		diffNextX = myNodes[i]%100-myNodes[i+1]%100;
		
		if (diffLastY != diffNextY || diffLastX != diffNextX) { //if direction changes
			if (i-lastWaypoint > 1)	//delete nodes between current location and last waypoint
				myNodes.erase(myNodes.begin()+lastWaypoint+1, myNodes.begin()+i); 
			i = ++lastWaypoint;	//update the last waypoint
		}	
	}
	if ((myNodes.size()-1) - lastWaypoint > 1)	//for set of nodes between last waypoint and goal
		myNodes.erase(myNodes.begin()+lastWaypoint+1, myNodes.end()-1);
	myNodes.erase(myNodes.begin());	//remove starting location from the waypoint list
	
	//update map array
	for (int i = 0; i < myNodes.size(); i++)
		map[myNodes[i]/100][myNodes[i]%100] = 4;
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
  planFile.open("plan-out.txt");

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
  planFile.open("plan-out.txt");

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

void writePlan(std::vector<int> myNodes)
{
	std::ofstream planFile;
	planFile.open("plan-out.txt");

	planFile << (myNodes.size())*2 << " ";
	for(int i = 0; i < myNodes.size(); i++) {
		planFile << double(myNodes[i]%100)/2-7.75 << " " << double(myNodes[i]/100)/2-7.75 << " ";
	}
	planFile.close();
} // End of writePlan
