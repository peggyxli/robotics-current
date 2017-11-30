/**
 * Robotics class project #5
 * Modified version of follow-plan.cc, originally written by Simon Parsons
 * Used to navigate a Roomba/Create to a set of specified locations and avoid obstacles along the way.
 * Group 5: Randolph Cisneros, Arsenii Lyzenko, Peggy Li
 **/


#include <iostream>
#include <fstream>
#include <libplayerc++/playerc++.h>
using namespace PlayerCc;


//Function headers
player_pose2d_t readPosition(LocalizeProxy& lp);
void printRobotData(BumperProxy& bp, player_pose2d_t pose);
void printLaserData(LaserProxy& sp);
int  readPlanLength(void);
void readPlan(double *, int);


int main(int argc, char *argv[])
{
	//Set up proxies to connect the interface to the robot
	player_pose2d_t pose;
	PlayerClient robot("localhost");
	BumperProxy bp(&robot,0);
	Position2dProxy pp(&robot,0);
	LocalizeProxy lp (&robot, 0);
	LaserProxy sp (&robot, 0);

	
	/* A plan is an integer, n, followed by n doubles (n has to be even). 
	 * The first and second doubles are the initial x and y (respectively) coordinates of the robot,
	 * the third and fourth doubles give the first location that the robot should move to, and so on. 
	 * The last pair of doubles give the point at which the robot should stop. */
	int pLength = readPlanLength(); // Find out how long the plan is from plan.txt
	double *plan = new double[pLength]; // Create enough space to store the plan
	readPlan(plan, pLength);    // Read the plan from the file plan.txt.
	
	
	int counter = 0;
	double speed, turnrate, diffY, diffX, diffAngle;
	pp.SetMotorEnable(true);
	
	
	for (int i = 0; i < pLength; i = i + 2) {	//for each pair of coordinates
		//navigate to waypoint
		while (std::abs(pose.px - plan[i]) > 0.001 || std::abs(pose.py - plan[i + 1]) > 0.001) {
			//update and print information from the robot
			robot.Read();
			pose = readPosition(lp);
			printRobotData(bp, pose);
			
			if (counter++ > 2) {
				if (sp.MinLeft() < .5) { //obstacle avoidance
					turnrate = sp.MinLeft() - 2;
					speed = sp.MinLeft()/2;
					std::cout << "Obstacle avoidance in progress." << std::endl;
				}
				else if (sp.MinRight() < .5) {	//obstacle avoidance
					turnrate = 2 - sp.MinRight();
					speed = sp.MinRight()/2;
					std::cout << "Obstacle avoidance in progress." << std::endl;
				}
				else {	//locate and move towards position
					//calculate angle needed to move to end position
					diffY = plan[i + 1] - pose.py;
					diffX = plan[i] - pose.px;
					diffAngle = atan2(diffY, diffX) - pose.pa;
					
					turnrate = diffAngle;	//set turnrate
					if (std::abs(diffAngle) < 0.001)	//move to position
						speed = sqrt(diffY*diffY+diffX*diffX);
					else speed = 0;	//stay in place and find angle
					
					std::cout << "We are going to\nX: " << plan[i] << "\nY: " << plan[i+1] << std::endl;
				}
			}
			//print info
			std::cout << "Speed: " << speed << std::endl;      
			std::cout << "Turn rate: " << turnrate << std::endl << std::endl;
			pp.SetSpeed(speed, turnrate);
		}
	}
} // end of main()


/**
 * readPosition
 * Function provided by sample code
 * Read the position of the robot from the localization proxy. 
 * The localization proxy gives us a hypothesis, and from that we extract the mean, which is a pose. 
 **/
player_pose2d_t readPosition(LocalizeProxy& lp) {
	player_pose2d_t pose;
	uint32_t hCount = lp.GetHypothCount();

	if(hCount > 0)
    	pose = lp.GetHypoth(0).mean;
  	return pose;
}


/**
 * printRobotData
 * Function provided by sample code
 * Print bumpers and location
 **/
void printRobotData(BumperProxy& bp, player_pose2d_t pose) {
	// Print out what the bumpers tell us
	std::cout << "Left  bumper: " << bp[0] << std::endl;
	std::cout << "Right bumper: " << bp[1] << std::endl;
	// Print out where we are
	std::cout << "We are at" << std::endl;
	std::cout << "X: " << pose.px << std::endl;
	std::cout << "Y: " << pose.py << std::endl;
	std::cout << "A: " << pose.pa << std::endl;
}


/**
 * readPlanLength
 * Function provided by sample code
 * Open the file plan.txt and read the first element, which should be an even integer, and return it.
 **/
int readPlanLength(void) {
	int length;
	std::ifstream planFile;
	planFile.open("plan.txt");

	planFile >> length;
	planFile.close();

	//Some minimal error checking
	if((length % 2) != 0){
		std::cout << "The plan has mismatched x and y coordinates" << std::endl;
		exit(1);
	}
	return length;
}


/**
 * readPlan
 * Function provided by sample code
 * Given the number of coordinates, read them in from plan.txt and put them in the array plan.
 **/
void readPlan(double *plan, int length) {
	int skip;
	std::ifstream planFile;
	planFile.open("plan.txt");

	planFile >> skip;
	for(int i = 0; i < length; i++)
		planFile >> plan[i];
	planFile.close();
}