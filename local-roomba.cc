/*
 * Robotics class project #4, part 1
 * Modified version of local-roomba.cc, originally written by Simon Parsons
 * Used to navigate a Roomba/Create to a specified location
 * Group 5: Randolph Cisneros, Arsenii Lyzenko, Peggy Li
 */

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <libplayerc++/playerc++.h>
using namespace PlayerCc;  

//Function headers
player_pose2d_t readPosition(LocalizeProxy& lp);
void printRobotData(BumperProxy& bp, player_pose2d_t pose);


int main(int argc, char *argv[]){  
	//set up proxies to connect the interface to the robot
	player_pose2d_t  pose;
	PlayerClient    robot("localhost");  
	BumperProxy     bp(&robot,0);  
	Position2dProxy pp(&robot,0);
	LocalizeProxy   lp (&robot, 0);
	
	double speed, turnrate, diffY, diffX, diffAngle;
	pp.SetMotorEnable(true);
	do {
		//update and print information from the robot
    	robot.Read();
    	pose = readPosition(lp);
		printRobotData(bp, pose);
		
    	if(bp[0] || bp[1]) {	//obstacle navigation
			turnrate = 1.0;
			if (rand()%2 == 0)
				speed = 1.0;
			else speed = -1.0;
    	} 
    	else {	//locate and move towards position
			//calculate angle needed to move to end position
			diffY = -3.5 - pose.py;
			diffX = 5 - pose.px;
			diffAngle = atan2(diffY, diffX) - pose.pa;
			
			turnrate = diffAngle;	//set turnrate
			if (diffAngle < 0.001)	//move to position
				speed = sqrt(diffY*diffY+diffX*diffX);
			else speed = 0;	//stay in place and find angle
		}
    	std::cout << "Speed: " << speed << std::endl;      
    	std::cout << "Turn rate: " << turnrate << std::endl << std::endl;
    	pp.SetSpeed(speed, turnrate);  
    } while (pose.px < 4.9999);
}


/* Function provided by sample code
 * Read the position of the robot from the localization proxy. 
 * The localization proxy gives us a hypothesis, and from that we extract the mean, which is a pose. 
 */
player_pose2d_t readPosition(LocalizeProxy& lp) {
	player_pose2d_t pose;
	uint32_t hCount = lp.GetHypothCount();

	if(hCount > 0)
    	pose = lp.GetHypoth(0).mean;
  	return pose;
}


/* Function provided by sample code
 * Print bumpers and location
 */
void printRobotData(BumperProxy& bp, player_pose2d_t pose){
	// Print out what the bumpers tell us
	std::cout << "Left  bumper: " << bp[0] << std::endl;
	std::cout << "Right bumper: " << bp[1] << std::endl;
	// Print out where we are
	std::cout << "We are at" << std::endl;
	std::cout << "X: " << pose.px << std::endl;
	std::cout << "Y: " << pose.py << std::endl;
	std::cout << "A: " << pose.pa << std::endl;
}