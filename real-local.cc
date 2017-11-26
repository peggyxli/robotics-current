/**
 * real-local.cc
 * This version is intended to run with the AMCL localization proxy,
 * which provides multiple hypotheses.
 * 
 **/


#include <iostream>
#include <cstdlib>
#include <cmath>
#include <libplayerc++/playerc++.h>
using namespace PlayerCc;  

//Function headers
player_pose2d_t readPosition(LocalizeProxy& lp);
void printLaserData(LaserProxy& sp);
void printRobotData(BumperProxy& bp, player_pose2d_t pose);
void chooseRandom(std::string, double&);

int main(int argc, char *argv[]) {  
	//set up proxies to connect the interface to the robot
	player_pose2d_t pose;   // For handling localization data
	player_laser_data laser; // For handling laser data
	PlayerClient robot("localhost");  
	BumperProxy bp(&robot,0);  
	Position2dProxy pp(&robot,0);
	LocalizeProxy lp (&robot, 0);
	LaserProxy sp (&robot, 0);
	
	bool locationFound = false;
	pp.SetMotorEnable(true);
	pp.SetSpeed(0.2, 0.1);

	while (!locationFound) {	
    	robot.Read();
    	pose = readPosition(lp);
		if (lp.GetHypothCount() > 0)
			if (lp.GetHypoth(0).alpha > .9 && lp.GetHypothCount() < 3)
				locationFound = true;	
    }

	if (pose.px < -4 && pose.py < -4) {
		std::cout << "\nSuccess!\nI have successfully located my position.";
		std::cout << "\nI am at (" << pose.px << "," << pose.py << ").";
		std::cout << "\nI am " << lp.GetHypoth(0).alpha*100 << " percent sure of my location." << std::endl;
				  
		double speed, turnrate, diffY, diffX, diffAngle;
		while (pose.px < 5 || pose.py < -3.5) {
			robot.Read();
			pose = readPosition(lp);
			printRobotData(bp, pose);
			printLaserData(sp);
			
			if (sp.MinLeft() < .5) {
				turnrate = sp.MinLeft() - 2;
				speed = sp.MinLeft()/2;
			}
			else if (sp.MinRight() < .5) {
				turnrate = 2 - sp.MinRight();
				speed = sp.MinRight()/2;
			}
			else {
				diffY = -3.5 - pose.py;
				diffX = 5 - pose.px;
				diffAngle = atan2(diffY, diffX) - pose.pa;
				
				turnrate = diffAngle;
				if (diffAngle < 0.001)	
					speed = sqrt(diffY*diffY+diffX*diffX);
				else speed = 0;
			}

			std::cout << "Speed: " << speed << std::endl;      
			std::cout << "Turn rate: " << turnrate << std::endl << std::endl;
			pp.SetSpeed(speed, turnrate);
		}
		std::cout << "\nSuccess!\nI have successfully navigated my final position.";
		std::cout << "\nI am at (" << pose.px << "," << pose.py << ")." << std::endl;
	}
}


/*
 * Read the position of the robot from the localization proxy. 
 *
 * The localization proxy gives us a set of "hypotheses", 
 * each of which is a number of possible locations for the robot,
 * and from each we extract the mean, which is a pose.
 */
player_pose2d_t readPosition(LocalizeProxy& lp) {
	player_pose2d_t pose;
	double weight;
	uint32_t hCount = lp.GetHypothCount();
	
	//Print AMCL data
	std::cout << "\nAMCL gives us " << hCount + 1 << " possible locations:" << std::endl;
	if(hCount > 0){	//check first, to avoid seg fault when booting up
	    for(int i = 0; i <= hCount; i++){
    		pose       = lp.GetHypoth(i).mean;
    		weight     = lp.GetHypoth(i).alpha;
    		std::cout << "X: " << pose.px << "\t";
    		std::cout << "Y: " << pose.py << "\t";
    		std::cout << "A: " << pose.pa << "\t";
    		std::cout << "W: " << weight  << std::endl;
		}
		return lp.GetHypoth(0).mean;	//return most likely hypothesis
	}
	return pose;
}


//Take laser readings and print laser data
void printLaserData(LaserProxy& sp) {
	//Print laser data
	std::cout << "Laser says..." << std::endl;
	std::cout << "Maximum distance I can see: " << sp.GetMaxRange() << std::endl;
	std::cout << "Number of readings I return: " << sp.GetCount() << std::endl;
	std::cout << "Closest thing on left: " << sp.MinLeft() << std::endl;
	std::cout << "Closest thing on right: " << sp.MinRight() << std::endl;
	return;
}


//Print bumpers and location
void printRobotData(BumperProxy& bp, player_pose2d_t pose) {
	// Print bumpers
	std::cout << "Left  bumper: " << bp[0] << std::endl;
	std::cout << "Right bumper: " << bp[1] << std::endl;
	// Print location
	std::cout << "We are at" << std::endl;
	std::cout << "X: " << pose.px << std::endl;
	std::cout << "Y: " << pose.py << std::endl;
	std::cout << "A: " << pose.pa << std::endl;
	return;
}