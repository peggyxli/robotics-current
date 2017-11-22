/**
 * local-roomba.cc
 **/


#include <iostream>
#include <libplayerc++/playerc++.h>
#include <cmath>
using namespace PlayerCc;  

/**
 * Function headers
 **/
player_pose2d_t readPosition(LocalizeProxy& lp);
void printRobotData(BumperProxy& bp, player_pose2d_t pose);

int main(int argc, char *argv[]){  

	int counter = 0;
	double speed;            // How fast do we want the robot to go forwards?
	double turnrate;         // How fast do we want the robot to turn?
	player_pose2d_t  pose;   // For handling localization data
	double myAngle = 0, diffX = 0, diffY = 0;

  // Set up proxies. These are the names we will use to connect to 
  // the interface to the robot.
	PlayerClient    robot("localhost");  
	BumperProxy     bp(&robot,0);  
	Position2dProxy pp(&robot,0);
	LocalizeProxy   lp (&robot, 0);

  // Allow the program to take charge of the motors (take care now)
	pp.SetMotorEnable(true);

  // Main control loop
	do {    
    	robot.Read();				// Update information from the robot.
    	pose = readPosition(lp);	// Read new information about position
		printRobotData(bp, pose);	// Print data on the robot to the terminal
		
		diffY = -3.5 - pose.py;
		diffX = 5 - pose.px;
		myAngle = atan2(diffY, diffX);
		std::cout << "My angle: " << myAngle << std::endl;
    	
    	if(bp[0] || bp[1]){
			speed= 0;
			turnrate= 0;
    	} 
    	else {
    		turnrate = myAngle - pose.pa;
			if (myAngle - pose.pa < 0.0001)
				speed = sqrt(diffX*diffX+diffY*diffY);
			else
				speed = 0;
      	}

    	std::cout << "Speed: " << speed << std::endl;      
    	std::cout << "Turn rate: " << turnrate << std::endl << std::endl;
    	pp.SetSpeed(speed, turnrate);  
    } while (pose.px < 4.9999);
} // end of main()


/*
 * readPosition()
 * Read the position of the robot from the localization proxy. 
 * The localization proxy gives us a hypothesis, and from that we extract
 * the mean, which is a pose. 
 */
player_pose2d_t readPosition(LocalizeProxy& lp) {
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
}


/*
 *  printRobotData
 * Print out data on the state of the bumpers and the current location
 * of the robot.
 **/
void printRobotData(BumperProxy& bp, player_pose2d_t pose){
	// Print out what the bumpers tell us:
	std::cout << "Left  bumper: " << bp[0] << std::endl;
	std::cout << "Right bumper: " << bp[1] << std::endl;

	// Print out where we are
	std::cout << "We are at" << std::endl;
	std::cout << "X: " << pose.px << std::endl;
	std::cout << "Y: " << pose.py << std::endl;
	std::cout << "A: " << pose.pa << std::endl;
}