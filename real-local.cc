/**
 * real-local.cc
 * This version is intended to run with the AMCL localization proxy,
 * which provides multiple hypotheses.
 * 
 **/


#include <iostream>
#include <libplayerc++/playerc++.h>
using namespace PlayerCc;  

//Function headers
player_pose2d_t readPosition(LocalizeProxy& lp);
void printLaserData(LaserProxy& sp);
void printRobotData(BumperProxy& bp, player_pose2d_t pose);

int main(int argc, char *argv[]) {  

	int counter = 0;
	double speed;           
	double turnrate;        
	player_pose2d_t  pose;   // For handling localization data
	player_laser_data laser; // For handling laser data
	bool locationFound = false;
	
	// Set up proxies. These are the names we will use to connect to 
	// the interface to the robot.
	PlayerClient    robot("localhost");  
	BumperProxy     bp(&robot,0);  
	Position2dProxy pp(&robot,0);
	LocalizeProxy   lp (&robot, 0);
	LaserProxy      sp (&robot, 0);

	pp.SetMotorEnable(true);
	
	speed = 0;
    turnrate = .1;

	pp.SetSpeed(speed, turnrate);  
	std::cout << "Speed: " << speed << std::endl;      
	std::cout << "Turn rate: " << turnrate << std::endl << std::endl;

	while (!locationFound) {
    	robot.Read();				//Update information
    	pose = readPosition(lp);	//Read position
		if (lp.GetHypothCount() > 0)
			if (lp.GetHypoth(0).alpha > .9 && lp.GetHypothCount() < 4)
				locationFound = true;
    }
}


/**
 * readPosition()
 *
 * Read the position of the robot from the localization proxy. 
 *
 * The localization proxy gives us a set of "hypotheses", each of
 * which is a number of possible locations for the robot, and from
 * each we extract the mean, which is a pose.
 *
 * As the number of hypotheses drops, the robot should be more sure
 * of where it is.
 *
 **/

player_pose2d_t readPosition(LocalizeProxy& lp) {

	player_localize_hypoth_t hypothesis;
	player_pose2d_t          pose;
	uint32_t                 hCount;
	double                   weight;

	// Need some messing around to avoid a crash when the proxy is
	// starting up.

	hCount = lp.GetHypothCount();

	std::cout << "AMCL gives us " << hCount + 1 
            << " possible locations:" << std::endl;

	if(hCount > 0){
	    for(int i = 0; i <= hCount; i++){
    		hypothesis = lp.GetHypoth(i);
    		pose       = hypothesis.mean;
    		weight     = hypothesis.alpha;
    		std::cout << "X: " << pose.px << "\t";
    		std::cout << "Y: " << pose.py << "\t";
    		std::cout << "A: " << pose.pa << "\t";
    		std::cout << "W: " << weight  << std::endl;
		}
	}

  // This just returns the mean of the last hypothesis, it isn't necessarily
  // the right one.

  return pose;
}

//Take laser readins and print laser data
void printLaserData(LaserProxy& sp) {

double maxRange, minLeft, minRight, range, bearing;
	int points;

	maxRange  = sp.GetMaxRange();
	minLeft   = sp.MinLeft();
	minRight  = sp.MinRight();
	range     = sp.GetRange(5);
	bearing   = sp.GetBearing(5);
	points    = sp.GetCount();

	//Print out laser data
	std::cout << "Laser says..." << std::endl;
	std::cout << "Maximum distance I can see: " << maxRange << std::endl;
	std::cout << "Number of readings I return: " << points << std::endl;
	std::cout << "Closest thing on left: " << minLeft << std::endl;
	std::cout << "Closest thing on right: " << minRight << std::endl;
	std::cout << "Range of a single point: " << range << std::endl;
	std::cout << "Bearing of a single point: " << bearing << std::endl;

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
}
