/*
	Mock Mission Test of Full Features
	* Start
	* Grab
	* Assiting Docking
	* Refuel
	* END
	
	For incomplete features add mock or dummy features
	--Current Dummy Features
	* User input X,Y,Z cartesian coordinates
	* User input Serial communication to simulate indepedent stages
*/

#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "UsbMX.h"
#include "JHPWMPCA9685.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>

using namespace std;
using namespace cv;

int main(int argc, const char *argv[])
{
	int chaseSignal = 0;
	float coordDist[3];
	
	cout << "Before starting the mission initialize the servos " <<
			"to hold the starting ARM configuration" << endl << endl;
	// servoINIT();
	
	cout << "Chase is moving towards station keeping position! " << endl;
	cout << "Waiting 3 seconds until Chase is within range!" << endl << endl;
	
	// Delay to simulate Chase aligning to capture position
	sleep(3);
	// chaseReceive(1); // 1 for Capture Phase start

	cout << "Chase sending the \'1\' flag to simulate the RM " <<
			"receiving the signal. Enter a \'1\' to proceed" << endl;
			
	while( chaseSignal != 1)
	{
		cout << "Enter 1 to proceed: ";
		cin >> chaseSignal;
	}
	
	/********************* GRAB PHASE **************************/
	cout << "Now RM is beggining the capture phase! << endl;
	
	cout << "Turning on the Base Camera" << endl;
	// baseCameraINIT(); // Turn on Base Camera
	
	cout << "Recognizing the capture ring!" << endl;
	// recognizeCaptureRing(); // returns the X,Y,Z cartesians;
	cout << "Enter the coordinate distances 1 at a time! " << endl;
	cout << "X: ";
	cin >> coordDist[0];
	cout << "Y: ";
	cin >> coordDist[1];
	cout << "Z: ";
	cin >> coordDist[2];
	
	cout << "Sending to FK and IK to move RM to desired position!" << endl;
	// controlAlgorithm(); // returns values to RM to desired position
	// closeEE(); // close EE to grab
	/************************ END GRAB PHASE **********************/ 
	
	/************************ START DOCKING PHASE *****************/
	cout << "Signaling chase to turn off thrust controller and " <<
			"begin the assisted docking phase" << endl;
	

	return 0;
}

