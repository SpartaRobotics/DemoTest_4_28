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

<<<<<<< HEAD
=======
#define SEC 1000000 // 1 Second in micro second units for delay
#define MSEC 1000	// 1 milli second in micro second units for delay

#define BAUD_RATE B115200

#define CLW_OPN 160 // Open Claw
#define CLW_CLS 490 // Close Claw

#define ID1_MIN 0
#define ID2_MIN 0
#define ID3_MIN 1250
#define ID4_MIN 500
#define ID5_MIN 590
#define ID6_MIN 0

#define ID1_MAX 1200
#define ID2_MAX 3600
#define ID3_MAX 3250
#define ID4_MAX 3150
#define ID5_MAX 3260
#define ID6_MAX 4095

#define ID1_RST 575
#define ID2_RST 2250
#define ID3_RST 2250
#define ID4_RST 1830
#define ID5_RST 1950
#define ID6_RST 2650

#define DSPEED 5 // Default Speed

void servoINIT(UsbMX *control, int *servoPositions);

>>>>>>> 706cb78c5422c2a9df374e4e57e18fc35cef4d68
int main(int argc, const char *argv[])
{
	int chaseSignal = 0;
	float coordDist[3];
<<<<<<< HEAD
	
	cout << "Before starting the mission initialize the servos " <<
			"to hold the starting ARM configuration" << endl << endl;
	// servoINIT();
=======
	char moveInput, eeInput;
	
	// Position values for each servo
	int servoPOS[7] = {ID1_RST,ID2_RST,ID3_RST,ID4_RST,
					     ID5_RST,ID6_RST,CLW_OPN};
					     
	UsbMX control; // For Dyanmixel Servo CONTROL
	
	
	cout << endl;
	cout << "Before starting the mission initialize the servos " <<
			"to hold the starting ARM configuration" << endl << endl;
	control.begin("/dev/ttyUSB0", BAUD_RATE);
	servoINIT(&control, servoPOS);
>>>>>>> 706cb78c5422c2a9df374e4e57e18fc35cef4d68
	
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
<<<<<<< HEAD
	
	/********************* GRAB PHASE **************************/
	cout << "Now RM is beggining the capture phase! << endl;
=======
	cout << endl;
	
	/********************* GRAB PHASE **************************/
	cout << "Now RM is beggining the capture phase!" << endl << endl;
>>>>>>> 706cb78c5422c2a9df374e4e57e18fc35cef4d68
	
	cout << "Turning on the Base Camera" << endl;
	// baseCameraINIT(); // Turn on Base Camera
	
<<<<<<< HEAD
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
	
=======
	cout << "Recognizing the capture ring!" << endl << endl;
	// recognizeCaptureRing(); // returns the X,Y,Z cartesians;

	while(moveInput != 'n')
	{
		cout << "Enter the coordinate distances one at a time (in mm)!" << endl;
		cout << "X: ";
		cin >> coordDist[0];
		cout << "Y: ";
		cin >> coordDist[1];
		cout << "Z: ";
		cin >> coordDist[2];

		cout << "Sending to FK & IK and moving to desired position" << endl;
		cout << coordDist[0] << " " << coordDist[1] << " " << coordDist[2] << endl;
		cout << "Moving servos to position!" << endl << endl;
		// ControlAlgorithm()

		cout << "Close the end-effector? (\'y\' or \'n\')" << endl;
		cin >> eeInput;

		if(eeInput == 'y')
		{
			// closeEE();
		}
		cout << endl;

		cout << "Continue with grabbing? (\'y\' or \'n\'): " << endl;
		cin >> moveInput;
		
	}
	
	cout << endl;
	cout << "Grabbing phase has been completed!" << endl << endl;

	cout << "Now signaling the Chase satellite for docking phase" << endl;
	cout << "Enter a '2' to signal the Chase to proceed" << endl;

	while( chaseSignal != 2)
	{
		cout << "Enter 2 to proceed: ";
		cin >> chaseSignal;
	}
	cout << endl;

	cout << "Signaling chase to turn off thrust controller and " <<
			"begin the assisted docking phase" << endl;
	cout << "GRAB PHASE COMPLETED!!!" << endl << endl;

	/************************ END GRAB PHASE **********************/ 
	
	/************************ START DOCKING PHASE *****************/
	
	cout << "STARTING DOCKING PHASE!!!" << endl << endl;
>>>>>>> 706cb78c5422c2a9df374e4e57e18fc35cef4d68

	return 0;
}

<<<<<<< HEAD
=======
void servoINIT(UsbMX *control, int *servoPositions)
{
	for(int i = 1; i < 7; i++)
	{
			control->setEndless(i, ON);
	}
		
	for(int i = 0; i < 6; i++)
	{
		control->moveSpeed(i+1, servoPositions[i], DSPEED);
	}
}

>>>>>>> 706cb78c5422c2a9df374e4e57e18fc35cef4d68
