// Testing Manual Control!

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

// One (and only one) of your C++ files must define CVUI_IMPLEMENTATION
// before the inclusion of cvui.h to ensure its implementaiton is compiled.
#define CVUI_IMPLEMENTATION
#include "cvui.h"

#define WINDOW_NAME "RM CONTROL"
#define SEC 1000000 // 1 Second in micro second units for delay
#define MSEC 1000	// 1 milli second in micro second units for delay

#define BAUD_RATE B115200

#define CLW_OPN 120 // Open Claw
#define CLW_CLS 420 // Close Claw

#define ID1_MIN 0
#define ID2_MIN 600
#define ID3_MIN 940
#define ID4_MIN 515
#define ID5_MIN 610
#define ID6_MIN 1665

#define ID1_MAX 1500
#define ID2_MAX 3186
#define ID3_MAX 3300
#define ID4_MAX 3160
#define ID5_MAX 3260
#define ID6_MAX 3715

#define ID1_RST 885
#define ID2_RST 795
#define ID3_RST 2235
#define ID4_RST 1810
#define ID5_RST 1945
#define ID6_RST 646

#define DSPEED 5 // Default Speed

using namespace std;
using namespace cv;

VideoCapture clawCam;
Mat frame;
char quit;

void displayGUI(UsbMX *control, int *servoThres, int *servoCon, 
				int *servoReadPos, int *moveStatus, float *baseCoords,
				float *clawCoords);
void toggleServo(UsbMX *control, int servoId, int *status, int posX, int posY);
int getkey();
int map ( int x, int in_min, int in_max, int out_min, int out_max);

int main(int argc, const char *argv[])
{
	frame = cv::Mat(650, 1250, CV_8UC3);
	char c;
	int err;
	int count = 0;
	int speed = 0;
	
	// Position values for each servo
	int servoThres[7] = {ID1_RST,ID2_RST,ID3_RST,ID4_RST,
					     ID5_RST,ID6_RST,CLW_OPN};
	
	// Position for executing concurrent arm configuration
	int servoCon[6] = {ID1_RST,ID2_RST,ID3_RST,ID4_RST,
					   ID5_RST,ID6_RST};
	
	// Read positions from servos
	int servoReadPos[6];
	
	// Indicates if the servos will move
	int moveStatus[6] = {0,0,0,0,0,0};
	
	// Stores Base camera cartesian coordinates
	float baseCoords[3] = {0,0,0};
	
	// Stores Claw camera cartesian coordinates
	float clawCoords[3] = {0,0,0};
	
	//VideoCapture clawCam;
	clawCam.open(0);
	Mat clawFrame;
	
	VideoCapture baseCam;/*
	baseCam.open(1);
	Mat baseFrame;
	*/
	Mat currentFrame;
	
	PCA9685 *pca9685 = new PCA9685() ;
    err = pca9685->openPCA9685();
    if (err < 0){
        printf("Error: %d", pca9685->error);
    } else {
        pca9685->setAllPWM(0,0) ;
        pca9685->reset() ;
        pca9685->setPWMFrequency(60);
    }

	UsbMX control;
	control.begin("/dev/ttyUSB0", BAUD_RATE);

	// Init cvui and tell it to create a OpenCV window, i.e. cv::namedWindow(WINDOW_NAME).
	cvui::init(WINDOW_NAME);

	while (true) {
		
		displayGUI(&control, servoThres, servoCon, servoReadPos, 
				   moveStatus, baseCoords, clawCoords);
		
		for(int i = 0; i < 6; i++)
		{
			servoReadPos[i] = control.readPosition(i+1);
			
			if(moveStatus[i])   
			{
				control.moveSpeed(i+1, servoThres[i], DSPEED);
			}
		}
		
		pca9685->setPWM(0,0,servoThres[6]);
        pca9685->setPWM(1,0,servoThres[6]);
        
		// Check if ESC key was pressed
		c = cv::waitKey(33);
		if( c == 'q' || c == 27 || quit == 'q') break;
	}

	pca9685->setPWM(0,0,CLW_OPN);
    pca9685->setPWM(1,0,CLW_OPN);
    sleep(1);
    pca9685->closePCA9685();
    
	control.disconnect();

	return 0;
}

void displayGUI(UsbMX *control, int *servoThres, int *servoCon, 
				int *servoReadPos, int *moveStatus, float *baseCoords,
				float *clawCoords)
{
	
	// Fill the frame with a nice color
	frame = cv::Scalar(49, 52, 49);
	
	
	if (cvui::button(frame, 50, 50, " EXIT PROGRAM ")) {
		quit = 'q';
	}
	
	if (cvui::button(frame, 250, 50, "GO TO RESET POSITION")) {
		servoThres[0] = ID1_RST;
		servoThres[1] = ID2_RST;
		servoThres[2] = ID3_RST;
		servoThres[3] = ID4_RST;
		servoThres[4] = ID5_RST;
		servoThres[5] = ID6_RST;
	}
	
	if (cvui::button(frame, 500, 50, "GO TO READ POSITIONS")) {
		for(int i = 0; i < 6; i++)
		{
			servoThres[i] = servoReadPos[i];
		}
	}
	
	if (cvui::button(frame, 750, 50, "POWER OFF ALL SERVOS")) {
		memset(moveStatus, 0, 6*sizeof(int) );
		for(int i = 1; i < 7; i++)
		{
			control->setEndless(i, ON);
		}
	}
	
	if (cvui::button(frame, 1000, 50, "POWER ON ALL SERVOS ")) {
		memset(moveStatus, 1, 6*sizeof(int) );
		for(int i = 1; i < 7; i++)
		{
			control->setEndless(i, OFF);
			servoThres[i-1] = servoReadPos[i-1];
		}
	}
	
	if (cvui::button(frame, 940, 575, "CONCURRENT MOVE")) {
		for(int i = 0; i < 6; i++)
		{
			servoThres[i] = servoCon[i];
		}
	}
	
	toggleServo(control, 1, &moveStatus[0], 105, 160);
	toggleServo(control, 2, &moveStatus[1], 105, 230);
	toggleServo(control, 3, &moveStatus[2], 105, 300);
	toggleServo(control, 4, &moveStatus[3], 105, 370);
	toggleServo(control, 5, &moveStatus[4], 105, 440);
	toggleServo(control, 6, &moveStatus[5], 105, 510);
	
	if (cvui::button(frame, 80, 580, "OPEN")) {
		servoThres[6] = CLW_OPN;
	}
	
	if (cvui::button(frame, 155, 580, "CLOSE")) {
		servoThres[6] = CLW_CLS;
	}

	cvui::counter(frame, 240, 162, &servoThres[0]);
	cvui::counter(frame, 240, 232, &servoThres[1]);
	cvui::counter(frame, 240, 302, &servoThres[2]);
	cvui::counter(frame, 240, 378, &servoThres[3]);
	cvui::counter(frame, 240, 442, &servoThres[4]);
	cvui::counter(frame, 240, 512, &servoThres[5]);
	
	cvui::trackbar(frame, 360, 155, 400, &servoThres[0], 0, 1000);
	cvui::trackbar(frame, 360, 225, 400, &servoThres[1], 600, 3186);
	cvui::trackbar(frame, 360, 295, 400, &servoThres[2], 940, 3300);
	cvui::trackbar(frame, 360, 365, 400, &servoThres[3], 515, 3160);
	cvui::trackbar(frame, 360, 435, 400, &servoThres[4], 610, 3260);
	cvui::trackbar(frame, 360, 505, 400, &servoThres[5], 1665, 3715);
	cvui::trackbar(frame, 360, 575, 400, &servoThres[6], 120, 420); // Claw EE
	
	cvui::trackbar(frame, 810, 155, 400, &servoCon[0], 0, 1000);
	cvui::trackbar(frame, 810, 225, 400, &servoCon[1], 600, 3186);
	cvui::trackbar(frame, 810, 295, 400, &servoCon[2], 940, 3300);
	cvui::trackbar(frame, 810, 365, 400, &servoCon[3], 515, 3160);
	cvui::trackbar(frame, 810, 425, 400, &servoCon[4], 610, 3260);
	cvui::trackbar(frame, 810, 505, 400, &servoCon[5], 1665, 3715);

	cvui::printf(frame, 525, 15, 0.8, 0xc1c1c1, "SPARTA RM INTERFACE");
	
	cvui::printf(frame, 860, 125, 0.6, 0xc1c1c1, "CONCURRENT SERVO MOVEMENT");
	cvui::printf(frame, 450, 125, 0.6, 0xc1c1c1, "SINGLE SERVO MOVEMENT");
	cvui::printf(frame, 105, 115, 0.4, 0xc1c1c1, "TOGGLE");
	cvui::printf(frame, 105, 135, 0.4, 0xc1c1c1, "ON/OFF");
	cvui::printf(frame, 170, 115, 0.4, 0xc1c1c1, "CURRENT");
	cvui::printf(frame, 170, 135, 0.4, 0xc1c1c1, "POSITION");
	cvui::printf(frame, 255, 115, 0.4, 0xc1c1c1, "MOVEMENT");
	cvui::printf(frame, 267, 135, 0.4, 0xc1c1c1, "(FINE)");
	
	cvui::printf(frame, 20, 165, 0.5, 0xc1c1c1, "SERVO 1:");
	cvui::printf(frame, 185, 165, 0.5, 0xc1c1c1, "%d", servoReadPos[0]);
	
	cvui::printf(frame, 20, 225, 0.5, 0xc1c1c1, "SERVO 2:");
	cvui::printf(frame, 185, 225, 0.5, 0xc1c1c1, "%d", servoReadPos[1]);
	
	cvui::printf(frame, 20, 305, 0.5, 0xc1c1c1, "SERVO 3:");
	cvui::printf(frame, 185, 305, 0.5, 0xc1c1c1, "%d", servoReadPos[2]);
	
	cvui::printf(frame, 20, 375, 0.5, 0xc1c1c1, "SERVO 4:");
	cvui::printf(frame, 185, 375, 0.5, 0xc1c1c1, "%d", servoReadPos[3]);
	
	cvui::printf(frame, 20, 445, 0.5, 0xc1c1c1, "SERVO 5:");
	cvui::printf(frame, 185, 445, 0.5, 0xc1c1c1, "%d", servoReadPos[4]);
	
	cvui::printf(frame, 20, 515, 0.5, 0xc1c1c1, "SERVO 6:");
	cvui::printf(frame, 185, 515, 0.5, 0xc1c1c1, "%d", servoReadPos[5]);
	
	cvui::printf(frame, 20, 585, 0.5, 0xc1c1c1, "CLAW: ");
	
	// Update cvui stuff and show everything on the screen
	cvui::imshow(WINDOW_NAME, frame);
}

void toggleServo(UsbMX *control, int servoId, int *status, int posX, int posY)
{
	if(*status)
	{
		if (cvui::button(frame, posX, posY, "ON"))
		{
			control->setEndless(servoId, ON);
			*status = 0;
		}
	}
	else
	{
		if (cvui::button(frame, posX, posY, "OFF"))
		{
			control->setEndless(servoId, OFF);
			*status = 1;
		}
	}
}

int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    character = fgetc(stdin);
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

int map ( int x, int in_min, int in_max, int out_min, int out_max) {
    int toReturn =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ;
    
    return toReturn ;
}
