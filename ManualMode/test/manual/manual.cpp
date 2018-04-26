// Testing Manual Control!

#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <armadillo>
#include "UsbMX.h"
#include "JHPWMPCA9685.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <math.h>

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
#define ID2_MIN 200
#define ID3_MIN 940
#define ID4_MIN 515
#define ID5_MIN 610
#define ID6_MIN 1665

#define ID1_MAX 1000
#define ID2_MAX 1062
#define ID3_MAX 3300
#define ID4_MAX 3160
#define ID5_MAX 3260
#define ID6_MAX 3715

#define ID1_RST 314
#define ID2_RST 795
#define ID3_RST 2235
#define ID4_RST 1810
#define ID5_RST 1945
#define ID6_RST 646

#define MOVE_OFFSET 2048 
#define MULTI_OFFSET 6144

#define DSPEED 5 // Default Speed

using namespace std;
using namespace cv;
using namespace arma;

double getSgn(double x);
mat sixDofJ(cube Abi);
cube getAbe(vec q);
vec rad2enc(mat q);
cube getAbeach(vec q);

VideoCapture clawCam;
cv::Mat frame;
char quit;

void displayGUI(UsbMX *control, int *servoThres, int *servoCon, 
				int *servoReadPos, int *moveStatus, int *baseCoords,
				int *clawCoords, int *controlPos);
void toggleServo(UsbMX *control, int servoId, int *status, int posX, int posY);
int getkey();
int map ( int x, int in_min, int in_max, int out_min, int out_max);
void drControl(double *Obe, int *controlPos);

int main(int argc, const char *argv[])
{
	frame = cv::Mat(800, 1250, CV_8UC3);
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
	
	// Positions from control algorithm
	int controlPos[6] = {0,0,0,0,0,0};
	
	// Indicates if the servos will move
	int moveStatus[6] = {0,0,0,0,0,0};
	
	// Stores Base camera cartesian coordinates
	int baseCoords[3] = {0,0,0};
	
	// Stores Claw camera cartesian coordinates
	int clawCoords[3] = {0,0,0};
	
	
	//VideoCapture clawCam;
	clawCam.open(0);
	cv::Mat clawFrame;
	
	VideoCapture baseCam;/*
	baseCam.open(1);
	Mat baseFrame;
	*/
	cv::Mat currentFrame;
	
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
				   moveStatus, baseCoords, clawCoords, controlPos);
		
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

/****************************GUI*************************************/


void displayGUI(UsbMX *control, int *servoThres, int *servoCon, 
				int *servoReadPos, int *moveStatus, int *baseCoords,
				int *clawCoords, int *controlPos)
{
	
	// Fill the frame with a nice color
	frame = cv::Scalar(49, 52, 49);
	
	// For mission stages
	cvui::window(frame, 15, 20, 440, 260,
	"                         CONTROL");
	
	// For controlling all servos
	cvui::window(frame, 470, 50, 380, 185, 
	"                   UTILITY INTERFACE");
	
	// For machine vision
	cvui::window(frame, 860, 15, 380, 220,
	 "                  MACHINE VISION INTERFACE");
	
	if (cvui::button(frame, 600, 190, " EXIT PROGRAM ")) {
		quit = 'q';
	}
	
	if (cvui::button(frame, 665, 140, "GO TO RESET POSITION")) {
		servoThres[0] = ID1_RST;
		servoThres[1] = ID2_RST;
		servoThres[2] = ID3_RST;
		servoThres[3] = ID4_RST;
		servoThres[4] = ID5_RST;
		servoThres[5] = ID6_RST;
	}
	
	if (cvui::button(frame, 665, 80, "GO TO READ POSITIONS")) {
		for(int i = 0; i < 6; i++)
		{
			servoThres[i] = servoReadPos[i];
		}
	}
	
	if (cvui::button(frame, 475, 80, "POWER OFF ALL SERVOS")) {
		memset(moveStatus, 0, 6*sizeof(int) );
		for(int i = 1; i < 7; i++)
		{
			control->setEndless(i, ON);
		}
	}
	
	if (cvui::button(frame, 475, 140, "POWER ON ALL SERVOS ")) {
		memset(moveStatus, 1, 6*sizeof(int) );
		for(int i = 1; i < 7; i++)
		{
			control->setEndless(i, OFF);
			servoThres[i-1] = servoReadPos[i-1];
		}
	}
	
	if (cvui::button(frame, 940, 750, "CONCURRENT MOVE")) {
		for(int i = 0; i < 6; i++)
		{
			servoThres[i] = servoCon[i];
		}
	}
	
	if (cvui::button(frame, 880, 40, "TOGGLE BASE CAM")) {
		clawCam.release();
		clawCam.open(1);
	}
	
	if (cvui::button(frame, 1070, 40, "TOGGLE CLAW CAM")) {
		//clawCam.open(0);
	}
	
	if (cvui::button(frame, 30, 50, "Calculate Servo Values")) {
		double Obe[3];
		copy(baseCoords, baseCoords + 3, Obe);
		drControl(Obe, controlPos);
	}
	
	if (cvui::button(frame, 230, 50, "GO TO CONTROL POSITIONS")) {
		for(int i = 0; i < 6; i++)
		{
				servoThres[i] = controlPos[i];
		}
	}
	
	// Control algorithm Test
	cvui::printf(frame, 20, 90, 0.5, 0xc1c1c1, "X: %d mm", baseCoords[0]);
	cvui::printf(frame, 20, 140, 0.5, 0xc1c1c1, "Y: %d mm", baseCoords[1]);
	cvui::printf(frame, 20, 190, 0.5, 0xc1c1c1, "Z: %d mm", baseCoords[2]);
	
	cvui::printf(frame, 50, 230, 0.5, 0xc1c1c1, "ID1: %d", controlPos[0]);
	cvui::printf(frame, 200, 230, 0.5, 0xc1c1c1, "ID2: %d", controlPos[1]);
	cvui::printf(frame, 350, 230, 0.5, 0xc1c1c1, "ID3: %d", controlPos[2]);
	cvui::printf(frame, 50, 260, 0.5, 0xc1c1c1, "ID4: %d", controlPos[3]);
	cvui::printf(frame, 200, 260, 0.5, 0xc1c1c1, "ID5: %d", controlPos[4]);
	cvui::printf(frame, 350, 260, 0.5, 0xc1c1c1, "ID6: %d", controlPos[5]);
	
	cvui::counter(frame, 120, 90, &baseCoords[0]);
	cvui::counter(frame, 120, 140, &baseCoords[1]);
	cvui::counter(frame, 120, 190, &baseCoords[2]);
	
	cvui::trackbar(frame, 210, 80, 240, &baseCoords[0], -150, 350);
	cvui::trackbar(frame, 210, 130, 240, &baseCoords[1], -150, 350);
	cvui::trackbar(frame, 210, 180, 240, &baseCoords[2], -150, 350);
	
	// Toggles the servos on and off
	toggleServo(control, 1, &moveStatus[0], 105, 335);
	toggleServo(control, 2, &moveStatus[1], 105, 405);
	toggleServo(control, 3, &moveStatus[2], 105, 475);
	toggleServo(control, 4, &moveStatus[3], 105, 545);
	toggleServo(control, 5, &moveStatus[4], 105, 615);
	toggleServo(control, 6, &moveStatus[5], 105, 685);
	
	if (cvui::button(frame, 80, 755, "OPEN")) {
		servoThres[6] = CLW_OPN;
	}
	
	if (cvui::button(frame, 155, 755, "CLOSE")) {
		servoThres[6] = CLW_CLS;
	}

	// Single servo preceision movement
	cvui::counter(frame, 240, 337, &servoThres[0]);
	cvui::counter(frame, 240, 407, &servoThres[1]);
	cvui::counter(frame, 240, 477, &servoThres[2]);
	cvui::counter(frame, 240, 547, &servoThres[3]);
	cvui::counter(frame, 240, 617, &servoThres[4]);
	cvui::counter(frame, 240, 687, &servoThres[5]);
	
	// Single Servo Movement
	cvui::trackbar(frame, 360, 325, 400, &servoThres[0], 0, 1000);
	cvui::trackbar(frame, 360, 395, 400, &servoThres[1], 200, 1062);
	cvui::trackbar(frame, 360, 465, 400, &servoThres[2], 940, 3300);
	cvui::trackbar(frame, 360, 535, 400, &servoThres[3], 515, 3160);
	cvui::trackbar(frame, 360, 605, 400, &servoThres[4], 610, 3260);
	cvui::trackbar(frame, 360, 675, 400, &servoThres[5], 1665, 3715);
	cvui::trackbar(frame, 360, 745, 400, &servoThres[6], 120, 420); // Claw EE
	
	// Multi Servo Movement
	cvui::trackbar(frame, 810, 325, 400, &servoCon[0], 0, 1000);
	cvui::trackbar(frame, 810, 395, 400, &servoCon[1], 200, 1062);
	cvui::trackbar(frame, 810, 465, 400, &servoCon[2], 940, 3300);
	cvui::trackbar(frame, 810, 535, 400, &servoCon[3], 515, 3160);
	cvui::trackbar(frame, 810, 605, 400, &servoCon[4], 610, 3260);
	cvui::trackbar(frame, 810, 675, 400, &servoCon[5], 1665, 3715);

	cvui::printf(frame, 525, 15, 0.8, 0xc1c1c1, "SPARTA RM INTERFACE");
	
	cvui::printf(frame, 930, 80, 0.5, 0xc1c1c1, "BASE");
	cvui::printf(frame, 900, 100, 0.5, 0xc1c1c1, "COORDINATES");
	cvui::printf(frame, 905, 135, 0.5, 0xc1c1c1, "X: %d mm", baseCoords[0]);
	cvui::printf(frame, 905, 170, 0.5, 0xc1c1c1, "Y: %d mm", baseCoords[1]);
	cvui::printf(frame, 905, 205, 0.5, 0xc1c1c1, "Z: %d mm", baseCoords[2]);
	
	cvui::printf(frame, 1120, 80, 0.5, 0xc1c1c1, "CLAW");
	cvui::printf(frame, 1090, 100, 0.5, 0xc1c1c1, "COORDINATES");
	cvui::printf(frame, 1095, 135, 0.5, 0xc1c1c1, "X: %d mm", clawCoords[0]);
	cvui::printf(frame, 1095, 170, 0.5, 0xc1c1c1, "Y: %d mm", clawCoords[1]);
	cvui::printf(frame, 1095, 205, 0.5, 0xc1c1c1, "Z: %d mm", clawCoords[2]);
	
	cvui::printf(frame, 860, 300, 0.6, 0xc1c1c1, "CONCURRENT SERVO MOVEMENT");
	cvui::printf(frame, 450, 300, 0.6, 0xc1c1c1, "SINGLE SERVO MOVEMENT");
	cvui::printf(frame, 105, 290, 0.4, 0xc1c1c1, "TOGGLE");
	cvui::printf(frame, 105, 310, 0.4, 0xc1c1c1, "ON/OFF");
	cvui::printf(frame, 170, 290, 0.4, 0xc1c1c1, "CURRENT");
	cvui::printf(frame, 170, 310, 0.4, 0xc1c1c1, "POSITION");
	cvui::printf(frame, 255, 290, 0.4, 0xc1c1c1, "MOVEMENT");
	cvui::printf(frame, 267, 310, 0.4, 0xc1c1c1, "(FINE)");
	
	cvui::printf(frame, 20, 340, 0.5, 0xc1c1c1, "SERVO 1:");
	cvui::printf(frame, 185, 340, 0.5, 0xc1c1c1, "%d", servoReadPos[0]);
	
	cvui::printf(frame, 20, 410, 0.5, 0xc1c1c1, "SERVO 2:");
	cvui::printf(frame, 185, 410, 0.5, 0xc1c1c1, "%d", servoReadPos[1]);
	
	cvui::printf(frame, 20, 480, 0.5, 0xc1c1c1, "SERVO 3:");
	cvui::printf(frame, 185, 480, 0.5, 0xc1c1c1, "%d", servoReadPos[2]);
	
	cvui::printf(frame, 20, 550, 0.5, 0xc1c1c1, "SERVO 4:");
	cvui::printf(frame, 185, 550, 0.5, 0xc1c1c1, "%d", servoReadPos[3]);
	
	cvui::printf(frame, 20, 620, 0.5, 0xc1c1c1, "SERVO 5:");
	cvui::printf(frame, 185, 620, 0.5, 0xc1c1c1, "%d", servoReadPos[4]);
	
	cvui::printf(frame, 20, 690, 0.5, 0xc1c1c1, "SERVO 6:");
	cvui::printf(frame, 185, 690, 0.5, 0xc1c1c1, "%d", servoReadPos[5]);
	
	cvui::printf(frame, 20, 760, 0.5, 0xc1c1c1, "CLAW: ");
	
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

/******************************** CONTROL CODE ************************/
void drControl(double *Obe, int *controlPos)
{
    struct timeval start,stop;
    gettimeofday(&start,NULL);

	double offset, dt, eta_d, eta, qi, q2pi, mag;

	int a, i, attempts = 10000, reach=0, phase = 1;
	double lambda = .1;
	    
	vec serv(6,1),q(6,1),sinq(6,1),cosq(6,1),qd(6,1),qdeg(6,1),init(6,1),
		obe_d(3,1),phi_d(3,1),obe(3,1),dobe_d(3,1),ocp(3,1),oce(3,1),
	    oep(3,1),eps_d(3,1),eps(3,1),ep(3,1),eo(3,1),obc(3,1),o_ct(3,1),
	    oc_d(3,1),oep_b(3,1),csi_d(6,1),err(6,1),G(6,1),app(3,1),fkOut(7,1);
	    
	mat  K(6,6), 
	     Rz1(3,3),
	     Ry(3,3),
	     Rz2(3,3),
	     Rd(3,3),
	     Jg(6,6),
	     Abe(4,4),
	     posErr(1, attempts),
	     oriErr(1, attempts),
	     Rbe(3,3),
	     Rp(3,3),
	     Rt(3,3),
	     Rs(3,3),
	     Rbe_d(3,3);
	   
	     
	cube Abi(4,4,8);

 while(reach == 0)
  {	      
	   if (phase == 1) //Docking
	   {
	    
	    obe_d(0) = Obe[0];
	    obe_d(1) = Obe[1];
	    obe_d(2) = Obe[2];
	    
	    app << 0 << endr
	        << 0 << endr
	        << 50 << endr;
	    obe_d += app;
	   
	  
	    // Want Z straigt down & Y away from center of Target//
	    Rbe_d << 0 << 1 << 0 << endr
	          << 1 << 0 << 0 << endr
	          << 0 << 0 << -1 << endr;
	   }
	   else if (phase == 2) //Refueling
	   {
	   }
	          
	// Arbitrary Start. Will read this from servos //
	q << 1.2781 << endr // desired should be +.0027
	  << 1.4653 << endr // should be - for MATLAB
	  << .0261 << endr
	  << .8408 << endr
	  << 1.4837 << endr
	  << 3.0411 << endr;
	  
	dt = 0.3; 
	K = eye(6,6);
	K(1,1) = 2;
	K(2,2) = 2;
	K(3,3) = 2;
	
    phi_d << M_PI << endr
          << 0 << endr
          << 0 << endr;
	
	eta_d = 0.5*sqrt(Rbe_d(0,0)+Rbe_d(1,1)+Rbe_d(2,2)+1);
    eps_d << 0.5*getSgn(Rbe_d(2,1)-Rbe_d(1,2))*sqrt(Rbe_d(0,0)-Rbe_d(1,1)-Rbe_d(2,2)+1) << endr
        << 0.5*getSgn(Rbe_d(0,2)-Rbe_d(2,0))*sqrt(Rbe_d(1,1)-Rbe_d(0,0)-Rbe_d(2,2)+1) << endr
	    << 0.5*getSgn(Rbe_d(1,0)-Rbe_d(0,1))*sqrt(Rbe_d(2,2)-Rbe_d(0,0)-Rbe_d(1,1)+1) << endr;
	
	/* Vector Initialization */
	posErr.zeros(1,attempts);
	oriErr.zeros(1,attempts);
	eps.zeros(3,1);

	/* Jacobian */
	for(a=0; a<attempts; a++)
	{
		// Abi = getAbeach(q);
		Abi = getAbe(q);

		/* sub q into Abe for FK */
		Abe = Abi.slice(7);
		obe = Abe.submat(0,3,2,3);
        
		/* Convert Rbe to quaternion */
		eta = 0.5*sqrt(Abe(0,0)+Abe(1,1)+Abe(2,2)+1);
        eps << 0.5*getSgn(Abe(2,1)-Abe(1,2))*sqrt(Abe(0,0)-Abe(1,1)-Abe(2,2)+1) << endr
            << 0.5*getSgn(Abe(0,2)-Abe(2,0))*sqrt(Abe(1,1)-Abe(0,0)-Abe(2,2)+1) << endr
            << 0.5*getSgn(Abe(1,0)-Abe(0,1))*sqrt(Abe(2,2)-Abe(0,0)-Abe(1,1)+1) << endr;
        //cout << eta << endl;
        //eps.print("eps: ");
        
        /* calculate error */
        if (phase == 1) 
        {
		    ep = obe_d - obe;
		}
		else if (phase == 2)
		{
		    o_ct = trans(Rbe)*(obe_d-obe)-oce; //Check this math
		    ep = Rbe*(oce + o_ct);
		}
		eo = -(eta_d*eps) + (eta*eps_d) - cross(eps_d,eps);
		posErr(0,a) = sqrt(ep(0,0)*ep(0,0)+ep(1,0)*ep(1,0)+ep(2,0)*ep(2,0)); 
		oriErr(0,a) = sqrt(eo(0,0)*eo(0,0)+eo(1,0)*eo(1,0)+eo(2,0)*eo(2,0));

		/* determine when error is acceptable */

		if((posErr(0,a) < 10e-2) && (oriErr(0,a) < 10e-2))
		{
			//cout << "Attempts: "<< a << endl;
			q(1) = -q(1); //joint 2 reversed (flip for MATLAB sim)
			reach = 1;
			for(i=0; i<6; i++)
			{
			    // Convert (q>pi) --> (-pi<q<pi) //
			    q2pi = q(i)/(2*M_PI); 
			    qi = int(q2pi);
			    qd(i) = q2pi-qi;
			    if (qd(i) <= 0)
			    { 
			        mag = abs(qd(i));
			        if (mag > .5)
			        {
			            qd(i) = 2*M_PI*(1-mag);
			        }
			        else
		   	        {
			            qd(i) = 2*M_PI*qd(i);
			        }
			    }
			    else
			    { 
			        if (qd(i) > .5)
			        {
			            qd(i) = -2*M_PI*(1-qd(i));
			        }
			        else
		   	        {
			            qd(i) = 2*M_PI*qd(i);
			        }
			    }
			}
			//qd.print("qd: ");
			
			/////////rad -> deg////////////////
			qdeg = qd * 180 / M_PI;
			posErr = posErr.submat(0,0, 0,a);
			oriErr = oriErr.submat(0,0, 0,a);
			
			/////// q -> encoder values/////
			serv = (qd*651.7395);
			init << 1987 << endr
			     << 2293 << endr // subject to slip! 
			     << 2230 << endr
			     << 1810 << endr
			     << 1950 << endr
			     << 2430 << endr;
			
			for(i = 0; i < 6; i++)
			{
			    serv(i) = int(serv(i) + init(i)); //Joint offset!!!
			    if (serv(i) < 0)
			    {
			        serv(i) = serv(i) + 4095;
			    }
			}
		
			//serv.print("Encoder Values: ");
			//cout << endl << serv(0) << endl;
			for (i=0; i<6; i++)
			{
			    controlPos[i] = serv(i);
			}
	
		    if (serv(2) < 940  || serv(2) > 3330)
		    {
		        cout << "JOINT 3 O.B.!!!" << endl;
		        break;
		        reach = 0;
		    }
		    if (serv(3) < 515 || serv(3) > 3160)
		    {
		        cout << "JOINT 4 O.B.!!!" << endl;
		        break;
		        reach = 0;
		    }
		    if (serv(4) < 600 || serv(4) > 3260)
		    {
		        cout << "JOINT 5 O.B.!!!" << endl;
		        break;
		        reach = 0;
		    }
		    if (serv(5) < 1665 || serv(5) > 3715)
		    {
		        cout << "JOINT 6 O.B.!!!" << endl;
		        break;
		        reach = 0;
		    }
		    if (reach)
		    {
			   /* control.move(2, int(serv(2)));
			    usleep(2000000);
			    control.move(3, int(serv(3)));
			    usleep(2000000);*/
			}
		   
			break;
		}
                      
		/* Iterate q using Jg */
		Jg = sixDofJ(Abi); 

		    for(i=0; i<3; i++)
		    {
			    err(i,0)   = ep(i,0);
			    err(i+3,0) = eo(i,0);		
		    }
		    //err.print("err: ");
		    
		    q = q + dt*solve((Jg+eye(6,6)*lambda),K*err);
		   for(i=0; i<6; i++)
		   {
		        sinq(i) = sin(q(i));
		        cosq(i) = cos(q(i));
		        q(i) = atan2(sinq(i),cosq(i));
           }
    
     if(a == attempts)
   	 {
	    cout << "Out of Workspace for Desired Orientation" << endl;
	 }
        }
  }
   gettimeofday(&stop,NULL);
   double secs;
   secs = (double)(stop.tv_usec-start.tv_usec)/1000000 + (double)(stop.tv_sec-start.tv_sec);
   //printf("time: %f\n",secs);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/* sgn Function */
double getSgn(double x)
{ 
	if(x >= 0)
		return 1;
	else
		return -1;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Jacobian Function */
mat sixDofJ(cube Abi)
{
	int j;
	vec B(3,1), Oe(3,1), Oj(3,1), E(3,1);
	mat A(4,4), C(4,4), D(4,4), Jg(6,6);
	Jg.zeros(6,6);
	for(j = 0; j < 6; j = j + 1)
	{
		A = Abi.slice(j);
		B = A.submat(0,2, 2,2);

		C = Abi.slice(7);
		Oe = C.submat(0,3, 2,3);

		D = Abi.slice(j);
		Oj = D.submat(0,3, 2,3);

		E = cross(B, (Oe-Oj));

		Jg.submat(0,j, 2,j) = E;
		Jg.submat(3,j, 5,j) = B;
	}
	return Jg;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
cube getAbe(vec q)
{
	/* Generates transformation matrices between joint frames of a manipulator
	% 'i' refers to index of matrix depth (:,:,i)
	%  Aim1_i: aka each A from (i-1) frame to (i) for i = 1:n
	%    A01, A12, A13, ... A(n-1)n
	%  Ab_im1:   aka each A from base frame to (i-1) for i = [1:n, e]
	%    Ab0, Ab1, Ab2, ... Abn, Abe */
	mat d(6,1),theta(6,1),a(6,1),alpha(6,1);
	mat DHTable(6,4), A6e(4,4);
	cube Aim1_i(4,4,6), Abi(4,4,8);
	double thet,D,A,a1,alph,L1,L2,L3,L4,L5,n,i,j;
    L1 = 128.8;
    L2 = 194.8;
    L3 = 118.5 + 30.0;
    L4 = 118.5;
    L5 = 261.5;
    a1 = 0; // actually 1.8573cm
    
    DHTable << L1     << -M_PI+q(0)      <<   0.0 << M_PI/2  << endr
		    << a1     << q(1) + M_PI/2   <<   L2  << -M_PI/2 << endr 
		    <<    0   << q(2)            <<   L3  << M_PI/2  << endr
		    <<    0   << q(3)            <<   L4  << 0       << endr
		    <<    0   << q(4) + M_PI/2.0 <<   0   << M_PI/2  << endr 
		    <<   L5   << q(5)            <<   0   << 0       << endr;

	/* Separate DH table into columns */

	d     = DHTable.submat(0,0, 5,0);
	theta = DHTable.submat(0,1, 5,1);
	a     = DHTable.submat(0,2, 5,2);
	alpha = DHTable.submat(0,3, 5,3);

	/* n: number of joints */
	n = 6;

	/* Preallocate matrices */
	Aim1_i.zeros(4,4,6);
	Abi.zeros(4,4,8);

	/* Calculate each A from (i-1) to (i) */
	for (i = 0; i < 6; i++)
	{
	 	thet = theta(i,0); //'operator=' error w/out these
		alph = alpha(i,0);
		A = a(i,0);
		D = d(i,0); 
	    Aim1_i.slice(i) << cos(thet) << -sin(thet)*cos(alph) <<  sin(thet)*sin(alph) << A*cos(thet) << endr
 			            << sin(thet) <<  cos(thet)*cos(alph) << -cos(thet)*sin(alph) << A*sin(thet) << endr
 			            << 0.0 << sin(alph) << cos(alph) << D   << endr
 			            << 0.0 << 0.0       << 0.0       << 1.0 << endr;
	}

	// Calculate each Ab(i-1)
	Abi.slice(0)  = eye(4,4);
	A6e = eye(4,4);

	for (j = 0; j < 6; j++)
	{
		Abi.slice(j+1) = Abi.slice(j) * Aim1_i.slice(j);
	}
	Abi.slice(n+1) = Abi.slice(n) * A6e;
	return Abi;
}
//////////////////////////////////////////////////////////////////////////////////
cube getAbeach(vec q)
{
// Jonathan Buchholz' auto function
// Sparta Robotics FK function of q
// Last updated: 4/23/2018

// Joint position vector
float  q1,  q2,  q3,  q4,  q5,  q6;
cube Abeach_q(4,4,8);
q1 = q(0);
q2 = q(1);
q3 = q(2);
q4 = q(3);
q5 = q(4);
q6 = q(5);

// Abeach_q: transformation from base to each frame
Abeach_q.zeros(4,4,8);

Abeach_q.slice(0)  << 1.0 << 0 << 0 << 0 << endr
 << 0 << 1.0 << 0 << 0 << endr
 << 0 << 0 << 1.0 << 0 << endr
 << 0 << 0 << 0 << 1.0 << endr;

Abeach_q.slice(1)  << -1.0*cos(q1) << 0 << -1.0*sin(q1) << 0 << endr
 << -1.0*sin(q1) << 0 << cos(q1) << 0 << endr
 << 0 << 1.0 << 0 << 12.880000000000000004440892098501 << endr
 << 0 << 0 << 0 << 1.0 << endr;

Abeach_q.slice(2)  << cos(q1)*sin(q2) << sin(q1) << cos(q1)*cos(q2) << 19.479999999999999982236431605997*cos(q1)*sin(q2) << endr
 << sin(q1)*sin(q2) << -1.0*cos(q1) << cos(q2)*sin(q1) << 19.479999999999999982236431605997*sin(q1)*sin(q2) << endr
 << cos(q2) << 0 << -1.0*sin(q2) << 19.479999999999999982236431605997*cos(q2) + 12.880000000000000004440892098501 << endr
 << 0 << 0 << 0 << 1.0 << endr;

Abeach_q.slice(3)  << sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2) << cos(q1)*cos(q2) << cos(q1)*sin(q2)*sin(q3) - 1.0*cos(q3)*sin(q1) << 19.479999999999999982236431605997*cos(q1)*sin(q2) + 11.850000000000000005551115123126*sin(q1)*sin(q3) + 11.850000000000000005551115123126*cos(q1)*cos(q3)*sin(q2) << endr
 << cos(q3)*sin(q1)*sin(q2) - 1.0*cos(q1)*sin(q3) << cos(q2)*sin(q1) << cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3) << 19.479999999999999982236431605997*sin(q1)*sin(q2) - 11.850000000000000005551115123126*cos(q1)*sin(q3) + 11.850000000000000005551115123126*cos(q3)*sin(q1)*sin(q2) << endr
 << cos(q2)*cos(q3) << -1.0*sin(q2) << cos(q2)*sin(q3) << 19.479999999999999982236431605997*cos(q2) + 11.850000000000000005551115123126*cos(q2)*cos(q3) + 12.880000000000000004440892098501 << endr
 << 0 << 0 << 0 << 1.0 << endr;

Abeach_q.slice(4)  << cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*sin(q4) << cos(q1)*cos(q2)*cos(q4) - 1.0*sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) << cos(q1)*sin(q2)*sin(q3) - 1.0*cos(q3)*sin(q1) << 19.479999999999999982236431605997*cos(q1)*sin(q2) + 11.850000000000000005551115123126*sin(q1)*sin(q3) + 11.850000000000000005551115123126*cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + 11.850000000000000005551115123126*cos(q1)*cos(q3)*sin(q2) + 11.850000000000000005551115123126*cos(q1)*cos(q2)*sin(q4) << endr
 << cos(q2)*sin(q1)*sin(q4) - 1.0*cos(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) << sin(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) + cos(q2)*cos(q4)*sin(q1) << cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3) << 19.479999999999999982236431605997*sin(q1)*sin(q2) - 11.850000000000000005551115123126*cos(q1)*sin(q3) - 11.850000000000000005551115123126*cos(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) + 11.850000000000000005551115123126*cos(q3)*sin(q1)*sin(q2) + 11.850000000000000005551115123126*cos(q2)*sin(q1)*sin(q4) << endr
 << cos(q2)*cos(q3)*cos(q4) - 1.0*sin(q2)*sin(q4) << - 1.0*cos(q4)*sin(q2) - 1.0*cos(q2)*cos(q3)*sin(q4) << cos(q2)*sin(q3) << 19.479999999999999982236431605997*cos(q2) + 11.850000000000000005551115123126*cos(q2)*cos(q3) - 11.850000000000000005551115123126*sin(q2)*sin(q4) + 11.850000000000000005551115123126*cos(q2)*cos(q3)*cos(q4) + 12.880000000000000004440892098501 << endr
 << 0 << 0 << 0 << 1.0 << endr;

Abeach_q.slice(5)  << - 1.0*sin(q5)*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*sin(q4)) - 1.0*cos(q5)*(sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - 1.0*cos(q1)*cos(q2)*cos(q4)) << cos(q1)*sin(q2)*sin(q3) - 1.0*cos(q3)*sin(q1) << cos(q5)*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*sin(q4)) - 1.0*sin(q5)*(sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - 1.0*cos(q1)*cos(q2)*cos(q4)) << 19.479999999999999982236431605997*cos(q1)*sin(q2) + 11.850000000000000005551115123126*sin(q1)*sin(q3) + 11.850000000000000005551115123126*cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + 11.850000000000000005551115123126*cos(q1)*cos(q3)*sin(q2) + 11.850000000000000005551115123126*cos(q1)*cos(q2)*sin(q4) << endr
 << cos(q5)*(sin(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) + cos(q2)*cos(q4)*sin(q1)) + sin(q5)*(cos(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) - 1.0*cos(q2)*sin(q1)*sin(q4)) << cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3) << sin(q5)*(sin(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) + cos(q2)*cos(q4)*sin(q1)) - 1.0*cos(q5)*(cos(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) - 1.0*cos(q2)*sin(q1)*sin(q4)) << 19.479999999999999982236431605997*sin(q1)*sin(q2) - 11.850000000000000005551115123126*cos(q1)*sin(q3) - 11.850000000000000005551115123126*cos(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) + 11.850000000000000005551115123126*cos(q3)*sin(q1)*sin(q2) + 11.850000000000000005551115123126*cos(q2)*sin(q1)*sin(q4) << endr
 << sin(q5)*(sin(q2)*sin(q4) - 1.0*cos(q2)*cos(q3)*cos(q4)) - 1.0*cos(q5)*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)) << cos(q2)*sin(q3) << - 1.0*sin(q5)*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)) - 1.0*cos(q5)*(sin(q2)*sin(q4) - 1.0*cos(q2)*cos(q3)*cos(q4)) << 19.479999999999999982236431605997*cos(q2) + 11.850000000000000005551115123126*cos(q2)*cos(q3) - 11.850000000000000005551115123126*sin(q2)*sin(q4) + 11.850000000000000005551115123126*cos(q2)*cos(q3)*cos(q4) + 12.880000000000000004440892098501 << endr
 << 0 << 0 << 0 << 1.0 << endr;

Abeach_q.slice(6)  << - 1.0*cos(q6)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*sin(q4)) + cos(q5)*(sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - 1.0*cos(q1)*cos(q2)*cos(q4))) - 1.0*sin(q6)*(cos(q3)*sin(q1) - 1.0*cos(q1)*sin(q2)*sin(q3)) << sin(q6)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*sin(q4)) + cos(q5)*(sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - 1.0*cos(q1)*cos(q2)*cos(q4))) - 1.0*cos(q6)*(cos(q3)*sin(q1) - 1.0*cos(q1)*sin(q2)*sin(q3)) << cos(q5)*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*sin(q4)) - 1.0*sin(q5)*(sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - 1.0*cos(q1)*cos(q2)*cos(q4)) << 19.479999999999999982236431605997*cos(q1)*sin(q2) + 11.850000000000000005551115123126*sin(q1)*sin(q3) + 26.150000000000000022204460492503*cos(q5)*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*sin(q4)) - 26.150000000000000022204460492503*sin(q5)*(sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - 1.0*cos(q1)*cos(q2)*cos(q4)) + 11.850000000000000005551115123126*cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + 11.850000000000000005551115123126*cos(q1)*cos(q3)*sin(q2) + 11.850000000000000005551115123126*cos(q1)*cos(q2)*sin(q4) << endr
 << sin(q6)*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) + cos(q2)*cos(q4)*sin(q1)) + sin(q5)*(cos(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) - 1.0*cos(q2)*sin(q1)*sin(q4))) << cos(q6)*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)) - 1.0*sin(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) + cos(q2)*cos(q4)*sin(q1)) + sin(q5)*(cos(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) - 1.0*cos(q2)*sin(q1)*sin(q4))) << sin(q5)*(sin(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) + cos(q2)*cos(q4)*sin(q1)) - 1.0*cos(q5)*(cos(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) - 1.0*cos(q2)*sin(q1)*sin(q4)) << 19.479999999999999982236431605997*sin(q1)*sin(q2) - 11.850000000000000005551115123126*cos(q1)*sin(q3) + 26.150000000000000022204460492503*sin(q5)*(sin(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) + cos(q2)*cos(q4)*sin(q1)) - 26.150000000000000022204460492503*cos(q5)*(cos(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) - 1.0*cos(q2)*sin(q1)*sin(q4)) - 11.850000000000000005551115123126*cos(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) + 11.850000000000000005551115123126*cos(q3)*sin(q1)*sin(q2) + 11.850000000000000005551115123126*cos(q2)*sin(q1)*sin(q4) << endr
 << cos(q2)*sin(q3)*sin(q6) - 1.0*cos(q6)*(cos(q5)*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)) - 1.0*sin(q5)*(sin(q2)*sin(q4) - 1.0*cos(q2)*cos(q3)*cos(q4))) << sin(q6)*(cos(q5)*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)) - 1.0*sin(q5)*(sin(q2)*sin(q4) - 1.0*cos(q2)*cos(q3)*cos(q4))) + cos(q2)*cos(q6)*sin(q3) << - 1.0*sin(q5)*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)) - 1.0*cos(q5)*(sin(q2)*sin(q4) - 1.0*cos(q2)*cos(q3)*cos(q4)) << 19.479999999999999982236431605997*cos(q2) + 11.850000000000000005551115123126*cos(q2)*cos(q3) - 11.850000000000000005551115123126*sin(q2)*sin(q4) - 26.150000000000000022204460492503*sin(q5)*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)) - 26.150000000000000022204460492503*cos(q5)*(sin(q2)*sin(q4) - 1.0*cos(q2)*cos(q3)*cos(q4)) + 11.850000000000000005551115123126*cos(q2)*cos(q3)*cos(q4) + 12.880000000000000004440892098501 << endr
 << 0 << 0 << 0 << 1.0 << endr;

Abeach_q.slice(7)  << - 1.0*cos(q6)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*sin(q4)) + cos(q5)*(sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - 1.0*cos(q1)*cos(q2)*cos(q4))) - 1.0*sin(q6)*(cos(q3)*sin(q1) - 1.0*cos(q1)*sin(q2)*sin(q3)) << sin(q6)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*sin(q4)) + cos(q5)*(sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - 1.0*cos(q1)*cos(q2)*cos(q4))) - 1.0*cos(q6)*(cos(q3)*sin(q1) - 1.0*cos(q1)*sin(q2)*sin(q3)) << cos(q5)*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*sin(q4)) - 1.0*sin(q5)*(sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - 1.0*cos(q1)*cos(q2)*cos(q4)) << 19.479999999999999982236431605997*cos(q1)*sin(q2) + 11.850000000000000005551115123126*sin(q1)*sin(q3) + 26.150000000000000022204460492503*cos(q5)*(cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*sin(q4)) - 26.150000000000000022204460492503*sin(q5)*(sin(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - 1.0*cos(q1)*cos(q2)*cos(q4)) + 11.850000000000000005551115123126*cos(q4)*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + 11.850000000000000005551115123126*cos(q1)*cos(q3)*sin(q2) + 11.850000000000000005551115123126*cos(q1)*cos(q2)*sin(q4) << endr
 << sin(q6)*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) + cos(q2)*cos(q4)*sin(q1)) + sin(q5)*(cos(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) - 1.0*cos(q2)*sin(q1)*sin(q4))) << cos(q6)*(cos(q1)*cos(q3) + sin(q1)*sin(q2)*sin(q3)) - 1.0*sin(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) + cos(q2)*cos(q4)*sin(q1)) + sin(q5)*(cos(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) - 1.0*cos(q2)*sin(q1)*sin(q4))) << sin(q5)*(sin(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) + cos(q2)*cos(q4)*sin(q1)) - 1.0*cos(q5)*(cos(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) - 1.0*cos(q2)*sin(q1)*sin(q4)) << 19.479999999999999982236431605997*sin(q1)*sin(q2) - 11.850000000000000005551115123126*cos(q1)*sin(q3) + 26.150000000000000022204460492503*sin(q5)*(sin(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) + cos(q2)*cos(q4)*sin(q1)) - 26.150000000000000022204460492503*cos(q5)*(cos(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) - 1.0*cos(q2)*sin(q1)*sin(q4)) - 11.850000000000000005551115123126*cos(q4)*(cos(q1)*sin(q3) - 1.0*cos(q3)*sin(q1)*sin(q2)) + 11.850000000000000005551115123126*cos(q3)*sin(q1)*sin(q2) + 11.850000000000000005551115123126*cos(q2)*sin(q1)*sin(q4) << endr
 << cos(q2)*sin(q3)*sin(q6) - 1.0*cos(q6)*(cos(q5)*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)) - 1.0*sin(q5)*(sin(q2)*sin(q4) - 1.0*cos(q2)*cos(q3)*cos(q4))) << sin(q6)*(cos(q5)*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)) - 1.0*sin(q5)*(sin(q2)*sin(q4) - 1.0*cos(q2)*cos(q3)*cos(q4))) + cos(q2)*cos(q6)*sin(q3) << - 1.0*sin(q5)*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)) - 1.0*cos(q5)*(sin(q2)*sin(q4) - 1.0*cos(q2)*cos(q3)*cos(q4)) << 19.479999999999999982236431605997*cos(q2) + 11.850000000000000005551115123126*cos(q2)*cos(q3) - 11.850000000000000005551115123126*sin(q2)*sin(q4) - 26.150000000000000022204460492503*sin(q5)*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)) - 26.150000000000000022204460492503*cos(q5)*(sin(q2)*sin(q4) - 1.0*cos(q2)*cos(q3)*cos(q4)) + 11.850000000000000005551115123126*cos(q2)*cos(q3)*cos(q4) + 12.880000000000000004440892098501 << endr
 << 0 << 0 << 0 << 1.0 << endr;

return Abeach_q;
}
