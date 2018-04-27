/*
Capstone
Capstone Project Code
12/3/2017

Authors: Bruce Nelson
*/

// Inlucde the standard and opencv libraries
#include<opencv2/objdetect/objdetect.hpp> // object-detection
#include<opencv2/highgui/highgui.hpp>     // UI interface
#include<opencv2/imgproc/imgproc.hpp>     // Image processing
#include<iostream>                        // standard library
#include<time.h>
#include <sys/time.h>

using namespace cv;
using namespace std;

#define WIDTH 640
#define HEIGHT 480
#define DEV 0
#define PORTD 128 // Port diameter in mm
#define REFXMM 130
#define REFYMM 130
#define REFZMM 600 // MM of camera distance
#define REFXPP 200
#define REFYPP 200
#define PMM 0.65 // Pixels per MM
#define DEBUG 0


// Convert FPS from integer to ostream to be display on frame
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

// Creates and store the cascade dataset for the license ports
std::string port_cascade_file = "port.xml";
CascadeClassifier port_cascade;

// stores current frame
Mat frame;

// Detects and recognizes the object from the cascade file
void detection(Mat frame)
{
    std::vector<Rect> ports; // Creates a vector for license ports
    Mat gray, blurPlate;
    int midPortX = 0;
    int midPortY = 0;
    float distance;
    

    cvtColor(frame, gray, COLOR_BGR2GRAY); 
    port_cascade.detectMultiScale(gray, ports, 1.1, 5);

    for( size_t i = 0; i< ports.size(); i++ ) 
    {
        rectangle(frame, ports[i], Scalar( 0, 0, 255 ), 3, 1 );
        midPortX = ports[i].x + (ports[i].width / 2);

        putText(frame, "X Dist: " + SSTR(float(int(midPortX-WIDTH/2)*0.65)) + " mm", Point(0,30), FONT_HERSHEY_SIMPLEX, 1.00, Scalar(0,140,255), 2);
        line(frame, Point(WIDTH/2,HEIGHT/2), Point(midPortX, HEIGHT/2), Scalar( 0, 255, 0), 3);

        midPortY = ports[i].y + (ports[i].height / 2);

        putText(frame, "Y Dist: " + SSTR(float(int(-midPortY+HEIGHT/2)*0.65)) + " mm", Point(0,60), FONT_HERSHEY_SIMPLEX, 1.00, Scalar(0,140,255), 2);
        line(frame, Point(WIDTH/2,HEIGHT/2), Point(WIDTH/2, midPortY), Scalar( 0, 255, 0), 3);

        line(frame, Point(WIDTH/2,HEIGHT/2), Point(midPortX, midPortY), Scalar( 255, 255, 0), 3);

        Point center(midPortX, midPortY);
        circle( frame, center, 3, Scalar(0,0,255), -1, 8, 0 );

        distance = float(REFYPP / float(ports[i].height)) * REFZMM;
        putText(frame, "Z Dist: " + SSTR(distance) + " mm", Point(0,90), FONT_HERSHEY_SIMPLEX, 1.00, Scalar(0,140,255), 2);
        
    }

    imshow("Port", frame); // Display the current frame
}


// Main - handles the video processing and capturing of frames
int main()
{

    if(!port_cascade.load(port_cascade_file))
    {
        std::cout << "Error! Could not load cascade file!" << std::endl;
        return -1;
    }

    VideoCapture cap;
    cap.open(DEV);     // opens a live video on the selected video device
    
    std::string VideoFileName = "";
    
    if(!cap.isOpened())
    {
        std::cout << "ERROR! No video found!" << std::endl;
        return -1;
    }

    //namedWindow("Port", WINDOW_AUTOSIZE);

    double fps = cap.get(CV_CAP_PROP_FPS); // gets frames per second

    Size size( // gets size of video capture device
        cap.get(CV_CAP_PROP_FRAME_WIDTH),
        cap.get(CV_CAP_PROP_FRAME_HEIGHT)
    );

    Mat gray;
    char quitProg; // handles user input for exiting the program
    Point center(cvRound(WIDTH/2), cvRound(HEIGHT/2));
    int elapsed;
    struct timeval st, et;

    while(1) // loops through the video until it ends or the user exits
    {
		gettimeofday(&st,NULL);
		
        cap >> frame; // stores the frame from the video 
        //if(frame.empty()) break; // if no frame received, exit program

		if(DEBUG)
		{
			circle( frame, center, 3, Scalar(0,255,0), -1, 8, 0 );
			detection(frame); // detects objects in the current frame

			cout << "FPS: " << cap.get(CV_CAP_PROP_FPS) << endl; 

			//If user enters a 'q' or ESC key, then exit the video
			imshow("Port", frame);
		}
		gettimeofday(&et,NULL);
        quitProg = waitKey(1);
        if(quitProg == 'q' || quitProg == 27)
            break;
            
        
        elapsed = ((et.tv_sec - st.tv_sec) + (et.tv_usec - st.tv_usec)) /1000;
		cout << "FPS: " <<  int(1000/elapsed) << " frames" << endl;
    }

    cap.release();       // closes the video file or capturing device
    destroyAllWindows(); // destroy current windows

    return 0;
}
