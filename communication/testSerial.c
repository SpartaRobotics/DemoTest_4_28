/* 

    AUTHOR: Bruce Nelson
    DATE: 4/5/2017

    CONTRIBUTIONS: Chrisheyd 
    www.chrisheydrick.com

    PURPOSE:
    * The purpose of this program to demostrate a simple example of reading
    and writing data from a linux system to an Arduino. The program setups
    the serial stream at a selected baud rate (matched with the Arduino's)
    and initializes communication. The program sends a single character string
    "2" to the Arduino. The Arduino will convert it to an integer add 4 to the
    value read, which should add up to 6. This value will then write from the
    Arduino to the host.

*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>

int fd;

void serialSetup(char *portStr); // Sets up serial communication to the Arduino
void clientStart(char *msg);
void transmitReceive(char *msg, int size);
void clientMessage(char* msg, int size); // prints a char array
void missionPhase(char readSignal, char *writeSignal, char *startMsg, char *endMsg);

int main(int argc, char *argv[])
{
    int n;

	// USB: "/dev/ttyACM0"
	// SERIAL: "/dev/ttyTHS0"
    serialSetup("/dev/ttyTHS0");

	// CAPTURE PHASE
	missionPhase('1', "2", "HOST: START CAPTURE", "HOST: END CAPTURE");
	
	// DOCKING PHASE
	missionPhase('3', "4", "HOST: START DOCKING", "HOST: END DOCKING");
	
	// REFUELING PHASE
	missionPhase('5', "6", "HOST: START REFUELING", "HOST: END REFUELING");
	
	// ARM RESET PHASE
	missionPhase('7', "8", "HOST: START ARM RESET", "HOST: END ARM RESET");

	// MISSION DONE
	printf("MISSION ACCOMPLISHED!\n\n");

	close(fd);
	
    return 0;
}


void serialSetup(char *portStr)
{
    struct termios toptions;

    /* open serial port */
    fd = open(portStr, O_RDWR | O_NOCTTY);
    printf("fd opened as %i\n\n", fd);
    
    /* wait for the Arduino to reboot */
    usleep(3500000);
    
    /* get current serial port settings */
    tcgetattr(fd, &toptions);
    /* set 115200 baud both ways */
    cfsetispeed(&toptions, B115200);
    cfsetospeed(&toptions, B115200);
    /* 8 bits, no parity, no stop bits */
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    /* Canonical mode */
    toptions.c_lflag |= ICANON;
    /* commit the serial port settings */
    tcsetattr(fd, TCSANOW, &toptions);
}

void missionPhase(char readSignal, char *writeSignal, char *startMsg, char *endMsg)
{
	int n;
    char buf[8] = "empty";
    
	while( buf[0] != readSignal)
    {
		n = read(fd, buf,8);
		buf[n] = '\0';
	}
	printf("%s\n", startMsg);
	
	// Do mission phase stuff here
	
	write(fd, writeSignal, 1 );
	printf("%s\n", endMsg);
}

/*
void clientMessage(char *msg, int size)
{
	int i;
	
    printf("CLIENT %d: ", size);
    for(i = 0; i < size; i++)
    {
        printf("%c", msg[i]);
    }
}
*/

/*
    int n;
    char buf[64] = "temp text";

    serialSetup();
   
    while( buf[0] != '1')
    {
		n = read(fd, buf,64);
		buf[n] = '\0';
	}
	printf("HOST: START CAPTURE \n");
	write(fd, "2", 1 );
	printf("HOST: END CAPTURE \n");
	
	while( buf[0] != '3')
    {
		n = read(fd, buf,64);
		buf[n] = '\0';
	}
	printf("HOST: START DOCKING \n");
	write(fd, "4", 1);
	printf("HOST: END DOCKING \n");
	
	while( buf[0] != '5')
	{
		n = read(fd, buf, 64);
		buf[n] = '\0';
	}
	printf("HOST: START REFUELING \n");
	write(fd, "6", 1);
	printf("HOST: END REFUELING \n");
	
	while( buf[0] != '7')
	{
		n = read(fd, buf, 64);
		buf[n] = '\0';
	}
	printf("HOST: START ARM RESET \n");
	write(fd, "8", 1);
	printf("HOST: END ARM RESET \n");

	close(fd);
	
 */
