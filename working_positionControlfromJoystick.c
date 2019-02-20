#include <fcntl.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <math.h>
#include <unistd.h>
#include "joystick.h"

#define PI 3.14159265359
#define bool unsigned char
#define true (1)
#define false (0)
#define NODES 3
#define CAN_TIMEOUT 5000

// Bring up socket CAN by typing the following command in the terminal
// $ sudo /sbin/ip link set can0 up type can bitrate 1000000
// the bit rate shuold match the bit rate set on the controller
// to bring down CAN
// $ sudo /sbin/ip link set can0 down

struct nodeDataT
{
	unsigned short statusWord;
	long encoder;
	short current;
	long velocity;
	long currentDemand;
	char displayOperatingMode;
	short ain1;
};

int hnd;
struct nodeDataT nodeData[NODES];
bool CANTimeoutHasOccurred = false;
clock_t time1;

unsigned int canOpenMonitorBus()
{
	int		stat;
	unsigned char node;
	unsigned short index;
	unsigned char subIndex;
	struct sockaddr_can addr;
	socklen_t len = sizeof(addr);
	struct can_frame frame;
	fd_set rfds;
	struct timeval tv;
	FD_ZERO(&rfds);
	FD_SET(hnd, &rfds);
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	frame.can_id = 0;
	stat = select(hnd + 1, &rfds, NULL, NULL, &tv);  
	
	if (stat > 0)
	{
		// Receive message from socket
		recvfrom(hnd, &frame, sizeof(struct can_frame), 0, (struct sockaddr*)&addr, &len);

		node = frame.can_id & 0x7f; 
		
		if ((node >= 1) && (node <= NODES))
		{
			if ((frame.can_id & 0xff80) == 0x580)		// SDO response
			{

				index = frame.data[1] + 256 * frame.data[2];
				subIndex = frame.data[3];
				if ((index == 0x6064) && (subIndex == 0))		// Position actual value, read
				{
					nodeData[node].encoder = frame.data[4] + frame.data[5] * 256 + frame.data[6] * 65536 + frame.data[7] * 16777216;
				}
				if ((index == 0x6041) && (subIndex == 0))		// Read statusword response
				{
					nodeData[node].statusWord = frame.data[4] + frame.data[5] * 256;
				}
				if ((index == 0x2027) && (subIndex == 0))		// Read current response
				{
					nodeData[node].current = frame.data[4] + frame.data[5] * 256;
				}
				if ((index == 0x606c) && (subIndex == 0))		// Read velocity response
				{
					nodeData[node].velocity = frame.data[4] + frame.data[5] * 256 + frame.data[6] * 65536 + frame.data[7] * 16777216;
				}
				if ((index == 0x2031) && (subIndex == 0))		// Read current demand response
				{
					nodeData[node].currentDemand = frame.data[4] + frame.data[5] * 256 + frame.data[6] * 65536 + frame.data[7] * 16777216;
				}
				if ((index == 0x6061) && (subIndex == 0))		// Read Mode of Operation Display
				{
					nodeData[node].displayOperatingMode = frame.data[4];
				}
				if ((index == 0x207c) && (subIndex == 1))		// Read analogue input
				{
					nodeData[node].ain1 = frame.data[4] + frame.data[5] * 256;
				}
			}
			if ((frame.can_id & 0xff80) == 0x180)		// TPDO1
			{
				// nodeData[node].encoder = frame.data[0] + frame.data[1] * 256 + frame.data[2] * 65536 + frame.data[3] * 16777216;
				nodeData[node].current = frame.data[4] + frame.data[5] * 256;
				nodeData[node].statusWord = frame.data[6] + frame.data[7] * 256;
			}
		}
	}
	return frame.can_id;
}

bool canOpenWaitForPacket(unsigned int can_id)
{
	long t = 0;
	while ((canOpenMonitorBus() != can_id) && (++t < CAN_TIMEOUT));
	if (t == CAN_TIMEOUT)
	{	
		printf("CAN Timeout can_id %d, %8ld\n", can_id, clock());
		CANTimeoutHasOccurred = true;
		usleep(2000);
		return false;
	}
	else
	{
		return true;
	}
}

void canWrite(int handle, unsigned int can_id, unsigned char*msg, unsigned char dlc, int timeout)
{
	struct can_frame frame;
	frame.can_id  = can_id;
	frame.can_dlc = dlc;
	frame.data[0] = msg[0];
	frame.data[1] = msg[1];
	frame.data[2] = msg[2];
	frame.data[3] = msg[3];
	frame.data[4] = msg[4];
	frame.data[5] = msg[5];
	frame.data[6] = msg[6];
	frame.data[7] = msg[7];
	write(handle, &frame, sizeof(struct can_frame));  
}


void canOpenWrite(unsigned char node, unsigned int index, unsigned char subIndex, unsigned int data)
{
	unsigned char msg[8] = {0x22, index & 0xff, index >> 8, subIndex, data & 0xff, (data >> 8) & 0xff, (data >> 16) & 0xff, (data >> 24) & 0xff};
	do
	{
		canWrite(hnd, 0x0600 + node, msg, sizeof(msg), 5000);
	}
	while (!canOpenWaitForPacket(0x0580 + node));
}

void canOpenRead(unsigned char node, unsigned int index, unsigned char subIndex)
{
	unsigned char msg[8] = {0x40, index & 0xff, index >> 8, subIndex, 0, 0, 0, 0};
	do
	{
		canWrite(hnd, 0x0600 + node, msg, sizeof(msg), 5000);
	}
	while (!canOpenWaitForPacket(0x0580 + node));		// Need to check it was the right packet
}

void canOpenPreOperational()
{
	unsigned char msg[2] = {0x80, 0x00};
	canWrite(hnd, 0x0000, msg, sizeof(msg), 0);
}

void canOpenStartNodes()
{
	unsigned char msg[2] = {0x01, 0x00};
	canWrite(hnd, 0x0000, msg, sizeof(msg), 0);
}

long positionTarget[NODES];

void setProfilePositionTarget(unsigned char node, double target)
{	

	positionTarget[node] = (long) target;
	canOpenWrite(node, 0x607a, 0, positionTarget[node]);	// Target position, converte position to type (long)
	
	for (int n = 0; n<=10; n++)  //write multiple times to get the value through
	{
	canOpenWrite(node, 0x6040, 0, 0x003f);			// Control word - ebable + new setpoint + change set immediately + absolute position
	// canOpenWrite(node, 0x6040, 0, 0x007f);   		// Control word - endable + new setpoint + change set immediately + relative position
	}
	
	canOpenWrite(node, 0x6040, 0, 0x000f);	//Enable motor

}

// Set controller parameters

void setControllerParameters()
{
	
	for (int nodeId = 1; nodeId <= NODES; nodeId++)
		{
			printf("SETTING PARAMETERS\n");  
			canOpenWrite(nodeId, 0x6410, 2, 1500);		// Output current limit 
			canOpenWrite(nodeId, 0x6098, 0, 0);			// Homing method (0 - no homing operation required)
			canOpenWrite(nodeId, 0x60c5, 0, 100000);	// Max acceleration
			canOpenWrite(nodeId, 0x607f, 0, 5000);		// Max profile velocity

			// OPERATING MODE
			canOpenWrite(nodeId, 0x6060, 0, 0x01);		// Set mode of operation to 1 (profile position mode)
			// canOpenWrite(nodeId, 0x6060, 0, 0x03);		// Set mode of operation to 3 (profile velocity mode)
			// canOpenWrite(nodeId, 0x6060, 0, 0x06);	// Set mode of operation to 6 (homing mode)			
			
			
			canOpenWrite(nodeId, 0x6081, 0, 1000);		// Profile velocity
			canOpenWrite(nodeId, 0x6083, 0, 10000);		// Profile acceleration
			canOpenWrite(nodeId, 0x6084, 0, 10000);		// Profile deceleration
			canOpenWrite(nodeId, 0x6085, 0, 10000);		// Quick stop deceleration
			canOpenWrite(nodeId, 0x6086, 0, 0);			// Motion profile type (0 = trapezoidal profile)

			canOpenWrite(nodeId, 0x2080, 0, 1000);		// Current threshold for homing mode
			canOpenWrite(nodeId, 0x2081, 0, 0);			// Home position
			canOpenWrite(nodeId, 0x2008, 0, 8);			// Miscellaneous configuration - measure slow speeds accurately by timing encoder pulses
			
			canOpenWrite(nodeId, 0x206b, 0, 0);			// Target velocity
			
			// diable motor and clear faults
			// this sets the control world flag to 0x0
			// this is required to overwite previously set values
			canOpenWrite(nodeId, 0x6040, 0, 0x0000);	// Control word - disable motor 
			canOpenWrite(nodeId, 0x6040, 0, 0x0080);	// Control word - clear fault
			// canOpenWrite(nodeId, 0x6040, 0, 0x000f);	// Control word - enable motor
			
			// PID Settings
			// canOpenWrite(nodeId, 0x30A1, 0x01, 446);	// Position controller P gain
			// canOpenWrite(nodeId, 0x30A1, 0x02, 3897);// Position controller I gain
			// canOpenWrite(nodeId, 0x30A1, 0x03, 4.25);// Position controller D gain
			
			canOpenWrite(nodeId, 0x6060, 0, 0x01);		// Set mode of operation to 1 (profile position mode)
			
		}
}

// Joystick functions

static int joystick_fd = -1;

struct joystickState
{ 
	long axis;
	long button;
};

int open_joystick()
{
        joystick_fd = open(JOYSTICK_DEVNAME, O_RDONLY | O_NONBLOCK);
        return joystick_fd;
}

int read_joystick_event(struct js_event *jse)
{
        int bytes;
        bytes = read(joystick_fd, jse, sizeof(*jse)); 
        if (bytes == -1)
				return 0;
        if (bytes == sizeof(*jse))
                return 1;
                
        printf("Unexpected bytes from joystick:%d\n", bytes);
        return -1;
}

void close_joystick()
{
    close(joystick_fd);
	joystick_fd = -1;
}

// Joystick functions ends

int main()
{
	unsigned char nodeId;
	struct sockaddr_can addr;
	struct ifreq ifr;
	char *ifname = "can0";
           
	// Initialise CAN interface
	if ((hnd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		printf("Error while opening socket\n");
		return -1;
	}	
	strcpy(ifr.ifr_name, ifname);
	ioctl(hnd, SIOCGIFINDEX, &ifr);
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex; 
	printf("%s at index %d\n", ifname, ifr.ifr_ifindex);
	if(bind(hnd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		printf("Error in socket bind\n");
		return -2;
	}

	usleep(2000);
	
	canOpenMonitorBus();
	canOpenPreOperational();
	canOpenStartNodes();
	canOpenMonitorBus();	
	
	// Set controller parameters
	setControllerParameters();
	usleep (2000);
	
	// Joystick stuff
	
	int i;
		
	int ps4_a[8] = {0}; //PS4 has 8 axis
	for (i = 0; i < 8; i++)
		ps4_a[i] = 0;
		
	int ps4_b[13]; //PS4 has 13 Buttoms
	for (i = 0; i < 13; i++)
		ps4_b[i] = 0;
		
	struct js_event jse;
	
	joystick_fd = open_joystick();
	printf ("\n Joystick id is %d\n", joystick_fd);
	
	long position_motor[NODES+1] = {0};
		
	while(1)
	 
	{	
	
		canOpenMonitorBus();				
	
		// State machine	
		for (nodeId = 1; nodeId <= NODES; nodeId++)
			{
				// printf("Reading status word\n");
				canOpenRead(nodeId, 0x6041, 0);	// Read statusword
				
				switch (nodeData[nodeId].statusWord & 0x006f)
				{
				case 0x08:	// 8 Fault
					printf("Fault\n");
					canOpenWrite(nodeId, 0x6040, 0, 0x0000);	// Control word - disable motor 
					canOpenWrite(nodeId, 0x6040, 0, 0x0080);	// Control word - clear fault
					break;
				case 0x21:	// 33 Ready to switch on
					printf("Switch on; status = %d\n", nodeData[1].statusWord);
					canOpenWrite(nodeId, 0x6040, 0, 0x0007);	// Control word - enable voltage
					break;
				case 0x23:	// 35 Switched on
					printf("Enable\n");
					canOpenWrite(nodeId, 0x6040, 0, 0x000f);	// Control word - enable motor 
					break;
				case 0x40:	// 64 Switch on disabled
					printf("Switch on disabled\n");
					canOpenWrite(nodeId, 0x6040, 0, 0x000e); 	// Control word - ready to switch on 
					break;
				case 0x27:	// 39/55 Operation enabled
					//printf("Setting to position mode\n");
					//canOpenWrite(nodeId, 0x6060, 0, 0x01);	// Set mode of operation to 1 (profile position mode)
					canOpenWrite(nodeId, 0x6040, 0, 0x000f);	//Enable motor
					break;
				}
			}
				
			// Read joystick
			
			// If Joystick cannot be accessed
			if (access(JOYSTICK_DEVNAME, F_OK) == -1)
			{	
				close_joystick();
				for (i = 0; i < 8; i++)
					ps4_a[i] = 0;
				printf("Cannot reach joystick\n");	
			}	
				
			////Else If joystick is reachable, open joystick
	        else if (access(JOYSTICK_DEVNAME, F_OK) == 0)
			{	
				
			if (joystick_fd == -1)
				open_joystick();        	        

			if (read_joystick_event(&jse) == 1)
			{ 
				//Read buttons
				if ((jse.type == 1) && (jse.number >= 0) && (jse.number < 13))
				{
					ps4_b[jse.number] = jse.value;
				}
				
				//Read axes
				if ((jse.type == 2) && (jse.number >= 0) && (jse.number < 8))
				{
					ps4_a[jse.number] = jse.value;
				}
			}
			printf("%16ld, %8hd, %8hd, %8hd, %8hd", clock(), ps4_a[0], ps4_a[1], ps4_a[3], ps4_a[4]);
			printf("%8hd, %8hd, %8hd, %8hd\n", ps4_b[0], ps4_b[1], ps4_b[2], ps4_b[3]);		
			//printf("%8hd, %8hd, %8hd, %8hd\n", ps4_b[4], ps4_b[5], ps4_b[6], ps4_b[7]);	
				
			}
			
				
			// Set motion 
			
			if (ps4_a[6] > 30000)
			position_motor[1] = position_motor[1]+1000;
			else if (ps4_a[6] < -30000)
			position_motor[1] = position_motor[1]-1000;
				
			if (ps4_a[7] > 30000)
			position_motor[2] = position_motor[2]+1000;
			else if (ps4_a[7] < -30000)
			position_motor[2] = position_motor[2]-1000;
				
			setProfilePositionTarget(1, position_motor[1]);	// Write position target
			setProfilePositionTarget(2, position_motor[2]);	// Write position target
			setProfilePositionTarget(3,10*ps4_a[3]);	// Write position target
			
			//canOpenRead(1, 0x6064, 0); // Read encoder
			//printf("Encoder count position 1: %ld\n", nodeData[1].encoder);	
			//printf("status word 1 = %d\n", nodeData[1].statusWord & 0x006f);
			//printf("press any key to continue 1");
			//getchar();

						
	}  //while loop ends
	
}
