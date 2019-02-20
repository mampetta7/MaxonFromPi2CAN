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
#define NODES 12
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
	long home_offset;
	long position;  //position of the motor after applying home_offset
	short current;
	long velocity;
	long currentDemand;
	char displayOperatingMode;
	short ain1;
};

int hnd;
struct nodeDataT nodeData[NODES];
bool CANTimeoutHasOccurred = false;
bool HomingDone = false;
clock_t time1;
// int nodeId;

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
				nodeData[node].encoder = frame.data[0] + frame.data[1] * 256 + frame.data[2] * 65536 + frame.data[3] * 16777216;
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


void setControllerParameters()
{
	
	for (int nodeId = 1; nodeId <= NODES; nodeId++)
		{
			printf("SETTING PARAMETERS\n");  
			
			// diable motor and clear faults
			canOpenWrite(nodeId, 0x6040, 0, 0x0000);	// Control word - clear 
			canOpenWrite(nodeId, 0x6040, 0, 0x0007);	// Control word - disable motor (0111)
			
			//canOpenWrite(nodeId, 0x6410, 2, 200);		// Output current limit 
			canOpenWrite(nodeId, 0x60c5, 0, 100000);	// Max acceleration
			canOpenWrite(nodeId, 0x607f, 0, 5000);		// Max profile velocity

			// OPERATING MODE
			// canOpenWrite(nodeId, 0x6060, 0, 0x01);	// Set mode of operation to 1 (profile position mode)
			// canOpenWrite(nodeId, 0x6060, 0, 0x03);	// Set mode of operation to 3 (profile velocity mode)
			// canOpenWrite(nodeId, 0x6060, 0, 0x06);	// Set mode of operation to 6 (homing mode)		
			// canOpenWrite(nodeId, 0x6060, 0, 0x0a);   // Set mode of ooeration to 10 (cyclic synchronous torque mode) 	
			
			canOpenWrite(nodeId, 0x6081, 0, 1000);		// Profile velocity
			canOpenWrite(nodeId, 0x6083, 0, 10000);		// Profile acceleration
			canOpenWrite(nodeId, 0x6084, 0, 10000);		// Profile deceleration
			canOpenWrite(nodeId, 0x6085, 0, 10000);		// Quick stop deceleration
			canOpenWrite(nodeId, 0x6086, 0, 0);			// Motion profile type (0 = trapezoidal profile)
			
			// PID Settings
			// canOpenWrite(nodeId, 0x30A1, 0x01, 446);	// Position controller P gain
			// canOpenWrite(nodeId, 0x30A1, 0x02, 3897);// Position controller I gain
			// canOpenWrite(nodeId, 0x30A1, 0x03, 4.25);// Position controller D gain
			
		}
}
	
int enableMotors()
{
	// State machine	
	for (int nodeId = 1; nodeId <= NODES; nodeId++)
		{
			
			canOpenRead(nodeId, 0x6041, 0);	// Read statusword
			// Continue to initialize motors until all nodes are operational
			// Status word for operational is 0x27 (39)
			while((nodeData[nodeId].statusWord & 0x006f) != 0x27)
			{	
				canOpenRead(nodeId, 0x6041, 0);	// Read statusword
				switch (nodeData[nodeId].statusWord & 0x006f)
					{
					case 0x08:	// 8 Fault
						printf("Fault: Node %d\n", nodeId);
						canOpenWrite(nodeId, 0x6040, 0, 0x0000);	// Control word - disable motor 
						canOpenWrite(nodeId, 0x6040, 0, 0x0080);	// Control word - clear fault
						break;
					case 0x21:	// 33 Ready to switch on
						printf("Ready to switch on: Node %d\n", nodeId);
						canOpenWrite(nodeId, 0x6040, 0, 0x0007);	// Control word - enable voltage
						break;
					case 0x23:	// 35 Switched on
						printf("Enable: Node %d\n", nodeId);
						canOpenWrite(nodeId, 0x6040, 0, 0x000f);	// Control word - enable motor (enable operation) 
						break;
					case 0x40:	// 64 Switch on disabled
						printf("Switch on disabled: Node %d\n", nodeId);
						canOpenWrite(nodeId, 0x6040, 0, 0x000e); 	// Control word - ready to switch on
						// canOpenWrite(nodeId, 0x6040, 0, 0x000f);	//Enable motor 
						break;
					case 0x27:	// 39 Operation enabled
						printf("Opertation enabled: Node %d\n", nodeId);
						canOpenWrite(nodeId, 0x6040, 0, 0x000f);	//Enable motor
						break;
					}
			}
		}	
		
	return 1;
}

int applyPretension(int pretension_current, long int pretension_velocity)
{
	// specify current is in mAmps, velocity in rpm
	
	for (int nodeId = 1; nodeId <= NODES; nodeId++)
		{
			canOpenWrite(nodeId, 0x6060, 0, 0x0a);   // Set to current mode (10)	
			canOpenWrite(nodeId, 0x6080, 0, pretension_velocity); // max speed in current mode
			
			canOpenRead(nodeId, 0x6041, 0);	// Read statusword
			
			if ((nodeData[nodeId].statusWord & 0x006f) != 0x27)
				enableMotors();
			

			if(nodeId % 2 == 0)
				canOpenWrite(nodeId, 0x6071, 0, -pretension_current);  // apply current
			else
				canOpenWrite(nodeId, 0x6071, 0, pretension_current);  // apply current
		}	
	
	usleep(200000);
	
	// Continue with pre-tension as long as the motors are moving.
	long int system_velocity;
	while (1)  
	{
		system_velocity = 0;
		for (int nodeId = 1; nodeId <= NODES; nodeId++)
		{
			canOpenRead(nodeId, 0x606c, 0);	// Read velocity
			printf("%ld : Node %d Velocity: %ld, StatusWord: %d\n", clock(), nodeId, nodeData[nodeId].velocity, nodeData[nodeId].statusWord);
			system_velocity = system_velocity + nodeData[nodeId].velocity;
		}
		
		if (system_velocity == 0)
		{
			printf ("Pretension done!!!\n");
			usleep(200000);
			break;
		}
	}
	return 1;
}

void setHomelocation()
{
	for (int nodeId = 1; nodeId <= NODES; nodeId++)
	{
		canOpenRead(nodeId, 0x6064, 0); // Read encoder
		nodeData[nodeId].home_offset = nodeData[nodeId].encoder;
		printf("Node %d, home_offset %ld\n", nodeId, nodeData[nodeId].home_offset);
	}
	HomingDone = true;
	printf("Homing done\n");
}

void getPosition(int nodeId)
{
	canOpenRead(nodeId, 0x6064, 0); // Read encoder position
	if (HomingDone)
	nodeData[nodeId].position = nodeData[nodeId].encoder - nodeData[nodeId].home_offset;
	else
	nodeData[nodeId].position = nodeData[nodeId].encoder;
	 
}

int main(int argc, char *argv[])
{ 
	int nodeId;
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

	
	// If there is an argument input disable motors and exit
	if (argc == 2)
	{
		printf("Arument received. Shutting down...:(\n");
		for (nodeId = 1; nodeId <= NODES; nodeId++)
		{
			canOpenWrite(nodeId, 0x6040, 0, 0x0000);
		}
		usleep(200000);
		exit(0);
	}
	
	usleep(2000);
	
	// SET CONTROLLER PARAMETERS
	setControllerParameters();
	usleep (20000);
	
	enableMotors(); // turn on the motors and bring to operating state
	
	// applyPretension(current, speed). Current in mAmps. Speed in rpm
	applyPretension(200, 500);
	usleep(200000);
	
	printf("Press any key to set home position. \n");
	getchar();
	
	setHomelocation();
	printf ("Homing done. Press any key to continue.\n");
	getchar();
	
	FILE *logfile = fopen("log.txt", "w");
	
	int count = 0;
		
	while(1) 
	{	
				
		count = count + 1;	
		for (nodeId = 1; nodeId <= NODES; nodeId++)
		{
			getPosition(nodeId);
			//canOpenRead(nodeId, 0x6064, 0); // Read encoder
			printf("%ld,", nodeData[nodeId].position);
			fprintf(logfile, "%ld,", nodeData[nodeId].position);
			usleep(2000);
		}
		
		fprintf(logfile,"\n");
			
		getchar();		
		
		if (count == 32)
		{
			fclose(logfile);
			break;
		}
					
	}  //while loop ends
	
}
