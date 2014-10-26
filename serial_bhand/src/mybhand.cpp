///////////////////////////////////////////////////////////
//                                                       //
//           Automatically Generated C++ Code            //
//           BHand Control Center Version 1.0            //
//                                                       //
//                                                       //
//                                                       //
///////////////////////////////////////////////////////////


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <deque>
#include "BHand.h"
#include "BHandAppHelper.h" // for connection menu
#include <sys/time.h>// Needed for ms time.
#include <time.h>// Needed for ms time.
// ROS include
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
// my ros
#include "serial_bhand/State.h"
#include "serial_bhand/Order.h"
#include "serial_bhand/Action.h"
using namespace std;

BHand  bh;           // Handles all hand communication

char   buf[100];     // Buffer for reading back hand parameters
int    value;        // Some commands use an int* instead of 
                     // a buffer to relay information

int    result;       // Return value (error) of all BHand calls


///////////////////////////////////////////////////////////
// Error Handler - called whenever result != 0           //
// Feel free to customize this function                  //
///////////////////////////////////////////////////////////

void Error()
{
	printf("ERROR: %d\n%s\n", result, bh.ErrorMessage(result));
	exit(0);
}


// get the difference in time between two timevals (difference in ms)
double diffclock(timeval* currentTime, timeval* startTime)
{
	if (((currentTime->tv_usec)/1000. - (startTime->tv_usec)/1000.) < 0.0) {
		return ((1000.0 + (currentTime->tv_usec)/1000.) - (startTime->tv_usec)/1000.);
	} else {
		return ((currentTime->tv_usec)/1000. - (startTime->tv_usec)/1000.);
	}
}


///////////////////////////////////////////////////////////
//  Initialize hand, set timeouts and baud rate          //
///////////////////////////////////////////////////////////
void Initialize(char *hand_version)
{
	// Set hardware description before initialization
	// hand_version: 262: BH8-262
	//               280: BH8-280
	char ver[256];
	sprintf(ver, "BH8-%s", hand_version);
	printf("%s is initiarize ...\n", ver);
	int hwIndex = BHandHardware::getBHandHardwareIndex(ver);
	if (hwIndex < 0)
	{
		printf("\n\nThe API has not been compiled to include target hand.\n");
			Error();
	}
	bh.setHardwareDesc(hwIndex);
	bool use280Config = (strcmp(bh.getHardwareDesc()->getModelNumber(), "BH8-280") == 0);
	//printf("\nuse280Config = %d\n", use280Config);
	if (result = handInitWithMenu(&bh))
		Error();
	//if (result = bh.InitSoftware(com_port, THREAD_PRIORITY_TIME_CRITICAL))
	//	Error();

	printf("Initialization..."); 
	if (result = bh.InitHand(""))
		Error();
	else
		printf(" Done\n");
}


///////////////////////////////////////////////////////////
//  Loads a file to be run in realtime mode              //
///////////////////////////////////////////////////////////
void LoadFile(char *toLoad, deque<float*> *file_data)
{

	FILE* fp = fopen(toLoad, "rt");
	if (!fp)
		return;

	int v1,v2,v3,v4;

	while (fscanf(fp, "%d %d %d %d", &v1, &v2, &v3, &v4) != EOF)
{
		float* values = new float[4];
		values[0] = v1;
		values[1] = v2;
		values[2] = v3;
		values[3] = v4;


		file_data->push_back(values);

	}

	fclose(fp);
}


///////////////////////////////////////////////////////////
//  Execute commands, return 1 if interrupted with a key //
///////////////////////////////////////////////////////////

int HandOpen()
{
	
	printf("Action -> Open...\n");

	if (result = bh.Open("123"))
		Error();
	if (UnbufferedGetChar() != EOF)
		return 1;

	return 0;
}

int HandClose()
{
	printf("Action -> Close...\n");

	if (result = bh.Close("123"))
			Error(); 
	if (UnbufferedGetChar() != EOF)
			return 1; 
	return 0;
}

bool bhandService(serial_bhand::Action::Request &req,
				serial_bhand::Action::Response &res)
{
		if (req.order == req.INITIALIZE) {/*{{{*/ // Initialize
				ROS_INFO("Initialize...");
				if (result = bh.InitHand("")){
						res.result = false;
						Error();
				}
				else
						ROS_INFO("Initialeze Success!!");
				res.result = true;
		}/*}}}*/
		else if (req.order == req.OPEN) {/*{{{*/ // Hand Open
				ROS_INFO("Hand Open...");
				if (result = bh.Open("123")){
						res.result = false;
						Error();
				}
				if (UnbufferedGetChar() != EOF){
						res.result = false;
						return false;
				}
				res.result = true;
		}/*}}}*/
		else if (req.order == req.CLOSE) {/*{{{*/ // Hand Close
				ROS_INFO("Hand Close...");
				if (result = bh.Close("123")){
						res.result = false;
						Error(); 
				}
				if (UnbufferedGetChar() != EOF){
						res.result = false;
						return false; 
				}
				res.result = true;
		}/*}}}*/
		else if (req.order == req.T_OPEN){ /*{{{*/ // Hand Torque Open
				ROS_INFO("Hand Torque Open");
				if (result = bh.TorqueOpen("123")) {
						res.result = false;
						Error();
				}
				if (UnbufferedGetChar() != EOF){
						res.result = false;
						return false; 
				}
				res.result = true;
		} /*}}}*/
		else if (req.order == req.T_CLOSE){/*{{{*/ // Hand Torque Close
				ROS_INFO("Hand Torque Close");
				if (result = bh.TorqueClose("123")) {
						res.result = false;
						Error();
				}
				if (UnbufferedGetChar() != EOF){
						res.result = false;
						return false; 
				}
				res.result = true;
		}/*}}}*/
		else if (req.order == req.S_OPEN){/*{{{*/ // Spread Open
				ROS_INFO("Spread Open");
				if (result = bh.Command("SO")) {
						res.result = false;
						Error();
				}
				if (UnbufferedGetChar() != EOF){
						res.result = false;
						return false; 
				}
				res.result = true;
		}/*}}}*/
		else if (req.order == req.S_CLOSE){/*{{{*/ // Spread Close
				ROS_INFO("Spread Close");
				if (result = bh.Command("SC")) {
						res.result = false;
						Error();
				}
				if (UnbufferedGetChar() != EOF){
						res.result = false;
						return false; 
				}
				res.result = true;
		}/*}}}*/
		else if (req.order == req.COMMAND){/*{{{*/ // Another Commands
                char buf[100];
				ROS_INFO("COMMAND MODE");
				if (result = bh.Command(req.s_order.c_str(), buf)) {
						res.result = false;
						Error();
				}
				if (UnbufferedGetChar() != EOF){
						res.result = false;
						return false; 
				}
				res.result = true;
		}/*}}}*/

		return true;
}


int main(int argc, char **argv)
{
	if (argc != 2) {
		cout << "1 argument required!!" << endl;
		exit(0);
	}
	// // ros setting
	// init node
	ros::init(argc, argv, "bhand");
	// Publish or Subscribe topic
	ros::NodeHandle n;
	ros::Publisher state_pub = n.advertise<serial_bhand::State>("bhand_state", 1000);
	// ros::Subscriber order_sub = n.advertise<serial_bhand::Order>("bhand_order", 1000);
	//
	// Service server
	ros::ServiceServer service = n.advertiseService("bhand_action", bhandService);

	// 
	ros::Rate loop_rate(0.5);

	/* Turn off output buffering for printf() */
	setvbuf(stdout, 0, _IONBF, 0);


	/* Turn on unbuffered input */
	UnbufferedInputStart();

	// Initialize Hand
	serial_bhand::State state;
	std::string str;
	str = "INIT_NO";
	state.stateInfo = str;
	state.state = serial_bhand::State::INIT_NO;
	state_pub.publish(state);
	printf("\n\n\r\t\tInitializing Software...");
	Initialize(argv[1]);
	state.state = serial_bhand::State::INIT_OK;
	str = "INIT_OK";
	state.stateInfo = str;
	state_pub.publish(state);

	ros::spin();

	/* Turn off unbuffered input */
	UnbufferedInputEnd();
	return 0;
}
