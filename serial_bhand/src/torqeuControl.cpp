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
#include <deque>
#include "BHand.h"
#include "BHandAppHelper.h" // for connection menu
#include <sys/time.h>// Needed for ms time.
#include <time.h>// Needed for ms time.
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
void Initialize()
{
	// Set hardware description before initialization
	int hwIndex = BHandHardware::getBHandHardwareIndex("BH8-262");
	//int hwIndex = BHandHardware::getBHandHardwareIndex("BH8-280");
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

int Execute()
{
	printf("Execute: Press Any Key to Abort...");


	if (result = bh.TorqueClose("123"))
		Error();
	if (UnbufferedGetChar() != EOF)
		return 1;

	if (result = bh.TorqueOpen("123"))
		Error();
	if (UnbufferedGetChar() != EOF)
		return 1;

	if (result = bh.Command("1FGET MOV", buf))
		Error();
	if (UnbufferedGetChar() != EOF)
		return 1;

	if (result = bh.Command("2FGET MOV", buf))
		Error();
	if (UnbufferedGetChar() != EOF)
		return 1;

	if (result = bh.Command( "3FGET MOV", buf ))
		Error();
	if (UnbufferedGetChar() != EOF)
		return 1;

	if (result = bh.Command( "SFGET MOV", buf ))
		Error();
	if (UnbufferedGetChar() != EOF)
		return 1;

	if (result = bh.Command("1FGET MCV", buf))
		Error();
	if (UnbufferedGetChar() != EOF)
		return 1;

	if (result = bh.Command("2FGET MCV", buf))
		Error();
	if (UnbufferedGetChar() != EOF)
		return 1;

	if (result = bh.Command( "3FGET MCV", buf ))
		Error();
	if (UnbufferedGetChar() != EOF)
		return 1;

	if (result = bh.Command( "SFGET MCV", buf ))
		Error();
	if (UnbufferedGetChar() != EOF)
		return 1;

	if (result = bh.Command("1FGET MCV", buf))
		Error();
	if (UnbufferedGetChar() != EOF)
		return 1;

	if (result = bh.Command("2FGET MCV", buf))
		Error();
	if (UnbufferedGetChar() != EOF)
		return 1;

	if (result = bh.Command( "3FGET MCV", buf ))
		Error();
	if (UnbufferedGetChar() != EOF)
		return 1;

	if (result = bh.Command( "SFGET MCV", buf ))
		Error();
	if (UnbufferedGetChar() != EOF)
		return 1;

	if (result = bh.Set("1", "MCV", 20))
		Error();
	if (UnbufferedGetChar() != EOF)
		return 1;

	if (result = bh.Set("2", "MCV", 20))
		Error();
	if (UnbufferedGetChar() != EOF)
		return 1;

	if (result = bh.Set("3", "MCV", 20))
		Error();
	if (UnbufferedGetChar() != EOF)
		return 1;

	if (result = bh.Set("4", "MCV", 60))
		Error();
	if (UnbufferedGetChar() != EOF)
		return 1;

	if (result = bh.Close("123"))
		Error();
	if (UnbufferedGetChar() != EOF)
		return 1;

	return 0;
}

int main()
{
	/* Turn off output buffering for printf() */
	setvbuf(stdout, 0, _IONBF, 0);

	/* Turn on unbuffered input */
	UnbufferedInputStart();

	printf("\n\n\r\t\tInitializing Software...");
	Initialize();
	printf("Executing - ");
	Execute();
	printf(" Done\n");

	/* Turn off unbuffered input */
	UnbufferedInputEnd();
	return 0;
}
