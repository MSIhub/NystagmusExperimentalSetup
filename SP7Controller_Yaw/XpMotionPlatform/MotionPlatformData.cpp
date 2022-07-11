// Downloaded from https://developer.x-plane.com/code-sample/motionplatformdata/

/*
Version 1.0.0.4			Pilot
Version 1.0.0.3			Changed order of data in the frame
Version 1.0.0.2			Changed broadcast to hosto destination (improves compatibility)
Version 1.0.0.1			Intitial
*/

#include <stdio.h>
#include <string.h>
#include <winsock2.h>

#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMProcessing.h"
#include "XPLMDataAccess.h"

#define UDP_PORT 0x5850
#define NUM_DATA 19 // nc1
#define NUM_LINES 21
// Globals.
// Use MPD_ as a prefix for the global variables

// Used to store data for display
char MPD_Buffer[NUM_LINES][80];
// Used to store calculated motion data
float MPD_MotionData[NUM_DATA];

// Window ID
XPLMWindowID MPD_Window = NULL;

// Datarefs
XPLMDataRef	MPD_DR_t = NULL;
XPLMDataRef	MPD_DR_groundspeed = NULL;
XPLMDataRef	MPD_DR_m_total = NULL;
XPLMDataRef	MPD_DR_the = NULL;
XPLMDataRef	MPD_DR_psi = NULL;
XPLMDataRef	MPD_DR_phi = NULL;
XPLMDataRef	MPD_DR_ax = NULL;
XPLMDataRef	MPD_DR_ay = NULL;
XPLMDataRef	MPD_DR_az = NULL;
XPLMDataRef	MPD_DR_p = NULL;
XPLMDataRef	MPD_DR_q = NULL;
XPLMDataRef	MPD_DR_r = NULL;
XPLMDataRef	MPD_PH_x = NULL;
XPLMDataRef	MPD_PH_y = NULL;
XPLMDataRef	MPD_PH_z = NULL;
XPLMDataRef	MPD_PH_psi = NULL;
XPLMDataRef	MPD_PH_theta = NULL;
XPLMDataRef	MPD_PH_phi = NULL;
XPLMDataRef	MPD_Zulu = NULL; // nc2 (new lines)

double lasttime;
SOCKET sock;
struct sockaddr_in server;
FILE *fp;

//---------------------------------------------------------------------------
// Function prototypes

float MotionPlatformDataLoopCB(float elapsedMe, float elapsedSim, int counter, void * refcon);

void MotionPlatformDataDrawWindowCallback(
                                   XPLMWindowID         inWindowID,    
                                   void *               inRefcon);    

void MotionPlatformDataHandleKeyCallback(
                                   XPLMWindowID         inWindowID,    
                                   char                 inKey,    
                                   XPLMKeyFlags         inFlags,    
                                   char                 inVirtualKey,    
                                   void *               inRefcon,    
                                   int                  losingFocus);    

int MotionPlatformDataHandleMouseClickCallback(
                                   XPLMWindowID         inWindowID,    
                                   int                  x,    
                                   int                  y,    
                                   XPLMMouseStatus      inMouse,    
                                   void *               inRefcon);    

float MPD_fallout(float data, float low, float high);
float MPD_fltlim(float data, float min, float max);
float MPD_fltmax2 (float x1,const float x2);
void MPD_CalculateMotionData(void);

//---------------------------------------------------------------------------
// SDK Mandatory Callbacks

PLUGIN_API int XPluginStart(
						char *		outName,
						char *		outSig,
						char *		outDesc)
{
	strcpy(outName, "XP-SP7");
	strcpy(outSig, "it.swhard.xpsp7");
	strcpy(outDesc, "A plug-in that exports relevant motion data for SP7 platform.");

	MPD_Window = XPLMCreateWindow(
		                      50, 600, 200, 200,								/* Area of the window. */
                              1,												/* Start visible. */
                              MotionPlatformDataDrawWindowCallback,			/* Callbacks */
                              MotionPlatformDataHandleKeyCallback,
                              MotionPlatformDataHandleMouseClickCallback,
                              NULL);											/* Refcon - not used. */

	XPLMRegisterFlightLoopCallback(MotionPlatformDataLoopCB, 1.0, NULL);
	
	MPD_DR_t = XPLMFindDataRef("sim/network/misc/network_time_sec");
	MPD_DR_groundspeed = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
	MPD_DR_m_total = XPLMFindDataRef("sim/flightmodel/weight/m_total");
	MPD_DR_phi = XPLMFindDataRef("Sim/flightmodel/position/true_phi"); // roll --> true_phi
	MPD_DR_the = XPLMFindDataRef("Sim/flightmodel/position/true_theta"); // pitch -->true_theta
	MPD_DR_psi = XPLMFindDataRef("Sim/flightmodel/position/true_psi"); // yaw --> true_psi
	MPD_DR_ax = XPLMFindDataRef("sim/flightmodel/position/local_ax"); // ax
	MPD_DR_ay = XPLMFindDataRef("sim/flightmodel/position/local_ay"); // ay
	MPD_DR_az = XPLMFindDataRef("sim/flightmodel/position/local_az"); // az
	MPD_DR_p = XPLMFindDataRef("sim/flightmodel/position/P"); // vroll
	MPD_DR_q = XPLMFindDataRef("sim/flightmodel/position/Q"); // vpitch
	MPD_DR_r = XPLMFindDataRef("sim/flightmodel/position/R"); // vyaw
	MPD_PH_x = XPLMFindDataRef("sim/graphics/view/pilots_head_x"); // pilot head x
	MPD_PH_y = XPLMFindDataRef("sim/graphics/view/pilots_head_y"); // pilot head y
	MPD_PH_z = XPLMFindDataRef("sim/graphics/view/pilots_head_z"); // pilot head z
	MPD_PH_phi = XPLMFindDataRef("sim/graphics/view/pilots_head_phi"); // pilot head roll
	MPD_PH_theta = XPLMFindDataRef("sim/graphics/view/pilots_head_theta"); // pilot head pitch
	MPD_PH_psi = XPLMFindDataRef("sim/graphics/view/pilots_head_psi"); // pilot head yaw
	MPD_Zulu = XPLMFindDataRef("sim/time/zulu_time_sec"); // Zulu time
	// nc3 (asss new with var)
	memset(MPD_Buffer, 0, sizeof(MPD_Buffer));

	// server comm start -- 
	server.sin_family = AF_INET;
	server.sin_port = htons(UDP_PORT);
	server.sin_addr.s_addr = inet_addr("172.16.1.1");

	sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock != SOCKET_ERROR)
	{
		char broadcast = '1';
		int res = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
		if (res < 0)
		{
			// ERROR
			sprintf(MPD_Buffer[0], "setsockopts error %d", res);
			closesocket(sock);
		} else {
			sprintf(MPD_Buffer[0], "socket ok");
		}
	} else {
		// ERROR
		sprintf(MPD_Buffer[0], "socket create error");
	}
	// Server comm end sec -- 
	fp = fopen("xpsp7.log", "w");
	return 1;
}

//---------------------------------------------------------------------------

PLUGIN_API void	XPluginStop(void)
{
    XPLMDestroyWindow(MPD_Window);
	XPLMUnregisterFlightLoopCallback(MotionPlatformDataLoopCB, NULL);
	fclose(fp);
}

//---------------------------------------------------------------------------

PLUGIN_API int XPluginEnable(void)
{
	return 1;
}

//---------------------------------------------------------------------------

PLUGIN_API void XPluginDisable(void)
{
}

//---------------------------------------------------------------------------

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void * inParam)
{
}


//---------------------------------------------------------------------------
// Mandatory callback for SDK 2D Window
// Used to display the data to the screen

void MotionPlatformDataDrawWindowCallback(
                                   XPLMWindowID         inWindowID,    
                                   void *               inRefcon)
{

	float		rgb [] = { 1.0, 1.0, 1.0 };
	int			l, t, r, b;

	XPLMGetWindowGeometry(inWindowID, &l, &t, &r, &b);
	XPLMDrawTranslucentDarkBox(l, t, r, b);

	for (int i=0; i<NUM_LINES; i++)
		XPLMDrawString(rgb, l+10, (t-20) - (10*i), MPD_Buffer[i], NULL, xplmFont_Basic);
}                                   

//---------------------------------------------------------------------------
// Mandatory callback for SDK 2D Window
// Not used in this plugin

void MotionPlatformDataHandleKeyCallback(
                                   XPLMWindowID         inWindowID,    
                                   char                 inKey,    
                                   XPLMKeyFlags         inFlags,    
                                   char                 inVirtualKey,    
                                   void *               inRefcon,    
                                   int                  losingFocus)
{
}                                   

//---------------------------------------------------------------------------
// Mandatory callback for SDK 2D Window
// Not used in this plugin

int MotionPlatformDataHandleMouseClickCallback(
                                   XPLMWindowID         inWindowID,    
                                   int                  x,    
                                   int                  y,    
                                   XPLMMouseStatus      inMouse,    
                                   void *               inRefcon)
{
	return 1;
}                                      

//---------------------------------------------------------------------------
// FlightLoop callback to calculate motion data and store it in our buffers

float MotionPlatformDataLoopCB(float elapsedMe, float elapsedSim, int counter, void * refcon)
{
	static const char* datatitles[] = { "T", "ax", "ay", "az", "roll", "pitch", "yaw", "vroll", "vpitch", "vyaw", "GS", "mass",
										"ph_x", "ph_y", "ph_z", "ph_roll", "ph_pitch", "ph_yaw", "zulu_time" }; //nc4 (add header)

	MPD_CalculateMotionData();

	float t = MPD_MotionData[0];
	float hz = 1.0 / (t - lasttime);
	lasttime = t;

	int res = sendto(sock, (char *)MPD_MotionData, sizeof(MPD_MotionData), 0, (sockaddr *)&server, sizeof(server)); //send data to network {cueing} // Add an infinite while loop and a sleep for some milliseconds to send data
	if (res < 0)
	{
		sprintf(MPD_Buffer[1], "send error %d", res);
	} else {
		sprintf(MPD_Buffer[1], "send ok [%d] [%dHz]", res, (int)hz);
	}

	for (int i = 0; i < NUM_DATA; i++)
	{
		sprintf(MPD_Buffer[i + 2], "%6s= %f", datatitles[i], MPD_MotionData[i]);
		if (fp != NULL)
		{
			if (i == 0)
				fprintf(fp, "%f", MPD_MotionData[i]);
			else
				fprintf(fp, ",%f", MPD_MotionData[i]);
		}
	}
	fprintf(fp, "\n");

	return (float)1.0/60.0; // maximum 60 Hz updates
}

//---------------------------------------------------------------------------
// Original function used in the Xplane code.

float MPD_fallout(float data, float low, float high)
{
	if (data < low) return data;
	if (data > high) return data;
	if (data < ((low + high) * 0.5)) return low;
    return high;
}

//---------------------------------------------------------------------------
// Original function used in the Xplane code.

float MPD_fltlim(float data, float min, float max)
{
	if (data < min) return min;
	if (data > max) return max;
	return data;
}

//---------------------------------------------------------------------------
// Original function used in the Xplane code.

float MPD_fltmax2 (float x1,const float x2)
{
	return (x1 > x2) ? x1 : x2;
}

//---------------------------------------------------------------------------
// This is original Xplane code converted to use 
// our datarefs instead of the Xplane variables

void MPD_CalculateMotionData(void)
{
	float t = XPLMGetDataf(MPD_DR_t);
	float groundspeed = XPLMGetDataf(MPD_DR_groundspeed);
	float m_total = XPLMGetDataf(MPD_DR_m_total);
	float ax = XPLMGetDataf(MPD_DR_ax);
	float ay = XPLMGetDataf(MPD_DR_ay);
	float az = XPLMGetDataf(MPD_DR_az);
	float phi = XPLMGetDataf(MPD_DR_phi); // roll// ic
	float theta = XPLMGetDataf(MPD_DR_the); // pitch
	float psi = XPLMGetDataf(MPD_DR_psi); // yaw //ic
	float p = XPLMGetDataf(MPD_DR_p); //vroll
	float q = XPLMGetDataf(MPD_DR_q); // vpitch
	float r = XPLMGetDataf(MPD_DR_r);  // vyaw
	float ph_x = XPLMGetDataf(MPD_PH_x);
	float ph_y = XPLMGetDataf(MPD_PH_y);
	float ph_z = XPLMGetDataf(MPD_PH_z);
	float ph_phi = XPLMGetDataf(MPD_PH_phi);
	float ph_theta = XPLMGetDataf(MPD_PH_theta);
	float ph_psi = XPLMGetDataf(MPD_PH_psi);
	float zulu = XPLMGetDataf(MPD_Zulu); 

	//nc5 (get values)

	// Store the results in an array so that we can easily send it.
	MPD_MotionData[0] = t;
	MPD_MotionData[1] = ax;
	MPD_MotionData[2] = ay;
	MPD_MotionData[3] = az;
	MPD_MotionData[4] = phi;
	MPD_MotionData[5] = theta;
	MPD_MotionData[6] = psi;
	MPD_MotionData[7] = p;
	MPD_MotionData[8] = q;
	MPD_MotionData[9] = r;
	MPD_MotionData[10] = groundspeed;
	MPD_MotionData[11] = m_total;
	MPD_MotionData[12] = ph_x;
	MPD_MotionData[13] = ph_y;
	MPD_MotionData[14] = ph_z;
	MPD_MotionData[15] = ph_phi;
	MPD_MotionData[16] = ph_theta;
	MPD_MotionData[17] = ph_psi;
	MPD_MotionData[18] = zulu;

	// nc6(put in the array)
}

//---------------------------------------------------------------------------

