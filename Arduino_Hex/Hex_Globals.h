//==============================================================================
// GLOBALS - The main global definitions for the CPhenix program - still needs
//		to be cleaned up.
// This program assumes that the main files were compiled as C files
//==============================================================================
#ifndef _HEX_GLOBALS_H_
#define _HEX_GLOBALS_H_
#include "Hex_Cfg.h"

//=============================================================================
//[CONSTANTS]
//=============================================================================
#define BUTTON_DOWN 0
#define BUTTON_UP 	1

#define	c1DEC		10
#define	c2DEC		100
#define	c4DEC		10000
#define	c6DEC		1000000

#define	cRR			0
#define	cRM			1
#define	cRF			2
#define	cLR			3
#define	cLM			4
#define	cLF			5

#define _BAP28_ 1
#define	WTIMERTICSPERMSMUL  	64	// BAP28 is 16mhz need a multiplyer and divider to make the conversion with /8192
#define WTIMERTICSPERMSDIV  	125 // 
#define USEINT_TIMERAV


extern void GaitSelect(void);
extern short SmoothControl (short CtrlMoveInp, short CtrlMoveOut, byte CtrlDivider);


//-----------------------------------------------------------------------------
// Define Global variables
//-----------------------------------------------------------------------------
extern boolean			g_fHexOn;				//Switch to turn on Phoenix
extern boolean			g_fPrev_HexOn;			//Previous loop state 

//Body position
extern long		BodyPosX; 		//Global Input for the position of the body
extern long		BodyPosY; 
extern long		BodyPosZ; 

//Body Inverse Kinematics
extern long		BodyRotX1; 		//Global Input pitch of the body
extern long		BodyRotY1;		//Global Input rotation of the body
extern long		BodyRotZ1; 		//Global Input roll of the body


//[gait]
extern byte			GaitType;			//Gait type
extern short		NomGaitSpeed;		//Nominal speed of the gait

extern short		LegLiftHeight;		//Current Travel height
extern long			TravelLengthX;		//Current Travel length X
extern long			TravelLengthZ; 		//Current Travel length Z
extern long			TravelRotationY; 	//Current Travel Rotation Y

//[Single Leg Control]
extern byte			SelectedLeg;
extern short		SLLegX;
extern short		SLLegY;
extern short		SLLegZ;
extern boolean			fSLHold;		 	//Single leg control mode

//--------------------------------------------------------------------
//[GP PLAYER]
extern boolean	GPStart;						//Start the GP Player
extern byte	GPSeq;							//Number of the sequence
extern boolean	GPEnable;						//Enables the GP player when the SSC version ends with "GP<cr>"

//[Balance]
extern boolean BalanceMode;

//[TIMING]
extern byte			InputTimeDelay;	//Delay that depends on the input to get the "sneaking" effect
extern word			SpeedControl;	//Adjustible Delay
extern word			SSCTime;		//Time for servo updates

// maybe should do this different
extern void	ServoDriverStart(void);
extern void	ServoDriverCommit(void);


extern void MSound(uint8_t _pin, byte cNotes, ...);
extern int DBGPrintf(const char *format, ...);
extern int SSCPrintf(const char *format, ...);
extern int SSCRead (byte* pb, int cb, word wTimeout, word wEOL);



// The defined controller must provide the following
extern void InitController(void);
extern void	ControlInput(void);
extern void	AllowControllerInterrupts(boolean fAllow);


// debug handler...
extern boolean g_fDBGHandleError;
#endif

