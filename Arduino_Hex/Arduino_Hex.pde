//#define DEBUG_PINCALCS
//#define DEBUG_BODYCALCS
//#define DEBUG_GAITS
//Project Lynxmotion Phoenix
//Description: Phoenix software
//Software version: V2.0
//Date: 29-10-2009
//Programmer: Jeroen Janssen [aka Xan]
//         Kurt Eckhardt(KurtE) converted to C and Arduino
//
// Seeeduino Mega Pro...
//
//NEW IN V2.X
//
//COMPILE USING ARDUINO 0021 WITH PS2 LIBRARY ONLY; NOT USING MODIFIED HARDWARE SERIAL LIBRARY.
//
//KNOWN BUGS:
//    - Lots ;)
//
//=============================================================================
// Header Files
//=============================================================================
#include <PS2X_lib.h>
#include <pins_arduino.h>
#include "Hex_globals.h"
#include "diyxbee.h"
#define BalanceDivFactor 6    //;Other values than 6 can be used, testing...CAUTION!! At your own risk ;)


//--------------------------------------------------------------------
//[TABLES]
//ArcCosinus Table
//Table build in to 3 part to get higher accuracy near cos = 1. 
//The biggest error is near cos = 1 and has a biggest value of 3*0.012098rad = 0.521 deg.
//-    Cos 0 to 0.9 is done by steps of 0.0079 rad. [1/127]
//-    Cos 0.9 to 0.99 is done by steps of 0.0008 rad [0.1/127]
//-    Cos 0.99 to 1 is done by step of 0.0002 rad [0.01/64]
//Since the tables are overlapping the full range of 127+127+64 is not necessary. Total bytes: 277

static const byte GetACos[] = {    
                    255,254,252,251,250,249,247,246,245,243,242,241,240,238,237,236,234,233,232,231,229,228,227,225, 
                    224,223,221,220,219,217,216,215,214,212,211,210,208,207,206,204,203,201,200,199,197,196,195,193, 
                    192,190,189,188,186,185,183,182,181,179,178,176,175,173,172,170,169,167,166,164,163,161,160,158, 
                    157,155,154,152,150,149,147,146,144,142,141,139,137,135,134,132,130,128,127,125,123,121,119,117, 
                    115,113,111,109,107,105,103,101,98,96,94,92,89,87,84,81,79,76,73,73,73,72,72,72,71,71,71,70,70, 
                    70,70,69,69,69,68,68,68,67,67,67,66,66,66,65,65,65,64,64,64,63,63,63,62,62,62,61,61,61,60,60,59,
                    59,59,58,58,58,57,57,57,56,56,55,55,55,54,54,53,53,53,52,52,51,51,51,50,50,49,49,48,48,47,47,47,
                    46,46,45,45,44,44,43,43,42,42,41,41,40,40,39,39,38,37,37,36,36,35,34,34,33,33,32,31,31,30,29,28,
                    28,27,26,25,24,23,23,23,23,22,22,22,22,21,21,21,21,20,20,20,19,19,19,19,18,18,18,17,17,17,17,16,
                    16,16,15,15,15,14,14,13,13,13,12,12,11,11,10,10,9,9,8,7,6,6,5,3,0 };//
                    
//Sin table 90 deg, persision 0.5 deg [180 values]
static const word GetSin[] = {0, 87, 174, 261, 348, 436, 523, 610, 697, 784, 871, 958, 1045, 1132, 1218, 1305, 1391, 1478, 1564, 
                 1650, 1736, 1822, 1908, 1993, 2079, 2164, 2249, 2334, 2419, 2503, 2588, 2672, 2756, 2840, 2923, 3007, 
                 3090, 3173, 3255, 3338, 3420, 3502, 3583, 3665, 3746, 3826, 3907, 3987, 4067, 4146, 4226, 4305, 4383, 
                 4461, 4539, 4617, 4694, 4771, 4848, 4924, 4999, 5075, 5150, 5224, 5299, 5372, 5446, 5519, 5591, 5664, 
                 5735, 5807, 5877, 5948, 6018, 6087, 6156, 6225, 6293, 6360, 6427, 6494, 6560, 6626, 6691, 6755, 6819, 
                 6883, 6946, 7009, 7071, 7132, 7193, 7253, 7313, 7372, 7431, 7489, 7547, 7604, 7660, 7716, 7771, 7826, 
                 7880, 7933, 7986, 8038, 8090, 8141, 8191, 8241, 8290, 8338, 8386, 8433, 8480, 8526, 8571, 8616, 8660, 
                 8703, 8746, 8788, 8829, 8870, 8910, 8949, 8987, 9025, 9063, 9099, 9135, 9170, 9205, 9238, 9271, 9304, 
                 9335, 9366, 9396, 9426, 9455, 9483, 9510, 9537, 9563, 9588, 9612, 9636, 9659, 9681, 9702, 9723, 9743, 
                 9762, 9781, 9799, 9816, 9832, 9848, 9862, 9876, 9890, 9902, 9914, 9925, 9935, 9945, 9953, 9961, 9969, 
                 9975, 9981, 9986, 9990, 9993, 9996, 9998, 9999, 10000 };//


//Build tables for Leg configuration like I/O and MIN/imax values to easy access values using a FOR loop
//Constants are still defined as single values in the cfg file to make it easy to read/configure

//SSC Pin numbers
static const byte cCoxaPin[] =     {cRRCoxaPin,  cRMCoxaPin,  cRFCoxaPin,  cLRCoxaPin,  cLMCoxaPin,  cLFCoxaPin};
static const byte cFemurPin[] = {cRRFemurPin, cRMFemurPin, cRFFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin};
static const byte cTibiaPin[] = {cRRTibiaPin, cRMTibiaPin, cRFTibiaPin, cLRTibiaPin, cLMTibiaPin, cLFTibiaPin};

//Min / imax values
static const short cCoxaMin1[] = {cRRCoxaMin1,  cRMCoxaMin1,  cRFCoxaMin1,  cLRCoxaMin1,  cLMCoxaMin1,  cLFCoxaMin1};
static const short cCoxaMax1[] = {cRRCoxaMax1,  cRMCoxaMax1,  cRFCoxaMax1,  cLRCoxaMax1,  cLMCoxaMax1,  cLFCoxaMax1};
static const short cFemurMin1[] ={cRRFemurMin1, cRMFemurMin1, cRFFemurMin1, cLRFemurMin1, cLMFemurMin1, cLFFemurMin1};
static const short cFemurMax1[] ={cRRFemurMax1, cRMFemurMax1, cRFFemurMax1, cLRFemurMax1, cLMFemurMax1, cLFFemurMax1};
static const short cTibiaMin1[] ={cRRTibiaMin1, cRMTibiaMin1, cRFTibiaMin1, cLRTibiaMin1, cLMTibiaMin1, cLFTibiaMin1};
static const short cTibiaMax1[] = {cRRTibiaMax1, cRMTibiaMax1, cRFTibiaMax1, cLRTibiaMax1, cLMTibiaMax1, cLFTibiaMax1};

//Body Offsets [distance between the center of the body and the center of the coxa]
static const short cOffsetX[] = {cRROffsetX, cRMOffsetX, cRFOffsetX, cLROffsetX, cLMOffsetX, cLFOffsetX};
static const short cOffsetZ[] = {cRROffsetZ, cRMOffsetZ, cRFOffsetZ, cLROffsetZ, cLMOffsetZ, cLFOffsetZ};

//Default leg angle
static const short cCoxaAngle1[] = {cRRCoxaAngle1, cRMCoxaAngle1, cRFCoxaAngle1, cLRCoxaAngle1, cLMCoxaAngle1, cLFCoxaAngle1};

//Start positions for the leg
static const short cInitPosX[] = {cRRInitPosX, cRMInitPosX, cRFInitPosX, cLRInitPosX, cLMInitPosX, cLFInitPosX};
static const short cInitPosY[] = {cRRInitPosY, cRMInitPosY, cRFInitPosY, cLRInitPosY, cLMInitPosY, cLFInitPosY};
static const short cInitPosZ[] = {cRRInitPosZ, cRMInitPosZ, cRFInitPosZ, cLRInitPosZ, cLMInitPosZ, cLFInitPosZ};


//--------------------------------------------------------------------
//[REMOTE]                 
#define cTravelDeadZone         4    //The deadzone for the analog input from the remote
//====================================================================
//[ANGLES]
short     CoxaAngle1[6];    //Actual Angle of the horizontal hip, decimals = 1
short     FemurAngle1[6];    //Actual Angle of the vertical hip, decimals = 1
short     TibiaAngle1[6];    //Actual Angle of the knee, decimals = 1
//--------------------------------------------------------------------
//[POSITIONS SINGLE LEG CONTROL]
boolean    fSLHold ;             //Single leg control mode

short    LegPosX[6];    //Actual X Posion of the Leg
short    LegPosY[6];    //Actual Y Posion of the Leg
short    LegPosZ[6];    //Actual Z Posion of the Leg
//--------------------------------------------------------------------
//[INPUTS]

//--------------------------------------------------------------------
//[GP PLAYER]
boolean    GPStart;            //Start the GP Player
byte    GPSeq;                //Number of the sequence
boolean    GPEnable;        //Enables the GP player when the SSC version ends with "GP<cr>"
//--------------------------------------------------------------------
//[OUTPUTS]
boolean    LedA;    //Red
boolean    LedB;    //Green
boolean     LedC;    //Orange
boolean    Eyes;    //Eyes output
//--------------------------------------------------------------------
//[VARIABLES]
byte    Index;                    //Index universal used
byte    LegIndex;                //Index used for leg Index Number

//GetSinCos / ArcCos
short            AngleDeg1;        //Input Angle in degrees, decimals = 1
short            sin4;             //Output Sinus of the given Angle, decimals = 4
short            cos4;            //Output Cosinus of the given Angle, decimals = 4
short            AngleRad4;        //Output Angle in radials, decimals = 4

//GetAtan2
short            AtanX;            //Input X
short            AtanY;            //Input Y
short            Atan4;            //ArcTan2 output
short            XYhyp2;            //Output presenting Hypotenuse of X and Y

//Body position
long            BodyPosX;         //Global Input for the position of the body
long            BodyPosY; 
long            BodyPosZ; 

//Body Inverse Kinematics
long            BodyRotX1;         //Global Input pitch of the body
long            BodyRotY1;        //Global Input rotation of the body
long            BodyRotZ1;      //Global Input roll of the body
short            PosX;            //Input position of the feet X
short            PosZ;            //Input position of the feet Z
short            PosY;            //Input position of the feet Y
//long            TotalX;            //Total X distance between the center of the body and the feet
//long            TotalZ;            //Total Z distance between the center of the body and the feet
long            BodyIKPosX;        //Output Position X of feet with Rotation
long            BodyIKPosY;        //Output Position Y of feet with Rotation
long            BodyIKPosZ;        //Output Position Z of feet with Rotation
// New with zentas stuff
short            BodyRotOffsetY;    //Input Y offset value to adjust centerpoint of rotation
short            BodyRotOffsetZ;    //Input Z offset value to adjust centerpoint of rotation


//Leg Inverse Kinematics
long            IKFeetPosX;        //Input position of the Feet X
long            IKFeetPosY;        //Input position of the Feet Y
long            IKFeetPosZ;        //Input Position of the Feet Z
boolean            IKSolution;        //Output true if the solution is possible
boolean            IKSolutionWarning;    //Output true if the solution is NEARLY possible
boolean            IKSolutionError;    //Output true if the solution is NOT possible
//--------------------------------------------------------------------
//[TIMING]
unsigned long            lTimerStart;    //Start time of the calculation cycles
unsigned long            lTimerEnd;        //End time of the calculation cycles
byte            CycleTime;        //Total Cycle time

word            SSCTime;        //Time for servo updates
word            PrevSSCTime;        //Previous time for the servo updates

byte            InputTimeDelay;    //Delay that depends on the input to get the "sneaking" effect
word            SpeedControl;    //Adjustible Delay
//--------------------------------------------------------------------
//[GLOABAL]
boolean            g_fHexOn;            //Switch to turn on Phoenix
boolean            g_fPrev_HexOn;        //Previous loop state 

//--------------------------------------------------------------------
//[Balance]
boolean            BalanceMode;
long            TotalTransX;
long             TotalTransZ;
long             TotalTransY;
long             TotalYBal1;
long             TotalXBal1;
long             TotalZBal1;

//[Single Leg Control]
byte            SelectedLeg;
byte            Prev_SelectedLeg;
short            SLLegX;
short            SLLegY;
short            SLLegZ;
boolean            AllDown;

//[gait]
byte            GaitType;            //Gait type
short            NomGaitSpeed;        //Nominal speed of the gait

short            LegLiftHeight;        //Current Travel height
long             TravelLengthX;        //Current Travel length X
long             TravelLengthZ;         //Current Travel length Z
long             TravelRotationY;     //Current Travel Rotation Y

short            TLDivFactor;        //Number of steps that a leg is on the floor while walking
short            NrLiftedPos;        //Number of positions that a single leg is lifted [1-3]
boolean            HalfLiftHeigth;        //If TRUE the outer positions of the ligted legs will be half height    

boolean            GaitInMotion;         //Temp to check if the gait is in motion
byte            StepsInGait;        //Number of steps in gait
boolean            LastLeg;             //TRUE when the current leg is the last leg of the sequence
byte            GaitStep;            //Actual Gait step

byte            GaitLegNr[6];        //Init position of the leg

byte            GaitLegNrIn;        //Input Number of the leg

long             GaitPosX[6];        //Array containing Relative X position corresponding to the Gait
long             GaitPosY[6];        //Array containing Relative Y position corresponding to the Gait
long             GaitPosZ[6];         //Array containing Relative Z position corresponding to the Gait
long             GaitRotY[6];        //Array containing Relative Y rotation corresponding to the Gait

boolean            fWalking;            //  True if the robot are walking
boolean            fContinueWalking;    // should we continue to walk?



//=============================================================================
// Function prototypes
//=============================================================================
extern void GaitSelect(void);
extern void    WriteOutputs(void);    
extern boolean FIsSSCGPEnabled(void);
extern void GPPlayer(void);
extern void    SingleLegControl(void);
extern void GaitSeq(void);
extern void    ServoDriverStart(void);
extern void ServoDriverCommit(void);
extern void FreeServos(void);
extern void BalanceBody(void);
extern void CheckAngles();

extern void    PrintSystemStuff(void);            // Try to see why we fault...

         
extern void BalCalcOneLeg (short PosX, short PosZ, short PosY, byte BalLegNr);
extern void BodyIK (short PosX, short PosZ, short PosY, short RotationY, byte BodyIKLeg) ;
extern void LegIK (short IKFeetPosX, short IKFeetPosY, short IKFeetPosZ, byte LegIKLegNr);
extern void Gait (byte GaitCurrentLegNr);
extern short GetATan2 (short AtanX, short AtanY);


//--------------------------------------------------------------------------
// SETUP: the main arduino setup function.
//--------------------------------------------------------------------------
void setup(){
    int error;

    DBGSerial.begin(57600);
    SSCSerial.begin(115200);

//Checks SSC version number if it ends with "GP"
//enable the GP player if it does
    
    delay(10);
    GPEnable = FIsSSCGPEnabled();

    // debug stuff
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    
    //Turning off all the leds
    LedA = 0;
    LedB = 0;
    LedC = 0;
    Eyes = 0;
         
    //Tars Init Positions
    for (LegIndex= 0; LegIndex <= 5; LegIndex++ )
    {
        LegPosX[LegIndex] = cInitPosX[LegIndex];    //Set start positions for each leg
        LegPosY[LegIndex] = cInitPosY[LegIndex];
        LegPosZ[LegIndex] = cInitPosZ[LegIndex];  
    }
    
    //Single leg control. Make sure no leg is selected
    SelectedLeg = 255; // No Leg selected
    Prev_SelectedLeg = 255;
    
    //Body Positions
    BodyPosX = 0;
    BodyPosY = 0;
    BodyPosZ = 0;
        
    //Body Rotations
    BodyRotX1 = 0;
    BodyRotY1 = 0;
    BodyRotZ1 = 0;
    BodyRotOffsetY = 0;        //Input Y offset value to adjust centerpoint of rotation
    BodyRotOffsetZ = 0;
    
        
    //Gait
    GaitType = 0;
    BalanceMode = 0;
    LegLiftHeight = 50;
    GaitStep = 1;
    GaitSelect();
    
    //Initialize Controller
    DBGPrintf("Before Init Controller\r");
    InitController();
    DBGPrintf("After Init Controller\r");
    
    //SSC
    SSCTime = 150;
    g_fHexOn = 0;
}

    
//=============================================================================
// Loop: the main arduino main Loop function
//=============================================================================


void loop(void)
{
    //Start time
    lTimerStart = millis(); 
    digitalWrite(2, HIGH);
    //Read input
    ControlInput();

#ifdef DEBUG_GAITS  
  // If we are in this mode, lets normalize the inputs as to make it easier to compare later...
  DebugLimitJoysticks(&TravelLengthX);
  DebugLimitJoysticks(&TravelLengthZ);
  DebugLimitJoysticks(&TravelRotationY);
  DebugLimitJoysticks(&BodyPosX);
  DebugLimitJoysticks(&BodyPosZ);
  DebugLimitJoysticks(&BodyRotY1);
  DebugLimitJoysticks(&BodyRotX1);
  DebugLimitJoysticks(&BodyRotZ1);
  
  // Try to also normalize BodyposY
  BodyPosY &= 0xFFF8;
#endif ;DEBUG_GAITS

    
    //GOSUB ReadButtons    //I/O used by the remote
    WriteOutputs();        //Write Outputs
            
    // Some debug stuff...
    #ifdef DEBUG_GAITS
    if (g_fHexOn && (TravelLengthX || TravelLengthZ))
    {
        DBGPrintf("Gait: %d %d BP:(%d, %d, %d) TL: (%d, %d), TRY: %d\n\r", 
            GaitType, GaitStep,
            (short)BodyPosX,(short)BodyPosY, (short)BodyPosZ,
            (short)TravelLengthX, (short)TravelLengthZ, (short)TravelRotationY);
    }
    #endif        
    //GP Player
    if (GPEnable)
        GPPlayer();
            
    //Single leg control
    SingleLegControl ();
            
    //Gait
    GaitSeq();
             
    //Balance calculations
    TotalTransX = 0;     //reset values used for calculation of balance
    TotalTransZ = 0;
    TotalTransY = 0;
    TotalXBal1 = 0;
    TotalYBal1 = 0;
    TotalZBal1 = 0;
    if (BalanceMode) {
        for (LegIndex = 0; LegIndex <= 2; LegIndex++) {    // balance calculations for all Right legs
#ifdef DEBUG_GAITS
            if (g_fHexOn && (TravelLengthX || TravelLengthZ))   {
                DBGPrintf("GP? %d %d %d-", (short)GaitPosX[LegIndex], (short)GaitPosY[LegIndex], (short)GaitPosZ[LegIndex]);
            }
#endif            

            BalCalcOneLeg (-LegPosX[LegIndex]+GaitPosX[LegIndex], 
                        LegPosZ[LegIndex]+GaitPosZ[LegIndex], 
                        (LegPosY[LegIndex]-cInitPosY[LegIndex])+GaitPosY[LegIndex], LegIndex);
        }

        for (LegIndex = 3; LegIndex <= 5; LegIndex++) {    // balance calculations for all Right legs
#ifdef DEBUG_GAITS
            if (g_fHexOn && (TravelLengthX || TravelLengthZ))   {
                DBGPrintf("GP? %d %d %d-", (short)GaitPosX[LegIndex], (short)GaitPosY[LegIndex], (short)GaitPosZ[LegIndex]);
            }
#endif            
            BalCalcOneLeg(LegPosX[LegIndex]+GaitPosX[LegIndex], 
                        LegPosZ[LegIndex]+GaitPosZ[LegIndex], 
                        (LegPosY[LegIndex]-cInitPosY[LegIndex])+GaitPosY[LegIndex], LegIndex);
        }
        BalanceBody();
    }
          
 //Reset IKsolution indicators 
     IKSolution = 0 ;
     IKSolutionWarning = 0; 
     IKSolutionError = 0 ;
            
     //Do IK for all Right legs
     for (LegIndex = 0; LegIndex <=2; LegIndex++) {    
        BodyIK(-LegPosX[LegIndex]+BodyPosX+GaitPosX[LegIndex] - TotalTransX,
                LegPosZ[LegIndex]+BodyPosZ+GaitPosZ[LegIndex] - TotalTransZ,
                LegPosY[LegIndex]+BodyPosY+GaitPosY[LegIndex] - TotalTransY,
                GaitRotY[LegIndex], LegIndex);
                               
        LegIK (LegPosX[LegIndex]-BodyPosX+BodyIKPosX-(GaitPosX[LegIndex] - TotalTransX), 
        LegPosY[LegIndex]+BodyPosY-BodyIKPosY+GaitPosY[LegIndex] - TotalTransY,
        LegPosZ[LegIndex]+BodyPosZ-BodyIKPosZ+GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
    }
          
    //Do IK for all Left legs  
    for (LegIndex = 3; LegIndex <=5; LegIndex++) {
        BodyIK(LegPosX[LegIndex]-BodyPosX+GaitPosX[LegIndex] - TotalTransX,
                LegPosZ[LegIndex]+BodyPosZ+GaitPosZ[LegIndex] - TotalTransZ,
                LegPosY[LegIndex]+BodyPosY+GaitPosY[LegIndex] - TotalTransY,
                GaitRotY[LegIndex], LegIndex);
        LegIK (LegPosX[LegIndex]+BodyPosX-BodyIKPosX+GaitPosX[LegIndex] - TotalTransX,
                LegPosY[LegIndex]+BodyPosY-BodyIKPosY+GaitPosY[LegIndex] - TotalTransY,
                LegPosZ[LegIndex]+BodyPosZ-BodyIKPosZ+GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
    }

    //Check mechanical limits
    CheckAngles();
                
    //Write IK errors to leds
    LedC = IKSolutionWarning;
    LedA = IKSolutionError;
            
    //Drive Servos
    if (g_fHexOn) {
        if (g_fHexOn && !g_fPrev_HexOn) {
            MSound(SOUND_PIN, 3, 60, 2000, 80, 2250, 100, 2500);
            XBeePlaySounds(3, 60, 2000, 80, 2250, 100, 2500);
            Eyes = 1;
        }
        
        //Set SSC time
        if ((abs(TravelLengthX)>cTravelDeadZone) || (abs(TravelLengthZ)>cTravelDeadZone) ||
                (abs(TravelRotationY*2)>cTravelDeadZone)) {         
            SSCTime = NomGaitSpeed + (InputTimeDelay*2) + SpeedControl;
                
            //Add aditional delay when Balance mode is on
            if (BalanceMode)
                SSCTime = SSCTime + 100;
        } else //Movement speed excl. Walking
            SSCTime = 200 + SpeedControl;
        
        // note we broke up the servo driver into start/commit that way we can output all of the servo information
        // before we wait and only have the termination information to output after the wait.  That way we hopefully
        // be more accurate with our timings...
        ServoDriverStart();
        
        // Sync BAP with SSC while walking to ensure the prev is completed before sending the next one
                
        fContinueWalking = false;
            
        // Finding any incident of GaitPos/Rot <>0:
        for (LegIndex = 0; LegIndex <= 5; LegIndex++) {
            if ( (GaitPosX[LegIndex] > 2) || (GaitPosX[LegIndex] < -2)
                    || (GaitPosY[LegIndex] > 2) || (GaitPosY[LegIndex] < -2)
                    || (GaitPosZ[LegIndex] > 2) || (GaitPosZ[LegIndex] < -2)
                    || (GaitRotY[LegIndex] > 2) || (GaitRotY[LegIndex] < -2) )    {
                fContinueWalking = true;
                break;
            }
        }
        if (fWalking || fContinueWalking) {
            word  wDelayTime;
            fWalking = fContinueWalking;
                  
            //Get endtime and calculate wait time
            lTimerEnd = millis();
            if (lTimerEnd > lTimerStart)
                CycleTime = lTimerEnd-lTimerStart;
            else
                CycleTime = 0xffffffffL - lTimerEnd + lTimerStart + 1;
            
            // if it is less, use the last cycle time...
            //Wait for previous commands to be completed while walking
            wDelayTime = (min(max ((PrevSSCTime - CycleTime), 1), NomGaitSpeed));
//            DBGPrintf("Delay: %d %d %d %d\n\r", (word)NomGaitSpeed, (word)CycleTime, (word)PrevSSCTime, (word)wDelayTime);
            digitalWrite(3, HIGH);
            delay (wDelayTime); 
            digitalWrite(3, LOW);
        }
        
        ServoDriverCommit();
        digitalWrite(2, LOW);            

    } else {
        //Turn the bot off
        if (g_fPrev_HexOn || (AllDown= 0)) {
            SSCTime = 600;
            ServoDriverStart();
            ServoDriverCommit();
            MSound(SOUND_PIN, 3, 100, 2500, 80, 2250, 60, 2000);
            XBeePlaySounds(3, 100, 2500, 80, 2250, 60, 2000);
            delay(600);
        } else {
            FreeServos();
            Eyes = 0;
        }
    }
                  
    //Store previous g_fHexOn State
    if (g_fHexOn)
        g_fPrev_HexOn = 1;
    else
        g_fPrev_HexOn = 0;
}


//===================================================================
#ifdef DEBUG_GAITS 
void DebugLimitJoysticks(long int *pDLJVal)
{
    // If we are in this mode, lets normalize the inputs as to make it easier to compare later...
    if (*pDLJVal < -64)
        *pDLJVal = -127;
    else if( *pDLJVal > 64)
        *pDLJVal = 127;
    else
        *pDLJVal = 0;
}
#endif


//--------------------------------------------------------------------
//[ReadButtons] Reading input buttons from the ABB
//--------------------------------------------------------------------
void ReadButtons(void)
{
}
//--------------------------------------------------------------------
//[WriteOutputs] Updates the state of the leds
//--------------------------------------------------------------------
void WriteOutputs(void)
{
 #if 0
    if (Eyes == 0)
        digitalWrite(cEyesPin, LOW);
    else
        digitalWrite (cEyesPin, HIGH);
#endif        
}

//--------------------------------------------------------------------
//[GP PLAYER]
//--------------------------------------------------------------------
boolean FIsSSCGPEnabled(void)
{
    char abVer[40];        // give a nice large buffer.
    byte cbRead;
    SSCSerial.print("ver\r");
    
    cbRead = SSCRead((byte*)abVer, sizeof(abVer), 10000, 13);

    DBGPrintf("Check GP Enable - cb %d\r", cbRead);
    if (cbRead > 0) {
        byte iT;
        for (iT = 0; iT < cbRead; iT++)
            DBGPrintf("%2x ", abVer[iT]);
        DBGSerial.write((byte*)abVer, cbRead);
    }
        
    if ((cbRead > 3) && (abVer[cbRead-3]=='G') && (abVer[cbRead-2]=='P') && (abVer[cbRead-1]==13))
        return true;

    MSound (SOUND_PIN, 2, 40, 2500, 40, 2500);
        XBeePlaySounds(2, 40, 2500, 40, 2500);
    
    return false;
}


//--------------------------------------------------------------------
//[GP PLAYER]
void GPPlayer(void)
{
    byte abStat[4];
    byte cbRead;
    
    //Start sequence
    if (GPStart==1) {
        AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...

        SSCPrintf("PL0SQ%dONCE\r", GPSeq); //Start sequence
                delay(20);
                SSCSerial.flush();        // get rid of anything that was previously queued up...
    
        //Wait for GPPlayer to complete sequence    
        do {
            SSCSerial.print("QPL0\r");
            cbRead = SSCRead((byte*)abStat, sizeof(abStat), 10000, (word)-1);  //    [GPStatSeq, GPStatFromStep, GPStatToStep, GPStatTime]
                        DBGPrintf("GP:%u %x %x %x %x\n\r", cbRead, abStat[0], abStat[1], abStat[2], abStat[3]);
                        delay(20);
        }
        while ((cbRead == sizeof(abStat)) && ((abStat[0]!=255) || (abStat[1]!=0) || (abStat[2]!=0) || (abStat[3]!=0)));

        AllowControllerInterrupts(true);    // Ok to process hserial again...

        GPStart=0;
    }  
}
 
//--------------------------------------------------------------------
//[SINGLE LEG CONTROL]
void SingleLegControl(void)
{

  //Check if all legs are down
    AllDown = (LegPosY[cRF]==cInitPosY[cRF]) && (LegPosY[cRM]==cInitPosY[cRM]) && (LegPosY[cRR]==cInitPosY[cRR]) & 
              (LegPosY[cLR]==cInitPosY[cLR]) && (LegPosY[cLM]==cInitPosY[cLM]) && (LegPosY[cLF]=cInitPosY[cLF]);

    if (SelectedLeg<=5) {
        if (SelectedLeg!=Prev_SelectedLeg) {
            if (AllDown) { //Lift leg a bit when it got selected
                LegPosY[SelectedLeg] = cInitPosY[SelectedLeg]-20;
        
                //Store current status
                  Prev_SelectedLeg = SelectedLeg;
            } else {//Return prev leg back to the init position
                LegPosX[Prev_SelectedLeg] = cInitPosX[Prev_SelectedLeg];
                LegPosY[Prev_SelectedLeg] = cInitPosY[Prev_SelectedLeg];
                LegPosZ[Prev_SelectedLeg] = cInitPosZ[Prev_SelectedLeg];
            }
        } else if (!fSLHold) {
            LegPosY[SelectedLeg] = LegPosY[SelectedLeg]+SLLegY;
            LegPosX[SelectedLeg] = cInitPosX[SelectedLeg]+SLLegX;
            LegPosZ[SelectedLeg] = cInitPosZ[SelectedLeg]+SLLegZ;     
        }
    } else {//All legs to init position
        if (!AllDown) {
            for(LegIndex = 0; LegIndex <= 5;LegIndex++) {
                LegPosX[LegIndex] = cInitPosX[LegIndex];
                LegPosY[LegIndex] = cInitPosY[LegIndex];
                LegPosZ[LegIndex] = cInitPosZ[LegIndex];
            }
        } 
        if (Prev_SelectedLeg!=255)
            Prev_SelectedLeg = 255;
    }
}


//--------------------------------------------------------------------
void GaitSelect(void)
{
  //Gait selector
    if (GaitType == 0) { //Ripple Gait 6 steps
        GaitLegNr[cLR] = 1;
        GaitLegNr[cRF] = 2;    
        GaitLegNr[cLM] = 3;      
        GaitLegNr[cRR] = 4;      
        GaitLegNr[cLF] = 5;      
        GaitLegNr[cRM] = 6;
                  
        NrLiftedPos = 1;
        HalfLiftHeigth = 0;    
        TLDivFactor = 4;
        StepsInGait = 6;
        NomGaitSpeed = 100;
    }    
    
    if (GaitType ==1 ) { //Ripple Gait 12 steps
        GaitLegNr[cLR] = 1;
        GaitLegNr[cRF] = 3;
        GaitLegNr[cLM] = 5;
        GaitLegNr[cRR] = 7;
        GaitLegNr[cLF] = 9;
        GaitLegNr[cRM] = 11;
        
        NrLiftedPos = 3;
        HalfLiftHeigth = 0;
        TLDivFactor = 8;      
        StepsInGait = 12;    
        NomGaitSpeed = 85;
    }
    
    if (GaitType == 2) { //Quadripple 9 steps
        GaitLegNr[cLR] = 1;
        GaitLegNr[cRF] = 2;
        GaitLegNr[cLM] = 4;      
        GaitLegNr[cRR] = 5;
        GaitLegNr[cLF] = 7;
        GaitLegNr[cRM] = 8;
            
        NrLiftedPos = 2;
        HalfLiftHeigth = 0;    
        TLDivFactor = 6;      
        StepsInGait = 9;        
        NomGaitSpeed = 100;
    }

     if (GaitType == 3) { //Tripod 4 steps
        GaitLegNr[cLR] = 3    ;
        GaitLegNr[cRF] = 1;
        GaitLegNr[cLM] = 1;
        GaitLegNr[cRR] = 1;
        GaitLegNr[cLF] = 3;
        GaitLegNr[cRM] = 3;
            
        NrLiftedPos = 1    ;
        HalfLiftHeigth = 0;        
        TLDivFactor = 2;      
        StepsInGait = 4;        
        NomGaitSpeed = 150;
    }  
    
  if (GaitType == 4) { //Tripod 6 steps
        GaitLegNr[cLR] = 4    ;
        GaitLegNr[cRF] = 1;
        GaitLegNr[cLM] = 1;
        GaitLegNr[cRR] = 1;
        GaitLegNr[cLF] = 4;
        GaitLegNr[cRM] = 4;
            
        NrLiftedPos = 2;
        HalfLiftHeigth = 0    ;
        TLDivFactor = 4    ;  
        StepsInGait = 6     ;   
        NomGaitSpeed = 100;
    }
  
    if (GaitType == 5) { //Tripod 8 steps
        GaitLegNr[cLR] = 5;
        GaitLegNr[cRF] = 1;
        GaitLegNr[cLM] = 1;
        GaitLegNr[cRR] = 1;
        GaitLegNr[cLF] = 5;
        GaitLegNr[cRM] = 5;
            
        NrLiftedPos = 3;
        HalfLiftHeigth = 1    ;
        TLDivFactor = 4      ;
        StepsInGait = 8       ; 
        NomGaitSpeed = 85;
    }
  
    if (GaitType == 6) { //Wave 12 steps
        GaitLegNr[cLR] = 1;
        GaitLegNr[cRF] = 11;
        GaitLegNr[cLM] = 3;
        
        GaitLegNr[cRR] = 7;
        GaitLegNr[cLF] = 5;
        GaitLegNr[cRM] = 9;
            
        NrLiftedPos = 1;
        HalfLiftHeigth = 0    ;
        TLDivFactor = 10     ; 
        StepsInGait = 12      ;  
        NomGaitSpeed = 85;
    }
  
    if (GaitType == 7) { //Wave 18 steps
        GaitLegNr[cLR] = 4 ;
        GaitLegNr[cRF] = 1;
        GaitLegNr[cLM] = 7;
        
        GaitLegNr[cRR] = 13;
        GaitLegNr[cLF] = 10;
        GaitLegNr[cRM] = 16;
            
        NrLiftedPos = 2;
        HalfLiftHeigth = 0    ;
        TLDivFactor = 16;      
        StepsInGait = 18;        
        NomGaitSpeed = 85;
    }
}    

//--------------------------------------------------------------------
//[GAIT Sequence]
void GaitSeq(void)
{
    //Calculate Gait sequence
    LastLeg = 0;
    for (LegIndex = 0; LegIndex <= 5; LegIndex++) { // for all legs
        if (LegIndex == 5) // last leg
            LastLeg = 1 ;
    
        Gait(LegIndex);
    }    // next leg
}


//--------------------------------------------------------------------
//[GAIT]
void Gait (byte GaitCurrentLegNr)
{

    //Check if the Gait is in motion
    GaitInMotion = ((abs(TravelLengthX)>cTravelDeadZone) || (abs(TravelLengthZ)>cTravelDeadZone) || (abs(TravelRotationY)>cTravelDeadZone));
    
    //Clear values under the cTravelDeadZone
    if (GaitInMotion==0) {    
        TravelLengthX=0;
        TravelLengthZ=0;
        TravelRotationY=0;
    }
    
    //Leg middle up position
     //Gait in motion                                                          Gait NOT in motion, return to home position
    if ( (GaitInMotion && (NrLiftedPos==1 || NrLiftedPos==3) && GaitStep==GaitLegNr[GaitCurrentLegNr])  
        || ( !GaitInMotion && GaitStep==GaitLegNr[GaitCurrentLegNr] && 
                ((abs(GaitPosX[GaitCurrentLegNr])>2) || (abs(GaitPosZ[GaitCurrentLegNr])>2) 
                    || (abs(GaitRotY[GaitCurrentLegNr])>2)) ))  {          //Up
        GaitPosX[GaitCurrentLegNr] = 0;
        GaitPosY[GaitCurrentLegNr] = -LegLiftHeight;
        GaitPosZ[GaitCurrentLegNr] = 0;
        GaitRotY[GaitCurrentLegNr] = 0;
    }  else {
        //Optional Half heigth Rear
        if ( ((NrLiftedPos==2 && GaitStep==GaitLegNr[GaitCurrentLegNr])     //+2
                || (NrLiftedPos==3 && (GaitStep==(GaitLegNr[GaitCurrentLegNr]-1) || GaitStep == (GaitLegNr[GaitCurrentLegNr]+(StepsInGait-1)) )))
                 && GaitInMotion) {                
            GaitPosX[GaitCurrentLegNr] = -TravelLengthX/2;
            GaitPosY[GaitCurrentLegNr] = -LegLiftHeight/(HalfLiftHeigth+1);
            GaitPosZ[GaitCurrentLegNr] = -TravelLengthZ/2;
            GaitRotY[GaitCurrentLegNr] = -TravelRotationY/2;

        } else {
            //Optional half heigth front
            if (  (NrLiftedPos>=2) && (
                      (GaitStep==(GaitLegNr[GaitCurrentLegNr]+1) || (GaitStep==(GaitLegNr[GaitCurrentLegNr]-(StepsInGait-1))) )) && 
                      GaitInMotion)  {        
                GaitPosX[GaitCurrentLegNr] = TravelLengthX/2;
                GaitPosY[GaitCurrentLegNr] = -LegLiftHeight/(HalfLiftHeigth+1);
                GaitPosZ[GaitCurrentLegNr] = TravelLengthZ/2;
                GaitRotY[GaitCurrentLegNr] = TravelRotationY/2;
            } else {
                //Leg front down position
                if ((GaitStep==(GaitLegNr[GaitCurrentLegNr]+NrLiftedPos) 
                        || GaitStep==(GaitLegNr[GaitCurrentLegNr]-(StepsInGait-NrLiftedPos))) && (GaitPosY[GaitCurrentLegNr]<0) ) {
                      GaitPosX[GaitCurrentLegNr] = TravelLengthX/2;
                      GaitPosZ[GaitCurrentLegNr] = TravelLengthZ/2;
                      GaitRotY[GaitCurrentLegNr] = TravelRotationY/2;          
                      GaitPosY[GaitCurrentLegNr] = 0;    //Only move leg down at once if terrain adaption is turned off
                }
                //Move body forward      
                else {
                    GaitPosX[GaitCurrentLegNr] -= ((long)TravelLengthX/(long)TLDivFactor)     ;
                    GaitPosY[GaitCurrentLegNr] = 0  ;
                    GaitPosZ[GaitCurrentLegNr] -=  ((long)TravelLengthZ/(long)TLDivFactor);
                    GaitRotY[GaitCurrentLegNr] -= ((long)TravelRotationY/(long)TLDivFactor);
               }
            }
        }
    }
    //Advance to the next step
    if (LastLeg)  {  //The last leg in this step
        GaitStep = GaitStep+1;
        if (GaitStep>StepsInGait)
              GaitStep = 1;
    }
}  


//--------------------------------------------------------------------
//[BalCalcOneLeg]
void BalCalcOneLeg (short PosX, short PosZ, short PosY, byte BalLegNr)
{
    short            CPR_X;            //Final X value for centerpoint of rotation
    short            CPR_Y;            //Final Y value for centerpoint of rotation
    short            CPR_Z;            //Final Z value for centerpoint of rotation

    //Calculating totals from center of the body to the feet
    CPR_Z = cOffsetZ[BalLegNr]+PosZ;
    CPR_X = cOffsetX[BalLegNr]+PosX;
    CPR_Y = 150 + PosY;        // using the value 150 to lower the centerpoint of rotation 'BodyPosY +

    TotalTransY += (long)PosY;
    TotalTransZ += (long)CPR_Z;
    TotalTransX += (long)CPR_X;
    
    GetATan2(CPR_X, CPR_Z);
    TotalYBal1 += ((long)Atan4*1800) / 31415;
    
    GetATan2 (CPR_X, CPR_Y);
    TotalZBal1 += (((long)Atan4*1800) / 31415) -900; //Rotate balance circle 90 deg
    
    GetATan2 (CPR_Z, CPR_Y);
    TotalXBal1 += (((long)Atan4*1800) / 31415) - 900; //Rotate balance circle 90 deg

#ifdef DEBUG_BODYCALCS
    if ((TravelLengthX != 0) || (TravelLengthZ !=0) )
        DBGPrintf("BCOL: %d %d %d %u :: %ld %ld %ld :: %ld %ld %ld\n\r", PosX, PosZ, PosY, BalLegNr,
                TotalTransY, TotalTransZ, TotalTransX, TotalYBal1, TotalZBal1, TotalXBal1);
            
#endif 

}  
//--------------------------------------------------------------------
//[BalanceBody]
void BalanceBody(void)
{
    TotalTransZ = TotalTransZ/BalanceDivFactor ;
    TotalTransX = TotalTransX/BalanceDivFactor;
    TotalTransY = TotalTransY/BalanceDivFactor;

    if (TotalYBal1 > 0)        //Rotate balance circle by +/- 180 deg
        TotalYBal1 -=  1800;
    else
        TotalYBal1 += 1800;
    
    if (TotalZBal1 < -1800)    //Compensate for extreme balance positions that causes owerflow
        TotalZBal1 += 3600;
    
    if (TotalXBal1 < -1800)    //Compensate for extreme balance positions that causes owerflow
        TotalXBal1 += 3600;
    
    //Balance rotation
    TotalYBal1 = -TotalYBal1/BalanceDivFactor;
    TotalXBal1 = -TotalXBal1/BalanceDivFactor;
    TotalZBal1 = TotalZBal1/BalanceDivFactor;

#ifdef DEBUG_BODYCALCS
    if ((TravelLengthX != 0) || (TravelLengthZ !=0) )
        DBGPrintf("BBody: %ld %ld %ld :: %ld %ld %ld\n\r", TotalTransY, TotalTransZ,TotalTransX, 
            TotalYBal1, TotalZBal1,TotalXBal1);
#endif

}


//--------------------------------------------------------------------
//[GETSINCOS] Get the sinus and cosinus from the angle +/- multiple circles
//AngleDeg1     - Input Angle in degrees
//sin4        - Output Sinus of AngleDeg
//cos4          - Output Cosinus of AngleDeg
void GetSinCos(short AngleDeg1)
{
    short        ABSAngleDeg1;    //Absolute value of the Angle in Degrees, decimals = 1
    //Get the absolute value of AngleDeg
    if (AngleDeg1 < 0)
        ABSAngleDeg1 = AngleDeg1 *-1;
    else
          ABSAngleDeg1 = AngleDeg1;
    
    //Shift rotation to a full circle of 360 deg -> AngleDeg // 360
    if (AngleDeg1 < 0)    //Negative values
        AngleDeg1 = 3600-(ABSAngleDeg1-(3600*(ABSAngleDeg1/3600)));
    else                //Positive values
        AngleDeg1 = ABSAngleDeg1-(3600*(ABSAngleDeg1/3600));
    
    if (AngleDeg1>=0 && AngleDeg1<=900)     // 0 to 90 deg
    {
        sin4 = GetSin[AngleDeg1/5];             // 5 is the presision (0.5) of the table
        cos4 = GetSin[(900-(AngleDeg1))/5];
    }     
        
    else if (AngleDeg1>900 && AngleDeg1<=1800)     // 90 to 180 deg
    {
        sin4 = GetSin[(900-(AngleDeg1-900))/5]; // 5 is the presision (0.5) of the table    
        cos4 = -GetSin[(AngleDeg1-900)/5];            
    }    
    else if (AngleDeg1>1800 && AngleDeg1<=2700) // 180 to 270 deg
    {
        sin4 = -GetSin[(AngleDeg1-1800)/5];     // 5 is the presision (0.5) of the table
        cos4 = -GetSin[(2700-AngleDeg1)/5];
    }    

    else if(AngleDeg1>2700 && AngleDeg1<=3600) // 270 to 360 deg
    {
        sin4 = -GetSin[(3600-AngleDeg1)/5]; // 5 is the presision (0.5) of the table    
        cos4 = GetSin[(AngleDeg1-2700)/5];            
    }
}    


//--------------------------------------------------------------------
//(GETARCCOS) Get the sinus and cosinus from the angle +/- multiple circles
//cos4        - Input Cosinus
//AngleRad4     - Output Angle in AngleRad4
long GetArcCos(short cos4)
{
    boolean NegativeValue/*:1*/;    //If the the value is Negative
    //Check for negative value
    if (cos4<0)
    {
        cos4 = -cos4;
        NegativeValue = 1;
    }
    else
        NegativeValue = 0;
    
    //Limit cos4 to his maximal value
    cos4 = min(cos4,c4DEC);
    
    if ((cos4>=0) && (cos4<9000))
    {
        AngleRad4 = GetACos[cos4/79];                 //79=table resolution (1/127);
        AngleRad4 = ((long)AngleRad4*616)/c1DEC;            //616=acos resolution (pi/2/255) ;
    }    
    else if ((cos4>=9000) && (cos4<9900))
    {
        AngleRad4 = GetACos[(cos4-9000)/8+114];         // 8=table resolution (0.1/127), 114 start address 2nd bytetable range 
        AngleRad4 = (long)((long)AngleRad4*616)/c1DEC;             //616=acos resolution (pi/2/255) 
    }
    else if ((cos4>=9900) && (cos4<=10000))
    {
        AngleRad4 = GetACos[(cos4-9900)/2+227];         //2=table resolution (0.01/64), 227 start address 3rd bytetable range 
        AngleRad4 = (long)((long)AngleRad4*616)/c1DEC;             //616=acos resolution (pi/2/255) 
    }
       
    //Add negative sign
    if (NegativeValue)
        AngleRad4 = 31416 - AngleRad4;

    return AngleRad4;
}    

unsigned long isqrt32 (unsigned long n) //
{
        unsigned long root;
        unsigned long remainder;
        unsigned long  place;

        root = 0;
        remainder = n;
        place = 0x40000000; // OR place = 0x4000; OR place = 0x40; - respectively

        while (place > remainder)
        place = place >> 2;
        while (place)
        {
                if (remainder >= root + place)
                {
                        remainder = remainder - root - place;
                        root = root + (place << 1);
                }
                root = root >> 1;
                place = place >> 2;
        }
        return root;
}


//--------------------------------------------------------------------
//(GETATAN2) Simplyfied ArcTan2 function based on fixed point ArcCos
//ArcTanX         - Input X
//ArcTanY         - Input Y
//ArcTan4          - Output ARCTAN2(X/Y)
//XYhyp2            - Output presenting Hypotenuse of X and Y
short GetATan2 (short AtanX, short AtanY)
{
    XYhyp2 = isqrt32(((long)AtanX*AtanX*c4DEC) + ((long)AtanY*AtanY*c4DEC));
    GetArcCos (((long)AtanX*(long)c6DEC) /(long) XYhyp2);
    
    if (AtanY < 0)                // removed overhead... Atan4 = AngleRad4 * (AtanY/abs(AtanY));  
        Atan4 = -AngleRad4;
    else
        Atan4 = AngleRad4;
#if 0 //fdef DEBUG_PINCALCS    
    if (g_fHexOn)
    {
        DBGPrintf(" GAT(%d, %d)", XYhyp2, Atan4); 
    }
#endif    
    return Atan4;
}    
    
//--------------------------------------------------------------------
//(BODY INVERSE KINEMATICS) 
//BodyRotX         - Global Input pitch of the body 
//BodyRotY         - Global Input rotation of the body 
//BodyRotZ         - Global Input roll of the body 
//RotationY         - Input Rotation for the gait 
//PosX            - Input position of the feet X 
//PosZ            - Input position of the feet Z 
//SinB                  - Sin buffer for BodyRotX
//CosB               - Cos buffer for BodyRotX
//SinG                  - Sin buffer for BodyRotZ
//CosG               - Cos buffer for BodyRotZ
//BodyIKPosX         - Output Position X of feet with Rotation 
//BodyIKPosY         - Output Position Y of feet with Rotation 
//BodyIKPosZ         - Output Position Z of feet with Rotation
void BodyIK (short PosX, short PosZ, short PosY, short RotationY, byte BodyIKLeg) 
{
    short            SinA4;          //Sin buffer for BodyRotX calculations
    short            CosA4;          //Cos buffer for BodyRotX calculations
    short            SinB4;          //Sin buffer for BodyRotX calculations
    short            CosB4;          //Cos buffer for BodyRotX calculations
    short            SinG4;          //Sin buffer for BodyRotZ calculations
    short            CosG4;          //Cos buffer for BodyRotZ calculations
    short             CPR_X;            //Final X value for centerpoint of rotation
    short            CPR_Y;            //Final Y value for centerpoint of rotation
    short            CPR_Z;            //Final Z value for centerpoint of rotation

    //Calculating totals from center of the body to the feet 
    CPR_X = cOffsetX[BodyIKLeg]+PosX;
    CPR_Y = PosY + BodyRotOffsetY;         //Define centerpoint for rotation along the Y-axis
    CPR_Z = cOffsetZ[BodyIKLeg] + PosZ + BodyRotOffsetZ;

    //Successive global rotation matrix: 
    //Math shorts for rotation: Alfa [A] = Xrotate, Beta [B] = Zrotate, Gamma [G] = Yrotate 
    //Sinus Alfa = SinA, cosinus Alfa = cosA. and so on... 
    
    //First calculate sinus and cosinus for each rotation: 
    GetSinCos (BodyRotX1+TotalXBal1);
    SinG4 = sin4;
    CosG4 = cos4;
    
    GetSinCos (BodyRotZ1+TotalZBal1); 
    SinB4 = sin4;
    CosB4 = cos4;
    
    GetSinCos (BodyRotY1+(RotationY*c1DEC)+TotalYBal1) ;
    SinA4 = sin4;
    CosA4 = cos4;
    
    //Calcualtion of rotation matrix: 
      BodyIKPosX = ((long)CPR_X*c2DEC - ((long)CPR_X*c2DEC*CosA4/c4DEC*CosB4/c4DEC - (long)CPR_Z*c2DEC*CosB4/c4DEC*SinA4/c4DEC 
              + (long)CPR_Y*c2DEC*SinB4/c4DEC ))/c2DEC;
      BodyIKPosZ = ((long)CPR_Z*c2DEC - ( (long)CPR_X*c2DEC*CosG4/c4DEC*SinA4/c4DEC + (long)CPR_X*c2DEC*CosA4/c4DEC*SinB4/c4DEC*SinG4/c4DEC 
              + (long)CPR_Z*c2DEC*CosA4/c4DEC*CosG4/c4DEC - (long)CPR_Z*c2DEC*SinA4/c4DEC*SinB4/c4DEC*SinG4/c4DEC 
              - (long)CPR_Y*c2DEC*CosB4/c4DEC*SinG4/c4DEC ))/c2DEC;
      BodyIKPosY = ((long)CPR_Y  *c2DEC - ( (long)CPR_X*c2DEC*SinA4/c4DEC*SinG4/c4DEC - (long)CPR_X*c2DEC*CosA4/c4DEC*CosG4/c4DEC*SinB4/c4DEC 
              + (long)CPR_Z*c2DEC*CosA4/c4DEC*SinG4/c4DEC + (long)CPR_Z*c2DEC*CosG4/c4DEC*SinA4/c4DEC*SinB4/c4DEC 
              + (long)CPR_Y*c2DEC*CosB4/c4DEC*CosG4/c4DEC ))/c2DEC;

#ifdef DEBUG_PINCALCS
    if (g_fHexOn && (TravelLengthX || TravelLengthZ))
    {
        DBGPrintf("  CPR XYZ: %d %d %d G4: %d %d ",CPR_X, CPR_Y, CPR_Z, SinG4, CosG4);
        DBGPrintf("B4: %d %d A4: %d %d", SinB4, CosB4, SinA4, CosA4);
        DBGPrintf(" BodyIK(%d %d %d) =", BodyIKPosX, BodyIKPosY, BodyIKPosZ);
    }
#endif    
}  



//--------------------------------------------------------------------
//[LEG INVERSE KINEMATICS] Calculates the angles of the coxa, femur and tibia for the given position of the feet
//IKFeetPosX            - Input position of the Feet X
//IKFeetPosY            - Input position of the Feet Y
//IKFeetPosZ            - Input Position of the Feet Z
//IKSolution            - Output true if the solution is possible
//IKSolutionWarning     - Output true if the solution is NEARLY possible
//IKSolutionError    - Output true if the solution is NOT possible
//FemurAngle1           - Output Angle of Femur in degrees
//TibiaAngle1           - Output Angle of Tibia in degrees
//CoxaAngle1            - Output Angle of Coxa in degrees
//--------------------------------------------------------------------
void LegIK (short IKFeetPosX, short IKFeetPosY, short IKFeetPosZ, byte LegIKLegNr)
{
    unsigned long    IKSW2;            //Length between Shoulder and Wrist, decimals = 2
    unsigned long    IKA14;            //Angle of the line S>W with respect to the ground in radians, decimals = 4
    unsigned long    IKA24;            //Angle of the line S>W with respect to the femur in radians, decimals = 4
    short            IKFeetPosXZ;    //Diagonal direction from Input X and Z
    long            Temp1;            
    long            Temp2;            
    long            T3;
    
    //Calculate IKCoxaAngle and IKFeetPosXZ
    GetATan2 (IKFeetPosX, IKFeetPosZ);
    CoxaAngle1[LegIKLegNr] = (((long)Atan4*180) / 3141) + cCoxaAngle1[LegIKLegNr];
    
    //Length between the Coxa and tars [foot]
    IKFeetPosXZ = XYhyp2/c2DEC;
    
    //Using GetAtan2 for solving IKA1 and IKSW
    //IKA14 - Angle between SW line and the ground in radians
    IKA14 = GetATan2 (IKFeetPosY, IKFeetPosXZ-cCoxaLength);
    
    //IKSW2 - Length between femur axis and tars
    IKSW2 = XYhyp2;
    
    //IKA2 - Angle of the line S>W with respect to the femur in radians
    Temp1 = ((((long)cFemurLength*cFemurLength) - ((long)cTibiaLength*cTibiaLength))*c4DEC + ((long)IKSW2*IKSW2));
    Temp2 = (long)(2*cFemurLength)*c2DEC * (unsigned long)IKSW2;
    T3 = Temp1 / (Temp2/c4DEC);
    IKA24 = GetArcCos (T3 );
#if 0 //def DEBUG_PINCALCS
    if (g_fHexOn && (TravelLengthX < - 30))
    {
        DBGPrintf("T1: %ld T2: %ld: T3 %ld AR: %d ", Temp1, Temp2, T3, AngleRad4);
    }
#endif    
    //IKFemurAngle
    FemurAngle1[LegIKLegNr] = -(long)(IKA14 + IKA24) * 180 / 3141 + 900;

    //IKTibiaAngle
    Temp1 = ((((long)cFemurLength*cFemurLength) + ((long)cTibiaLength*cTibiaLength))*c4DEC - ((long)IKSW2*IKSW2));
    Temp2 = (2*cFemurLength*cTibiaLength);
    GetArcCos (Temp1 / Temp2);
    TibiaAngle1[LegIKLegNr] = -(900-(long)AngleRad4*180/3141);
#if 0 //def DEBUG_PINCALCS
    if (g_fHexOn && (TravelLengthX || TravelLengthZ)) {
        DBGPrintf("T1: %lu T2: %lu: AR: %d ", Temp1, Temp2, AngleRad4);
    }
#endif

    //Set the Solution quality    
    if(IKSW2 < (cFemurLength+cTibiaLength-30)*c2DEC)
        IKSolution = 1;
    else
    {
        if(IKSW2 < (cFemurLength+cTibiaLength)*c2DEC) 
            IKSolutionWarning = 1;
        else
            IKSolutionError = 1    ;
    }
#ifdef DEBUG_PINCALCS
    if (g_fHexOn && (TravelLengthX || TravelLengthZ))
    {
        //DBGPrintf("POS(%d, %d, %d) IKA1: %d, IKSW: %d, IKA2 %d =>", IKFeetPosX, IKFeetPosY, IKFeetPosZ, IKA14, IKSW2,IKA24);
        DBGPrintf(" CA: %d FA: %d, TA: %d SWE: %x%x%x\n\r", 
                CoxaAngle1[LegIKLegNr],FemurAngle1[LegIKLegNr], TibiaAngle1[LegIKLegNr], IKSolution, IKSolutionWarning, IKSolutionError  );
    }
#endif    
}


//--------------------------------------------------------------------
//[CHECK ANGLES] Checks the mechanical limits of the servos
//--------------------------------------------------------------------
void CheckAngles(void)
{

    for (LegIndex = 0; LegIndex <=5; LegIndex++)
    {
        CoxaAngle1[LegIndex]  = min(max(CoxaAngle1[LegIndex], cCoxaMin1[LegIndex]), cCoxaMax1[LegIndex]);
        FemurAngle1[LegIndex] = min(max(FemurAngle1[LegIndex], cFemurMin1[LegIndex]),cFemurMax1[LegIndex]);
        TibiaAngle1[LegIndex] = min(max(TibiaAngle1[LegIndex], cTibiaMin1[LegIndex]),cTibiaMax1[LegIndex]);
    }
}


//--------------------------------------------------------------------
//[SERVO DRIVER Start] Updates the positions of the servos - This outputs
//         as much of the command as we can without committing it.  This
//         allows us to once the previous update was completed to quickly 
//        get the next command to start
//--------------------------------------------------------------------
void ServoDriverStart()
{
#ifdef cSSC_BINARYMODE
    byte    abOut[30];
    byte    *pbOut;
    word    wCoxaSSCV;        // Coxa value in SSC units
    word    wFemurSSCV;        //
    word    wTibiaSSCV;        //
#endif

    //Update Right Legs

    AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
//    disable(TIMERAINT);


#ifdef DEBUG_GAITS
    if (g_fHexOn && (TravelLengthX || TravelLengthZ))
        DBGPrintf("%d %d %d:", (short)GaitType, (short)GaitStep, (short)BalanceMode);
#endif
  

    for (LegIndex = 0; LegIndex <= 2; LegIndex++) {

#ifdef DEBUG_GAITS
        if (g_fHexOn && (TravelLengthX || TravelLengthZ)) {
            DBGPrintf("%02d=(%d)%04d ", (short)cCoxaPin[LegIndex], CoxaAngle1[LegIndex], wCoxaSSCV);
            DBGPrintf("%02d=(%d)%04d ", (short)cFemurPin[LegIndex], FemurAngle1[LegIndex], wFemurSSCV);
            DBGPrintf("%02d=(%d)%04d ", (short)cTibiaPin[LegIndex], TibiaAngle1[LegIndex], wTibiaSSCV);
        }    
#endif    

        SSCPrintf("#%dP%d", cCoxaPin[LegIndex],  ((long)(-CoxaAngle1[LegIndex] +900))*1000/1059+650);
        SSCPrintf("#%dP%d", cFemurPin[LegIndex], ((long)(-FemurAngle1[LegIndex]+900))*1000/1059+650);
        SSCPrintf("#%dP%d", cTibiaPin[LegIndex], ((long)(-TibiaAngle1[LegIndex]+900))*1000/1059+650);

    }  


      
      //Update Left Legs
    for (LegIndex = 3; LegIndex <= 5; LegIndex++)
    {

#ifdef DEBUG_GAITS
        if (g_fHexOn && (TravelLengthX || TravelLengthZ)) {
            DBGPrintf("%02d=(%d)%04d ", (short)cCoxaPin[LegIndex], CoxaAngle1[LegIndex], wCoxaSSCV);
            DBGPrintf("%02d=(%d)%04d ", (short)cFemurPin[LegIndex], FemurAngle1[LegIndex], wFemurSSCV);
            DBGPrintf("%02d=(%d)%04d ", (short)cTibiaPin[LegIndex], TibiaAngle1[LegIndex], wTibiaSSCV);
        }            
#endif    

        SSCPrintf("#%dP%d", cCoxaPin[LegIndex], ((long)(CoxaAngle1[LegIndex] +900))*1000/1059+650);
        SSCPrintf("#%dP%d", cFemurPin[LegIndex], ((long)(FemurAngle1[LegIndex]+900))*1000/1059+650);
        SSCPrintf("#%dP%d", cTibiaPin[LegIndex], ((long)(TibiaAngle1[LegIndex]+900))*1000/1059+650);

    }  

      

//    enable(TIMERAINT);
    AllowControllerInterrupts(true);    // Ok for hserial again...
}


//--------------------------------------------------------------------
//[SERVO DRIVER Start] Updates the positions of the servos - This outputs
//         as much of the command as we can without committing it.  This
//         allows us to once the previous update was completed to quickly 
//        get the next command to start
//--------------------------------------------------------------------
void ServoDriverCommit()
{
#ifdef cSSC_BINARYMODE
    byte    abOut[3];
#endif
    
    AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
//    disable(TIMERAINT);

#ifdef cSSC_BINARYMODE
    abOut[0] = 0xA1;
    abOut[1] = SSCTime >> 8;
    abOut[2] = SSCTime & 0xff;
    //SSCSerial.write((byte*)(void*)abOut, 3);        // Let the command begin
#else
      //Send <CR>
    SSCPrintf("T%d\n", SSCTime);
#endif
    Serial1.println();
#ifdef DEBUG_GAITS
    if (g_fHexOn && (TravelLengthX || TravelLengthZ)) {
	DBGPrintf("T:%d\n\r", SSCTime);
    }
#endif	
	


//    enable(TIMERAINT);
    AllowControllerInterrupts(true);    

    PrevSSCTime = SSCTime;
}
//--------------------------------------------------------------------
// SmoothControl (From Zenta) -  This function makes the body 
//            rotation and translation much smoother while walking
//--------------------------------------------------------------------
short SmoothControl (short CtrlMoveInp, short CtrlMoveOut, byte CtrlDivider)
{
    if (fWalking)
    {
        if (CtrlMoveOut < (CtrlMoveInp - 4))
              return CtrlMoveOut + abs((CtrlMoveOut - CtrlMoveInp)/CtrlDivider);
        else if (CtrlMoveOut > (CtrlMoveInp + 4))
              return CtrlMoveOut - abs((CtrlMoveOut - CtrlMoveInp)/CtrlDivider);
    }

    return CtrlMoveInp;
}

//--------------------------------------------------------------------
//[FREE SERVOS] Frees all the servos
//--------------------------------------------------------------------
void FreeServos(void)
{
    AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
//  disable(TIMERAINT);
    for (LegIndex = 0; LegIndex < 32; LegIndex+=4) {
        SSCPrintf("#%dP0#%dP0#%dP0#%dP0", LegIndex, LegIndex+1, LegIndex+2, LegIndex+3);
    }
    SSCSerial.print("T200");
    Serial1.println();
//  enable(TIMERAINT);
    AllowControllerInterrupts(true);    
}


//--------------------------------------------------------------------
// Why are we faulting?
//--------------------------------------------------------------------


void PrintSystemStuff(void)            // Try to see why we fault...
{

}

// BUGBUG:: Move to some library...
//==============================================================================
//    SoundNoTimer - Quick and dirty tone function to try to output a frequency
//            to a speaker for some simple sounds.
//==============================================================================
void SoundNoTimer(uint8_t _pin, unsigned long duration,  unsigned int frequency)
{
    volatile uint8_t *pin_port;
    volatile uint8_t pin_mask;
    long toggle_count = 0;
    long lusDelayPerHalfCycle;
    
    // Set the pinMode as OUTPUT
    pinMode(_pin, OUTPUT);

    pin_port = portOutputRegister(digitalPinToPort(_pin));
    pin_mask = digitalPinToBitMask(_pin);
    
    toggle_count = 2 * frequency * duration / 1000;
    lusDelayPerHalfCycle = 1000000L/(frequency * 2);
    
    // if we are using an 8 bit timer, scan through prescalars to find the best fit
    while (toggle_count--) {
        // toggle the pin
        *pin_port ^= pin_mask;
        
        // delay a half cycle
        delayMicroseconds(lusDelayPerHalfCycle);
    }    
    *pin_port &= ~(pin_mask);  // keep pin low after stop

}

void MSound(uint8_t _pin, byte cNotes, ...)
{
    va_list ap;
    unsigned int uDur;
    unsigned int uFreq;
    va_start(ap, cNotes);

    while (cNotes > 0) {
        uDur = va_arg(ap, unsigned int);
        uFreq = va_arg(ap, unsigned int);
        SoundNoTimer(_pin, uDur, uFreq);
        cNotes--;
    }
    va_end(ap);
}

// BUGBUG Place holder
int DBGPrintf(const char *format, ...)
{
    char szTemp[80];
    int ich;
    va_list ap;
    va_start(ap, format);
    ich = vsprintf(szTemp, format, ap);
    DBGSerial.write((byte*)szTemp, ich);
    va_end(ap);
}

// BUGBUG Place holder
int SSCPrintf(const char *format, ...)
{
    char szTemp[80];
    int ich;
    va_list ap;
    va_start(ap, format);
    ich = vsprintf(szTemp, format, ap);
    SSCSerial.write((byte*)szTemp, ich);
    va_end(ap);
}

// Quick and dirty helper function to read so many bytes in from the SSC with a timeout and an end of character marker...
int SSCRead (byte* pb, int cb, word wTimeout, word wEOL)
{
    int ich;
    byte* pbIn = pb;
    unsigned long ulTimeLastChar = micros();
    
    while (cb) {
        while ((ich = SSCSerial.read()) == -1) {
            // check for timeout
            if ((word)(micros()-ulTimeLastChar) > wTimeout)
                return (int)(pb-pbIn);
        }
        *pb++ = (byte)ich;
        cb--;

        if ((word)ich == wEOL)
            break;    // we matched so get out of here.
        ulTimeLastChar = micros();    // update to say we received something
    }
    return (int)(pb-pbIn);
}


