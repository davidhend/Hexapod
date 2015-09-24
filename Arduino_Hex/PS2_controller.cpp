//Project Lynxmotion Phoenix
//Description: Phoenix, control file.
//The control input subroutine for the phoenix software is placed in this file.
//Can be used with V2.0 and above
//Configuration version: V1.0
//Date: 25-10-2009
//Programmer: Jeroen Janssen (aka Xan)
//             Kurt Eckhardt (aka KurtE) - converted to c ported to Arduino...
//
//Hardware setup: PS2 version
// 
//NEW IN V1.0
//- First Release
//
//Walk method 1:
//- Left StickWalk/Strafe
//- Right StickRotate
//
//Walk method 2:
//- Left StickDisable
//- Right StickWalk/Rotate
//
//
//PS2 CONTROLS:
//[Common Controls]
//- StartTurn on/off the bot
//- L1Toggle Shift mode
//- L2Toggle Rotate mode
//- CircleToggle Single leg mode
//   - Square        Toggle Balance mode
//- TriangleMove body to 35 mm from the ground (walk pos) 
//and back to the ground
//- D-Pad upBody up 10 mm
//- D-Pad downBody down 10 mm
//- D-Pad leftdecrease speed with 50mS
//- D-Pad rightincrease speed with 50mS
//
//[Walk Controls]
//- selectSwitch gaits
//- Left Stick(Walk mode 1) Walk/Strafe
// (Walk mode 2) Disable
//- Right Stick(Walk mode 1) Rotate, 
//(Walk mode 2) Walk/Rotate
//- R1Toggle Double gait travel speed
//- R2Toggle Double gait travel length
//
//[Shift Controls]
//- Left StickShift body X/Z
//- Right StickShift body Y and rotate body Y
//
//[Rotate Controls]
//- Left StickRotate body X/Z
//- Right StickRotate body Y
//
//[Single leg Controls]
//- selectSwitch legs
//- Left StickMove Leg X/Z (relative)
//- Right StickMove Leg Y (absolute)
//- R2Hold/release leg position
//
//[GP Player Controls]
//- selectSwitch Sequences
//- R2Start Sequence
//
//====================================================================
// [Include files]
#include <WProgram.h> 
#include "Hex_Globals.h"
#ifdef USEPS2
#include <PS2X_lib.h>

//[CONSTANTS]
#define WALKMODE          0
#define TRANSLATEMODE     1
#define ROTATEMODE        2
#define SINGLELEGMODE     3
#define GPPLAYERMODE      4


#define cTravelDeadZone 4 //The deadzone for the analog input from the remote


//=============================================================================
// Global - Local to this file only...
//=============================================================================
// BUGBUG: Move to PS2 support
PS2X ps2x; // create PS2 Controller Class


static unsigned    g_BodyYOffset; 
static char        g_BodyYSift;
static byte        ControlMode;
static bool        DoubleHeightOn;
static bool        DoubleTravelOn;
static bool        WalkMethod;

extern void MSound(uint8_t _pin, byte cNotes, ...);
extern int DBGPrintf(const char *format, ...);

//==============================================================================
// This is The function that is called by the Main program to initialize
//the input controller, which in this case is the PS2 controller
//process any commands.
//==============================================================================

// If both PS2 and XBee are defined then we will become secondary to the xbee
#ifdef USEXBEE
void InitPS2Controller(void)
#else
void InitController(void)
#endif
{
    int error;
    
    delay(300);

    //error = ps2x.config_gamepad(57, 55, 56, 54);  // Setup gamepad (clock, command, attention, data) pins
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT);  // Setup gamepad (clock, command, attention, data) pins
 
    if(error == 0){
       DBGSerial.println("Found Controller, configured successful");
    }
   
    else if(error == 1)
       DBGSerial.println("No controller found");
   
    else if(error == 2)
       DBGSerial.println("Controller found but not accepting commands.");
    
    g_BodyYOffset = 0;
    g_BodyYSift = 0;
}

//==============================================================================
// This function is called by the main code to tell us when it is about to
// do a lot of bit-bang outputs and it would like us to minimize any interrupts
// that we do while it is active...
//==============================================================================
#ifndef USEXBEE// XBEE will provide this if defined...
void AllowControllerInterrupts(bool fAllow)
{
// We don't need to do anything...
}
#endif

//==============================================================================
// This is The main code to input function to read inputs from the PS2 and then
//process any commands.
//==============================================================================
#ifdef USEXBEE
void PS2ControlInput(void)
#else
void ControlInput(void)
#endif
{
    // Then try to receive a packet of information from the PS2.
    // Then try to receive a packet of information from the PS2.
    ps2x.read_gamepad();          //read controller and set large motor to spin at 'vibrate' speed

    // Wish the library had a valid way to verify that the read_gamepad succeeded... Will hack for now
    if ((ps2x.Analog(1) & 0xf0) == 0x70) {
        // In an analog mode so should be OK...

        if (ps2x.ButtonPressed(PSB_START)) {// OK lets try "0" button for Start. 
            DBGPrintf("Start button Pressed\r");
            if (g_fHexOn) {
                //Turn off
                BodyPosX = 0;
                BodyPosY = 0;
                BodyPosZ = 0;
                BodyRotX1 = 0;
                BodyRotY1 = 0;
                BodyRotZ1 = 0;
                TravelLengthX = 0;
                TravelLengthZ = 0;
                TravelRotationY = 0;
                g_BodyYOffset = 0;
                g_BodyYSift = 0;
                SelectedLeg = 255;
                g_fHexOn = 0;
            } else {
                //Turn on
                g_fHexOn = 1;
            }
        }

        if (g_fHexOn) {
            // [SWITCH MODES]
    
             //Translate mode
            if (ps2x.ButtonPressed(PSB_L1)) {// L1 Button Test
                MSound(SOUND_PIN, 1, 50, 2000);  //sound SOUND_PIN, [50\4000]
                if (ControlMode != TRANSLATEMODE )
                    ControlMode = TRANSLATEMODE;
                else {
                    if (SelectedLeg==255) 
                        ControlMode = WALKMODE;
                    else
                        ControlMode = SINGLELEGMODE;
                }
            }
  
            //Rotate mode
            if (ps2x.ButtonPressed(PSB_L2)) {    // L2 Button Test
                MSound(SOUND_PIN, 1, 50, 2000);  //sound SOUND_PIN, [50\4000]
                if (ControlMode != ROTATEMODE)
                    ControlMode = ROTATEMODE;
                else {
                    if (SelectedLeg == 255) 
                        ControlMode = WALKMODE;
                    else
                        ControlMode = SINGLELEGMODE;
                }
            }
    
            //Single leg mode fNO
            if (ps2x.ButtonPressed(PSB_CIRCLE)) {// O - Circle Button Test
                if (abs(TravelLengthX)<cTravelDeadZone && abs(TravelLengthZ)<cTravelDeadZone 
                        && abs(TravelRotationY*2)<cTravelDeadZone )   {
                    //Sound SOUND_PIN,[50\4000]
                    if (ControlMode != SINGLELEGMODE) {
                        ControlMode = SINGLELEGMODE;
                            if (SelectedLeg == 255)  //Select leg if none is selected
                                SelectedLeg=cRF; //Startleg
                    } else {
                        ControlMode = WALKMODE;
                        SelectedLeg=255;
                    }
                }
            }      

            // GP Player Mode X
            if (ps2x.ButtonPressed(PSB_CROSS)) { // X - Cross Button Test
                MSound(SOUND_PIN, 1, 50, 2000);  //sound SOUND_PIN, [50\4000]
                if (ControlMode != GPPLAYERMODE) {
                    ControlMode = GPPLAYERMODE;
                    GPSeq=0;
                } else
                    ControlMode = WALKMODE;
            }

            //[Common functions]
            //Switch Balance mode on/off 
            if (ps2x.ButtonPressed(PSB_SQUARE)) { // Square Button Test
                BalanceMode = !BalanceMode;
                if (BalanceMode) {
                    MSound(SOUND_PIN, 1, 250, 1500);  //sound SOUND_PIN, [250\3000]
                } else {
                    MSound(SOUND_PIN, 2, 100, 2000, 50, 4000);
                }
            }

            //Stand up, sit down  
            if (ps2x.ButtonPressed(PSB_TRIANGLE)) { // Triangle - Button Test
                if (g_BodyYOffset>0) 
                    g_BodyYOffset = 0;
                else
                    g_BodyYOffset = 35;
            }

            if (ps2x.ButtonPressed(PSB_PAD_UP))// D-Up - Button Test
                g_BodyYOffset = g_BodyYOffset+10;

            if (ps2x.ButtonPressed(PSB_PAD_DOWN))// D-Down - Button Test
                g_BodyYOffset = g_BodyYOffset-10;

            if (ps2x.ButtonPressed(PSB_PAD_RIGHT)) { // D-Right - Button Test
                if (SpeedControl>0) {
                    SpeedControl = SpeedControl - 50;
                    MSound(SOUND_PIN, 1, 50, 2000);  //sound SOUND_PIN, [50\4000]
                }
            }

            if (ps2x.ButtonPressed(PSB_PAD_LEFT)) { // D-Left - Button Test
                if (SpeedControl<2000 ) {
                    SpeedControl = SpeedControl + 50;
                    MSound(SOUND_PIN, 1, 50, 2000);  //sound SOUND_PIN, [50\4000]
                }
            }

            //[Walk functions]
            if (ControlMode == WALKMODE) {
                //Switch gates
                if (ps2x.ButtonPressed(PSB_SELECT)            // Select Button Test
                        && abs(TravelLengthX)<cTravelDeadZone //No movement
                        && abs(TravelLengthZ)<cTravelDeadZone 
                        && abs(TravelRotationY*2)<cTravelDeadZone  ) {
                    if (GaitType<7) {
                        MSound(SOUND_PIN, 1, 50, 2000);  //sound SOUND_PIN, [50\4000]
                        GaitType = GaitType+1;
                    } else {
                        MSound (9, 2, 50, 2000, 50, 2250); 
                        GaitType = 0;
                    }
                    GaitSelect();
                }
  
                //Double leg lift height
                if (ps2x.ButtonPressed(PSB_R1)) { // R1 Button Test
                    MSound(SOUND_PIN, 1, 50, 2000);  //sound SOUND_PIN, [50\4000]
                    DoubleHeightOn = !DoubleHeightOn;
                    if (DoubleHeightOn)
                        LegLiftHeight = 80;
                    else
                        LegLiftHeight = 50;
                }
  
                //Double Travel Length
                if (ps2x.ButtonPressed(PSB_R2)) {// R2 Button Test
                    MSound (SOUND_PIN, 1, 50, 2000);  //sound SOUND_PIN, [50\4000]
                    DoubleTravelOn = !DoubleTravelOn;
                }
  
                // Switch between Walk method 1 && Walk method 2
                if (ps2x.ButtonPressed(PSB_R3)) { // R3 Button Test
                    MSound (SOUND_PIN, 50, 2000);  //sound SOUND_PIN, [50\4000]
                    WalkMethod = !WalkMethod;
                }
  
                //Walking
                if (WalkMethod)  //(Walk Methode) 
                    TravelLengthZ = (ps2x.Analog(PSS_RY)-128); //Right Stick Up/Down  

                else {
                    TravelLengthX = -(ps2x.Analog(PSS_LX) - 128);
                    TravelLengthZ = (ps2x.Analog(PSS_LY) - 128);
                }

                if (!DoubleTravelOn) {  //(Double travel length)
                    TravelLengthX = TravelLengthX/2;
                    TravelLengthZ = TravelLengthZ/2;
                }

                TravelRotationY = -(ps2x.Analog(PSS_RX) - 128)/4; //Right Stick Left/Right 
            }

            //[Translate functions]
            //g_BodyYSift = 0
            if (ControlMode == TRANSLATEMODE) {
                BodyPosX = (ps2x.Analog(PSS_LX) - 128)/2;
                BodyPosZ = -(ps2x.Analog(PSS_LY) - 128)/3;
                BodyRotY1 = (ps2x.Analog(PSS_RX) - 128)*2;
                g_BodyYSift = (-(ps2x.Analog(PSS_RY) - 128)/2);
            }

            //[Rotate functions]
            if (ControlMode == ROTATEMODE) {
                BodyRotX1 = (ps2x.Analog(PSS_LY) - 128);
                BodyRotY1 = (ps2x.Analog(PSS_RX) - 128)*2;
                BodyRotZ1 = (ps2x.Analog(PSS_LX) - 128);
                g_BodyYSift = (-(ps2x.Analog(PSS_RY) - 128)/2);
            }

            //[Single leg functions]
            if (ControlMode == SINGLELEGMODE) {
                //Switch leg for single leg control
                if (ps2x.ButtonPressed(PSB_SELECT)) { // Select Button Test
                    MSound (SOUND_PIN, 1, 50, 2000);  //sound SOUND_PIN, [50\4000]
                    if (SelectedLeg<5)
                        SelectedLeg = SelectedLeg+1;
                    else
                        SelectedLeg=0;
                }

                SLLegX= (ps2x.Analog(PSS_LX) - 128)/2; //Left Stick Right/Left
                SLLegY= (ps2x.Analog(PSS_RY) - 128)/10; //Right Stick Up/Down
                SLLegZ = (ps2x.Analog(PSS_LY) - 128)/2; //Left Stick Up/Down

                // Hold single leg in place
                if (ps2x.ButtonPressed(PSB_R2)) { // R2 Button Test
                    MSound (SOUND_PIN, 1, 50, 2000);  //sound SOUND_PIN, [50\4000]
                    fSLHold = !fSLHold;
                }
            }
  
            //[GPPlayer functions]
            if (ControlMode == GPPLAYERMODE) {

                //Switch between sequences
                if (ps2x.ButtonPressed(PSB_SELECT)) { // Select Button Test
                    if (GPStart==0 ) {
                        if (GPSeq < 5) {  //Max sequence
                            MSound (SOUND_PIN, 1, 50, 1500);  //sound SOUND_PIN, [50\3000]
                            GPSeq = GPSeq+1;
                        } else {
                            MSound (SOUND_PIN, 2, 50, 2000, 50, 2250);//Sound SOUND_PIN,[50\4000, 50\4500]
                            GPSeq=0;
                        }
                    }
                }
                //Start Sequence
                if (ps2x.ButtonPressed(PSB_R2))// R2 Button Test
                    GPStart=1;
            }

            //Calculate walking time delay
            InputTimeDelay = 128 - max(max(abs(ps2x.Analog(PSS_LX) - 128), abs(ps2x.Analog(PSS_LY) - 128)), abs(ps2x.Analog(PSS_RX) - 128));
        }
  
        //Calculate BodyPosY
        BodyPosY = max(g_BodyYOffset + g_BodyYSift,  0);
    }
}

#endif //USEPS2


