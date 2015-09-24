//#define DEBUG

//Project Lynxmotion Phoenix
//Description: Phoenix, control file.
//  The control input subroutine for the phoenix software is placed in this file.
//  Can be used with V2.0 and above
//Configuration version: V1.0
//Hardware setup: DIY XBee
//
//NEW IN V1.0
// - First Release
//
//
//DIY CONTROLS:
// - Left Stick (WALKMODE) Body Height / Rotate
//     (Translate Mode) Body Height / Rotate body Y 
//     (Rotate Mode) Body Height / Rotate body Y
//     (Single leg Mode) Move Tars Y 
//
// - Right Stick (WALKMODE) Walk/Strafe
//      (Translate Mode) Translate body X/Z
//     (Rotate Mode) Rotate body X/Z
//     (Single leg Mode) Move Tars X/Z
//
//  - Left Slider Speed
// - Right Slider Leg Lift Height
//
// - A    Walk Mode
// - B    Translate Mode
// - C    Rotate Mode
// - D    Single Leg Mode
// - E    Balance Mode on/off
//
// - 0    Turn on/off the bot
//
// - 1-8   (Walk mode) Switch gaits
// - 1-6   (Single leg mode) Switch legs
//
//====================================================================

#include <WProgram.h> 
#include "Hex_Globals.h"

#ifdef USEXBEE
#include "diyxbee.h"

//=============================================================================
// Constants
//=============================================================================
//[CONSTANTS]
#define WALKMODE          0
#define TRANSLATEMODE     1
#define ROTATEMODE        2
#define SINGLELEGMODE     3
#define GPPLAYERMODE      4

#define cTravelDeadZone 4  //The deadzone for the analog input from the remote

//=============================================================================
// Global - Local to this file only...
//=============================================================================

DIYPACKET  g_diyp;
DIYPACKET  g_diypPrev;

extern "C" {
static const char  *s_asGateNames[] = {"Ripple 6", "Ripple 12", "Quadripple 9",
                "Tripod 4", "Tripod 6", "Tripod 8", "Wave 12", "Wave 18"};
}
static unsigned        g_BodyYOffset; 
static char            g_BodyYSift; 
static byte            bXBeeControlMode;

#ifdef USEPS2
byte        g_bWhichControl;  // Which input device are we currently using?
#define     WC_UNKNOWN  0      // Not sure yet
#define     WC_PS2   1      // Using PS2 to control robot
#define     WC_XBEE   2      // we are currently using the XBee to control the robot
#endif


// Some externals, probably should move somewhere...
#ifdef USEPS2
extern void InitPS2Controller(void);
extern void XBeeControlInput(void);
extern void PS2ControlInput(void);
#endif

#ifdef DEBUG
extern boolean g_fDebugOutput;
#endif


//==============================================================================
// This is The function that is called by the Main program to initialize
//   the input controller, which in this case is the PS2 controller
//   process any commands.
//==============================================================================
void InitController(void)
{
#ifdef USEPS2
    // We allow both the PS2 and XBee to control us so if both are defined call
    // the renamed PS2 controller Init code...
    g_bWhichControl = WC_UNKNOWN;  //start off not knowning which controller we will use

    InitPS2Controller();
#endif 

    // Lets try to initialize the XBEE to use
    InitXBee();

     
    // Clear any stuff left in the buffer
    delay(20);
    ClearXBeeInputBuffer();
    
    word wMy = GetXBeeMY();
    DBGPrintf("XBee My: %x\n\r", wMy);
    
    g_BodyYOffset = 0;
    g_BodyYSift = 0;
    bXBeeControlMode = WALKMODE;
}


//==============================================================================
// This function is called by the main code to tell us when it is about to
// do a lot of bit-bang outputs and it would like us to minimize any interrupts
// that we do while it is active...
//==============================================================================
void AllowControllerInterrupts(boolean fAllow)
{
}


//==============================================================================
// [ControlInput] - If Xbee and PS2 are defined this function will try to decide
//   which one should be in control.  If neither is in contorl it will
//   call both and in the PS2 case it will use it, if the user hits the
//   start button on the PS2... 
//   
//==============================================================================

#ifdef USEPS2
void ControlInput(void)
{
    // if we are not currently dfined as XBEE in control then call of to PS2 code
    if (g_bWhichControl != WC_XBEE)  {
        PS2ControlInput();
  
        if (g_fHexOn)
            g_bWhichControl = WC_PS2;
        else
           g_bWhichControl = WC_UNKNOWN;
    }
 
    // if not PS2 control, we will call XBEE function
    if (g_bWhichControl != WC_PS2) {
        XBeeControlInput();
    
        // If we made contact with XBee will use that...
        if (g_diystate.fTransReadyRecvd)
            g_bWhichControl = WC_XBEE;
        else
            g_bWhichControl = WC_UNKNOWN;
    }
}
#endif


//==============================================================================
// This is code the checks for and processes input from the DIY XBee receiver
//   work
//==============================================================================
#ifdef USEPS2
void XBeeControlInput(void)
#else
void ControlInput(void)
#endif
{
    byte iNumButton;
 
    // Then try to receive a packet of information from the XBEE.
    // It will return true if it has a valid packet
    if (ReceiveXBeePacket(&g_diyp)) {
//        toggle (P4);

        if (memcmp((void*)&g_diyp, (void*)&g_diypPrev, sizeof(g_diyp)) != 0) {
#ifdef XBEE_NEW_DATA_ONLY            
            if (g_diystate.fPacketForced)
                SendXbeeNewDataOnlyPacket(1);
#endif                
#ifdef DEBUG
            // setup to output back to our USB port
            if (g_fDebugOutput)  {
                DBGPrintf("%x - %d %d - %d %d - %d %d\n", g_diyp.wButtons, g_diyp.bRJoyLR, g_diyp.bRJoyUD,
                    g_diyp.bLJoyLR, g_diyp.bLJoyUD, g_diyp.bRSlider, g_diyp.bLSlider);
            }
#endif
        }

        // OK lets try "0" button for Start. 
        if ((g_diyp.wButtons & (1<<0)) && ((g_diypPrev.wButtons & (1<<0)) == 0)) { //Start Button (0 on keypad) test
            if(g_fHexOn)  {
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
            } else  {
                //Turn on
                g_fHexOn = 1;
            }
        } 
      
        if (g_fHexOn) {
            if ((g_diyp.wButtons & (1<<0xa)) && ((g_diypPrev.wButtons & (1<<0xa)) == 0)) { // A button test 
                MSound (SOUND_PIN, 1, 50, 2000);
                XBeePlaySounds(1, 50, 2000);
                bXBeeControlMode = WALKMODE;
                XBeeOutputString("Walking");
            }

            if ((g_diyp.wButtons & (1<<0xb)) && ((g_diypPrev.wButtons & (1<<0xb)) == 0)) { // B button test
                MSound (SOUND_PIN, 1, 50, 2000);
                XBeePlaySounds(1, 50, 2000);
                bXBeeControlMode = TRANSLATEMODE;
                XBeeOutputString("Body Translate");
            }

            if ((g_diyp.wButtons & (1<<0xc)) && ((g_diypPrev.wButtons & (1<<0xc)) == 0)) { // C button test
                MSound (SOUND_PIN, 1, 50, 2000);
                bXBeeControlMode = ROTATEMODE;
                XBeeOutputString("Body Rotate");
            }
         
            if ((g_diyp.wButtons & (1<<0xD)) && ((g_diypPrev.wButtons & (1<<0xd)) == 0)) { // D button test - Single Leg
                MSound (SOUND_PIN, 1, 50, 2000);
                if (SelectedLeg==255) // none
                    SelectedLeg=cRF;
                else if (bXBeeControlMode==SINGLELEGMODE) //Double press to turn all legs down
                    SelectedLeg=255;  //none
                bXBeeControlMode=SINGLELEGMODE;        
                XBeeOutputString ("Single Leg");
            }
            if ((g_diyp.wButtons & (1<<0xe)) && ((g_diypPrev.wButtons & (1<<0xe)) == 0)) { // E button test - Balance mode
                if (!BalanceMode) {
                    BalanceMode = 1;
                    MSound(SOUND_PIN, 2, 100, 2000, 50, 4000);
                    XBeePlaySounds(2, 100, 2000, 50, 4000);
                    XBeeOutputString("Balance On");
                } else {
                    BalanceMode = 0;
                    MSound(SOUND_PIN, 1, 250, 1500);
                    XBeePlaySounds(1, 50, 1500);
                    XBeeOutputString("Balance Off");
                }
            }

            if ((g_diyp.wButtons & (1<<0xf)) && ((g_diypPrev.wButtons & (1<<0xf)) == 0)) { // F button test - GP Player
                if (GPEnable) {   //F Button GP Player Mode Mode on/off -- SSC supports this mode
                    XBeeOutputString("Run Sequence");
                    MSound (SOUND_PIN, 1, 50, 2000);
                     
                    BodyPosX = 0;
                    BodyPosZ = 0;
                    BodyRotX1 = 0;
                    BodyRotY1 = 0;
                    BodyRotZ1 = 0;
                    TravelLengthX = 0;
                    TravelLengthZ = 0;
                    TravelRotationY = 0;
                     
                    SelectedLeg=255;  //none
                    fSLHold=0;
                   
                    bXBeeControlMode = GPPLAYERMODE;
                } else {
                    XBeeOutputString("Seq Disabled");
                    MSound (SOUND_PIN, 1, 50, 2000);
                }
            }
   
            //Hack there are several places that use the 1-N buttons to select a number as an index
            // so lets convert our bitmap of which key may be pressed to a number...
            // BUGBUG:: There is probably a cleaner way to convert... Will extract buttons 1-9
            iNumButton = 0;  // assume no button
            if ((g_diyp.wButtons & 0x3fe) && ((g_diypPrev.wButtons & 0x3fe) == 0)) { // buttons 1-9
                word w = g_diyp.wButtons & 0x3fe;
                
                while ((w & 0x1) == 0)     {
                    w >>= 1;
                    iNumButton++;
                } 
            }

            // BUGBUG:: we are using all keys now, may want to reserve some...  
            //Switch gait
            // We will do slightly different here than the RC version as we have a bit per button
            if ((bXBeeControlMode==WALKMODE) && iNumButton  && (iNumButton <= 8)) { //1-8 Button Gait select   
                if ( abs(TravelLengthX)<cTravelDeadZone &&  abs(TravelLengthZ)<cTravelDeadZone &&  
                        abs(TravelRotationY*2)<cTravelDeadZone)  {
                    //Switch Gait type
                    MSound(SOUND_PIN, 1, 50, 2000);   //Sound P9, [50\4000]
                    GaitType = iNumButton-1;
#ifdef DEBUG
                    DBGPrintf("New Gate: %d\n\r", GaitType);
#endif
                    GaitSelect();
#ifdef DEBUG
                    DBGPrintf("Output Gate Named\n\r");
#endif
                    XBeeOutputString((char*)s_asGateNames[GaitType]);
                }
            }

            //Switch single leg
            if (bXBeeControlMode==SINGLELEGMODE) {
                if (iNumButton>=1 && iNumButton<=6) {
                    MSound(SOUND_PIN, 1, 50, 2000);   //Sound P9, [50\4000]
                    SelectedLeg = iNumButton-1;
                    fSLHold=0;
                }
   
                if (iNumButton == 9) {  //Switch Directcontrol
                    MSound(SOUND_PIN, 1, 50, 2000);   //Sound P9, [50\4000]
                    fSLHold ^= 1;    //Toggle fSLHold
                }
 
            } else if (bXBeeControlMode==WALKMODE)  {
                SelectedLeg=255; // none
                fSLHold=0;
            }

            //Body Height
            BodyPosY = g_diyp.bLJoyUD / 2;
   
            //Leg lift height - Right slider has value 0-255 translate to 30-93
            LegLiftHeight = 30 + g_diyp.bRSlider/5;
   
            //Walk mode   
            if (bXBeeControlMode==WALKMODE) {
                TravelLengthX = -(g_diyp.bRJoyLR - 128);
                TravelLengthZ = -(g_diyp.bRJoyUD - 128) ;
                TravelRotationY = -(g_diyp.bLJoyLR - 128)/3;
                //BodyRotX1 = SmoothControl(((bPacket(PKT_RJOYUD)-128)*2), BodyRotX1, 2);
                //BodyRotZ1 = SmoothControl((-(bPacket(PKT_RPOT)-128)*2), BodyRotZ1, 2);
            }

            //Body move 
            if (bXBeeControlMode==TRANSLATEMODE)  {
                BodyPosX = (g_diyp.bRJoyLR-128)/2;
                BodyPosZ = (g_diyp.bRJoyUD-128)/2;
                BodyRotY1 = (g_diyp.bLJoyLR-128)*2; // need to integrate newer
            }  
   
            //Body rotate 
            if (bXBeeControlMode==ROTATEMODE) {
                BodyRotX1 = (g_diyp.bRJoyUD-128)*2;  // dito need to update
                BodyRotY1 = (g_diyp.bLJoyLR-128)*2;
                BodyRotZ1 = -(g_diyp.bRJoyLR-128)*2;
            }
   
            //Single Leg Mode
            if (bXBeeControlMode == SINGLELEGMODE)  {
                SLLegX = (g_diyp.bRJoyLR-128);
                SLLegZ = -(g_diyp.bRJoyUD-128);
                SLLegY = -(g_diyp.bLJoyLR-128)/5;
            }
   
            if (bXBeeControlMode == GPPLAYERMODE && iNumButton>=1 && iNumButton<=9) { //1-9 Button Play GP Seq
                word wGPSeqPtr;
                if (GPStart == 0)
                    GPSeq = iNumButton-1;
                // See if we can see if this sequence is defined
                SSCPrintf("EER -%d;2\n\r", GPSeq*2);
                if ((SSCRead((byte*)&wGPSeqPtr, sizeof(wGPSeqPtr), 
                        1000, 0xffff) == sizeof(wGPSeqPtr)) && (wGPSeqPtr != 0)  && (wGPSeqPtr != 0xffff)) {
                    // let user know that sequence was started
                    XBeeOutputString("Start Sequence");  // Tell user sequence started.
                    //     fGPSequenceRun = 1; // remember that ran one...
                    GPStart = 1;
                }  else
                    XBeeOutputString("Seq Not defined");  // that sequence was not defined...
            }
   
            //Calculate walking time delay
            InputTimeDelay = 128 -  max( max( abs(g_diyp.bRJoyLR-128),  abs(g_diyp.bRJoyUD-128)),  abs(g_diyp.bLJoyLR-128)) + (128 -(g_diyp.bLSlider)/2);
        }
        g_diypPrev = g_diyp; // remember the last packet
    }  else  {
        // Not a valid packet - we should go to a turned off state as to not walk into things!
        if (g_fHexOn && (g_diystate.fPacketForced ))  {
            // Turn off
            //   MSound (SOUND_PIN, 4, 100,2500, 80, 2250, 100, 2500, 60, 20000); // play it a little different...

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
        }
    }

#ifdef DEBUG_ENTERLEAVE
    DBGSerout ("Exit: Control Input\n\r");
#endif 
}

#endif //USEXBEE

