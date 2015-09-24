//=============================================================================
// DIYXBee.h XBee Support for the DIY Remote control
// [Packets sent from Remote to Robot]
//=============================================================================
#ifndef _DIYXBEE_H_
#define _DIYXBEE_H_
#include <WProgram.h> 

#ifndef CXBEEPACKETTIMEOUTMS
#define CXBEEPACKETTIMEOUTMS 250					// how long to wait for packet after we send request
#endif

#ifndef CXBEEFORCEREQMS	
#define CXBEEFORCEREQMS		1000					// if nothing in 1 second force a request...
#endif

#ifndef CXBEETIMEOUTRECVMS
#define CXBEETIMEOUTRECVMS	2000					// 2 seconds if we receive nothing
#endif


#define XBEE_TRANS_READY			0x01	// Transmitter is ready for requests.
	// Optional Word to use in ATDL command
#define XBEE_TRANS_NOTREADY		 0x02	// Transmitter is exiting transmitting on the sent DL
	// No Extra bytes.
#define XBEE_TRANS_DATA				0x03	// Data Packet from Transmitter to Robot*
	// Packet data described below.  Will only be sent when the robot sends
	// the remote a XBEE_RECV_REQ_DATA packet and we must return sequence number
#define XBEE_TRANS_NEW				0x04	// New Data Available
	// No extra data.  Will only be sent when we are in NEW only mode
#define XBEE_ENTER_SSC_MODE			0x05	// The controller is letting robot know to enter SSC echo mode
	// while in this mode the robot will try to be a pass through to the robot. This code assumes
	// cSSC_IN/cSSC_OUT.  The robot should probalby send a XBEE_SSC_MODE_EXITED when it decides
	// to leave this mode...	
	// When packet is received, fPacketEnterSSCMode will be set to TRUE.  Handlers should probalby
	// get robot to a good state and then call XBeeHandleSSCMode, which will return when some exit
	// condition is reached.  Start of with $$<CR> command as to signal exit
#define XBEE_REQ_SN_NI				0x06	// Request the serial number and NI string

//[Packets sent from Robot to remote]
#define XBEE_RECV_REQ_DATA			0x80	// Request Data Packet*
	// No extra bytes: expect to receive XBEE_TRANS_DATA_PACKET
#define XBEE_RECV_REQ_NEW			0x81	// Request Only New data
	// No Extra bytes goes into New only mode and we will typically 
	// wait until Remote says it has new data before asking for data.
	// In new mode, the remote may choose to choose a threshold of how big a change
	// needs to be before sending the XBEE_TRANS_NEW value.
#define XBEE_RECV_REQ_NEW_OFF		0x82	// We will request data when we want it
#define XBEE_RECV_NEW_THRESH	 	0x83	// Set new Data thresholds
	// currently not implemented
#define XBEE_RECV_DISP_VAL			0x84	// Display a value on line 2
	// If <cbExtra> is  0 then we will display the number contained in <SerialNumber> 
	// If not zero, then it is a count of bytes in a string to display.
#define XBEE_RECV_DISP_STR			0x85	// Display a string value on line 2
#define XBEE_PLAY_SOUND				0x86	// Will make sounds on the remote...
	//	<cbExtra> - 2 bytes per sound: Duration <0-255>, Sound: <Freq/25> to make fit in byte...
#define XBEE_SSC_MODE_EXITED		0x87	// a message sent back to the controller when
	// it has left SSC-mode.
#define XBEE_SEND_SN_NI_DATA		0x88	// Response for REQ_SN_NI - will return
	// 4 bytes - SNH
	// 4 bytes - SNL
	// up to 20 bytes(probably 14) for NI

//[XBEE_TRANS_DATA] - has 8 extra bytes
//	0 - Buttons High
//	1 - Buttons Low
// 	2 - Right Joystick L/R
//	3 - Right Joystick U/D
//	4 - Left Joystick L/R
//	5 - Left Joystick U/D
// 	6 - Right Slider
//	7 - Left Slider

// OK Lets define some structures...

typedef struct _xpacketheader {
	byte	bPacketType;				// Packet Type
	byte	bChksum;					// Check sum
	byte	bSeqNum;					// Sequence Number or ...
	byte	cbExtra;					// how many bytes extra
} XPACKETHEADER;
typedef XPACKETHEADER *PXPACKETHEADER;	// also define pointer to it

					
// Main data packet						
typedef struct _diypacket {
	word	wButtons;					// the 16 buttons
	byte	bRJoyLR;					// right joystick X (LR)
	byte	bRJoyUD;					//				..Y (UD)
	byte	bLJoyLR;					// Left Joystick  X (LR)
	byte	bLJoyUD;					// 				  Y (UD)
	byte	bRSlider;					// Right Slider
	byte	bLSlider;					// Left Slider
} DIYPACKET;

typedef DIYPACKET *PDIYPACKET;
	

#define PKT_BTNLOW		0				// Low Buttons 0-7
#define PKT_BTNHI		1				// High buttons 8-F
#define PKT_RJOYLR		2				// Right Joystick Up/Down
#define PKT_RJOYUD		3				// Right joystick left/Right
#define PKT_LJOYLR		4				// Left joystick Left/Right
#define PKT_LJOYUD		5				// Left joystick Up/Down
#define PKT_RSLIDER		6				// right slider
#define PKT_LSLIDER		7				// Left slider

typedef unsigned long ulong;

// Now define some static state stuff
typedef struct _diystate
{
	// state information
	boolean 	fTransReadyRecvd;
	boolean 	fPacketValid; 
	boolean 	fPacketTimeOut;
	boolean 	fPacketForced;

	// More internal to function...
	boolean 	fReqDataPacketSent;
	boolean 	fReqDataForced;
	boolean 	fSendOnlyNewMode;
	byte	bPacketNum;
	byte	bAPIPacket[33];				// Api packet
	word		wAPIDL;						// current destination.

	// Other information, could make static to file...
	ulong	ulLastPacket;
	ulong 	ulLastRequest;
	
} DIYSTATE;

extern DIYSTATE g_diystate;

// Forward references some may be moved to external files later
extern void InitXBee();    // assume hard coded for now to UART2... 
extern void SendXBeePacket(byte bPHType, byte cbExtra, byte *pbExtra);
extern void SendXbeeNewDataOnlyPacket(boolean fNewOnly);
extern void XBeeOutputVal(word w);
extern boolean ReceiveXBeePacket(PDIYPACKET pdiyp);

#define XBeeOutputString(pString) 	{SendXBeePacket(XBEE_RECV_DISP_STR, strlen(pString), (byte*)pString);}
#define XBEE_MAX_NOTES     5
extern void XBeePlaySounds(byte cNotes, ...);

//extern void XBeeOutputVal(byte bXbeeVal);
//extern void XBeeOutputString(char *pString);
//extern void XBeePlaySounds(byte *pb, byte cb);
extern word GetXBeeHVal (char c1, char c2);
//extern word GetXBeeMY();
//extern word GetXBeeDL();
#define GetXBeeMY()			GetXBeeHVal ('M', 'Y')
#define GetXBeeDL()			GetXBeeHVal ('D', 'L')

// These functions are the ones to actually talk to the hardware.  Should try to make sure
// all of the other functions talk through these...
extern byte ReadFromXBee(byte *pb, byte cb, ulong Timeout, word wEOL);
extern void ClearXBeeInputBuffer(void);
extern boolean XBeeCheckForQueuedBytes(void);
extern void WaitForXBeeTransmitComplete(void);
extern void WriteToXBee(byte *pb, byte cb);
extern int XBeePrintf(const char *format, ...);


extern void ClearInputBuffer(void);
#endif // _DIYXBEE_H_

