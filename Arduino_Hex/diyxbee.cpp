/****************************************************************************
 * - DIY remote control XBee support file
 *
 ****************************************************************************/
#define DEBUG
//#define DEBUG_OUT
//#define DEBUG_VERBOSE

#include "diyxbee.h"

#include "Hex_Globals.h"

DIYSTATE g_diystate;
boolean g_fDebugOutput = true;
#define XBEE_API_PH_SIZE    1            // Changed Packet Header to just 1 byte - Type - we don't use sequence number anyway...
//#define XBEE_NEW_DATA_ONLY 1
// Some forward definitions
#ifdef DEBUG
extern void DebugMemoryDump(const byte* , int, int);
static boolean s_fDisplayedTimeout = false;
#endif


//=============================================================================
// Xbee stuff
//=============================================================================
void InitXBee(void)
{
    XBeeSerial.begin(62500);    // BUGBUG??? need to see if this baud will work here.
    // Ok lets set the XBEE into API mode...

    delay(20);
    XBeePrintf("+++");
    WaitForXBeeTransmitComplete();
    delay(20);
    XBeePrintf("ATAP 1\rATCN\r");

    // for Xbee with no flow control, maybe nothing to do here yet...
    g_diystate.fTransReadyRecvd = false;
    g_diystate.fPacketValid = false; 
    g_diystate.fSendOnlyNewMode = false; // make sure it is init...
    g_diystate.bPacketNum = 0;
}


//=============================================================================
// byte ReadFromXBee - Read in a buffer of bytes.  We will pass in a timeout
//            value that if we dont receive a character in that amount of time 
//            something is wrong.  
//=============================================================================
// Quick and dirty helper function to read so many bytes in from the SSC with a timeout and an end of character marker...
byte ReadFromXBee(byte *pb, byte cb, ulong wTimeout, word wEOL)
{
    int ich;
    byte* pbIn = pb;
    unsigned long ulTimeLastChar = micros();

    while (cb) {
        while ((ich = XBeeSerial.read()) == -1) {
            // check for timeout
            if ((word)(micros()-ulTimeLastChar) > wTimeout)
                return (byte)(pb-pbIn);
        }
        *pb++ = (byte)ich;
        cb--;

        if ((word)ich == wEOL)
            break;    // we matched so get out of here.
        ulTimeLastChar = micros();    // update to say we received something
    }
    return (byte)(pb-pbIn);
}


//=============================================================================
// void WriteToXBee - output a buffer to the XBEE.  This is used to abstract
//            away the actual function needed to talk to the XBEE
//=============================================================================
inline void WriteToXBee(byte *pb, byte cb) __attribute__((always_inline));
void WriteToXBee(byte *pb, byte cb)
{
    XBeeSerial.write((byte*)pb, cb);
}


//=============================================================================
// void XBeePrintf - output a buffer to the XBEE.  This is used to abstract
//            away the actual function needed to talk to the XBEE
//=============================================================================
int XBeePrintf(const char *format, ...)
{
    char szTemp[80];
    int ich;
    va_list ap;
    va_start(ap, format);
    ich = vsprintf(szTemp, format, ap);
    XBeeSerial.write((byte*)szTemp, ich);
    va_end(ap);
}


//==============================================================================
// [SendXBeePacket] - Simple helper function to send the 4 byte packet header
//     plus the extra data if any
//      gosub SendXBeePacket[bPacketType, cbExtra, pExtra]
//==============================================================================
void SendXBeePacket(byte bPHType, byte cbExtra, byte *pbExtra)
{
    // Tell system to now output to the xbee
    byte abPH[9];
    byte *pbT;
    byte bChkSum;
    int i;

    // We need to setup the xbee Packet
    abPH[0]=0x7e;                        // Command prefix
    abPH[1]=0;                            // msb of size
    abPH[2]=cbExtra+XBEE_API_PH_SIZE + 5;    // size LSB
    abPH[3]=1;                             // Send to 16 bit address.

    g_diystate.bPacketNum = g_diystate.bPacketNum + 1;
    if (g_diystate.bPacketNum == 0)
        g_diystate.bPacketNum = 1;        // Don't pass 1 as this says no ack
    abPH[4]=g_diystate.bPacketNum;        // frame number
    abPH[5]=g_diystate.wAPIDL >> 8;        // Our current destination MSB/LSB
    abPH[6]=g_diystate.wAPIDL & 0xff;
    abPH[7]=0;                            // No Options

    abPH[8]=bPHType;

    // Now compute the initial part of the checksum
    bChkSum = 0;
    for (i=3;i <= 8; i++)
        bChkSum += abPH[i];

    // loop through the extra bytes in the exta to build the checksum;
    pbT = pbExtra;
    for (i=0; i < cbExtra; i++)
        bChkSum += *pbT++;                // add each byte to the checksum

    // Ok lets output the fixed part
    WriteToXBee(abPH,9);

    // Ok lets write the extra bytes if any to the xbee
    if (cbExtra)
        WriteToXBee(pbExtra, cbExtra);

    // Last write out the checksum
    bChkSum = 0xff - bChkSum;
    WriteToXBee(&bChkSum, 1);

#ifdef DEBUG_OUT
    // We moved dump before the serout as hserout will cause lots of interrupts which will screw up our serial output...
    // Moved after as we want the other side to get it as quick as possible...
    DBGPrintf("SDP: %2x %2x :", bPHType, cbExtra);

#ifdef DEBUG_VERBOSE        // Only ouput whole thing if verbose...
    if (abPH[3]) {
        byte i;
        for (i = 0; i < abPH[3]; i++) {
            DBGPrintf(" %2x", pbExtra++);
        }
    }
#endif    
    DBGPrintf("\r\n\r");

#endif

}

//==============================================================================
// [SendXbeeNewDataOnlyPacket] - Simple send packets to tell host if new only
// mode
//==============================================================================
void SendXbeeNewDataOnlyPacket(boolean fNewOnly)
{
    if (fNewOnly)
        SendXBeePacket(XBEE_RECV_REQ_NEW,  0, 0); 
    else
        SendXBeePacket(XBEE_RECV_REQ_NEW_OFF,  0, 0); 

    g_diystate.fSendOnlyNewMode = fNewOnly;
}


//==============================================================================
// [XBeeOutputVal] - Simple wrapper function to pass a word value back to
//            remote control to display
//==============================================================================
void XBeeOutputVal(word w)
{
    SendXBeePacket(XBEE_RECV_DISP_VAL, sizeof(w), (byte*)&w);
}

//==============================================================================
// [XBeePlaySounds] - Simple wrapper to take the inline notes and package them
//            up and semd them...
//            remote control to display
//==============================================================================
void XBeePlaySounds(byte cNotes, ...)
{
    va_list ap;
    byte abNotes[XBEE_MAX_NOTES*2];        // Should not hard code...
    byte cb = 0;

    va_start(ap, cNotes);

    if (cNotes > XBEE_MAX_NOTES)    // don't overrun our buffer...
        cNotes = XBEE_MAX_NOTES;

    while (cNotes > 0) {
        abNotes[cb++] = (byte)va_arg(ap, unsigned int);
        abNotes[cb++] = (byte)(va_arg(ap, unsigned int) / 25);
	cNotes--;
    }
    va_end(ap);
    SendXBeePacket(XBEE_PLAY_SOUND, cb, abNotes);
}


//////////////////////////////////////////////////////////////////////////////
//==============================================================================
// [APIRecvPacket - try to receive a packet from the XBee. 
//        - Will return TRUE if it receives something, else false
//        - Pass in buffer to receive packet.  Assumed it is big enough...
//        - pass in timeout if zero will return if no data...
//        
//==============================================================================
byte APIRecvPacket(ulong Timeout)
{
    byte cbRead;
    byte abT[3];
    byte bChksum;
    int i;

    short wPacketLen;
    //  First see if the user wants us to wait for input or not
    //    hserstat HSERSTAT_INPUT_EMPTY, _TP_Timeout            // if no input available quickly jump out.
    if (Timeout == 0) 
    {
        if (!XBeeSerial.available())
            return 0;        // nothing waiting for us...
        Timeout = 10000;    // .1 second?

    }

    // Now lets try to read in the data from the xbee...
    // first read in the delimter and packet length
    // We now do this in two steps.  The first to try to resync if the first character
    // is not the proper delimiter...

    do {    
        cbRead = ReadFromXBee(abT, 1, Timeout, 0xffff);
        if (cbRead == 0)
            return 0;
    } 
    while (abT[0] != 0x7e);

    cbRead = ReadFromXBee(abT, 2, Timeout, 0xffff);
    if (cbRead != 2)
        return 0;                // did not read in full header or the header was not correct.

    wPacketLen = (abT[0] << 8) + abT[1];

    // Now lets try to read in the packet
    cbRead = ReadFromXBee(g_diystate.bAPIPacket, wPacketLen+1, Timeout, 0xffff);


    // Now lets verify the checksum.
    bChksum = 0;
    for (i = 0; i < wPacketLen; i++)
        bChksum = bChksum + g_diystate.bAPIPacket[i];             // Add that byte to the buffer...


    if (g_diystate.bAPIPacket[wPacketLen] != (0xff - bChksum))
        return 0;                // checksum was off

    return wPacketLen;    // return the packet length as the caller may need to know this...
}



//==============================================================================
// [APISetXbeeHexVal] - Set one of the XBee Hex value registers.
//==============================================================================

void APISetXBeeHexVal(char c1, char c2, unsigned long _lval)
{
    byte abT[12];

    // Build a command buffer to output
    abT[0] = 0x7e;                    // command start
    abT[1] = 0;                        // Msb of packet size
    abT[2] = 8;                        // Packet size
    abT[3] = 8;                        // CMD=8 which is AT command

    g_diystate.bPacketNum = g_diystate.bPacketNum + 1;
    if (g_diystate.bPacketNum == 0)
        g_diystate.bPacketNum = 1;        // Don't pass 1 as this says no ack

    abT[4] = g_diystate.bPacketNum;    // Frame id
    abT[5] = c1;                    // Command name
    abT[6] = c2;

    abT[7] = _lval >> 24;            // Now output the 4 bytes for the new value
    abT[8] = (_lval >> 16) & 0xFF;
    abT[9] = (_lval >> 8) & 0xFF;
    abT[10] = _lval & 0xFF;

    // last but not least output the checksum
    abT[11] = 0xff - 
        ( ( 8+g_diystate.bPacketNum + c1 + c2 + (_lval >> 24) + ((_lval >> 16) & 0xFF) +
        ((_lval >> 8) & 0xFF) + (_lval & 0xFF) ) & 0xff);

    WriteToXBee(abT, sizeof(abT));

}


//==============================================================================
// [SetXbeeDL] - Set the XBee DL to the specified word that is passed
//         simple wrapper call to hex val
//==============================================================================
void SetXBeeDL (unsigned short wNewDL)
{
    APISetXBeeHexVal('D','L', wNewDL);
    g_diystate.wAPIDL = wNewDL;        // remember what DL we are talking to.
}


//==============================================================================
// [APISendXBeeGetCmd] - Output the command packet to retrieve a hex or string value
//==============================================================================

void APISendXBeeGetCmd(char c1, char c2)
{
    byte abT[8];

    // just output the bytes that we need...
    abT[0] = 0x7e;                    // command start
    abT[1] = 0;                        // Msb of packet size
    abT[2] = 4;                        // Packet size
    abT[3] = 8;                        // CMD=8 which is AT command

    g_diystate.bPacketNum = g_diystate.bPacketNum + 1;
    if (g_diystate.bPacketNum == 0)
        g_diystate.bPacketNum = 1;        // Don't pass 1 as this says no ack

    abT[4] = g_diystate.bPacketNum;    // Frame id
    abT[5] = c1;                    // Command name
    abT[6] = c2;

    // last but not least output the checksum
    abT[7] = 0xff - ((8 + g_diystate.bPacketNum + c1 + c2) & 0xff);
    WriteToXBee(abT, sizeof(abT));
}



//==============================================================================
// [GetXBeeHVal] - Set the XBee DL or MY or??? Simply pass the two characters
//             that were passed in to the XBEE
//==============================================================================
word GetXBeeHVal (char c1, char c2)
{

    // Output the request command
    APISendXBeeGetCmd(c1, c2);

    // Now lets loop reading responses 
    for (;;)
    {

        if (!APIRecvPacket(10000))
            break;

        // Only process the cmd return that is equal to our packet number we sent and has a valid return state
        if ((g_diystate.bAPIPacket[0] == 0x88) && (g_diystate.bAPIPacket[1] == g_diystate.bPacketNum) &&
            (g_diystate.bAPIPacket[4] == 0))
        {
            // BUGBUG: Why am I using the high 2 bytes if I am only processing words?
            return     (g_diystate.bAPIPacket[5] << 8) + g_diystate.bAPIPacket[6];
        }
    }
    return 0xffff;                // Did not receive the data properly.
}





/////////////////////////////////////////////////////////////////////////////


//==============================================================================
// [ClearXBeeInputBuffer] - This simple helper function will clear out the input
//                        buffer from the XBEE
//==============================================================================
void ClearXBeeInputBuffer(void)
{
    byte b[1];

#ifdef DEBUG
    boolean fBefore = g_fDebugOutput;
    g_fDebugOutput = false;
#endif    
    XBeeSerial.flush();    // clear out anything that was queued up...        
    while (ReadFromXBee(b, 1, 5000, 0xffff))
        ;    // just loop as long as we receive something...
#ifdef DEBUG
    g_fDebugOutput = fBefore;
#endif    
}



//==============================================================================
// [WaitForXBeeTransmitComplete] - This simple helper function will loop waiting
//                        for the uart to say it is done.
//==============================================================================
void WaitForXBeeTransmitComplete(void)
{
#if 0     // default Arduino implemention does not buffer output   
    // Only need to do something if we are a hardware serial port
    if (g_diystate.bpinIn == _S_HSERIAL_)
        while(!hserstat(HSERSTAT_OUTPUT_EMPTY))
            ;
#endif        
}

//==============================================================================
// [DebugMemoryDump] - striped down version of rprintfMemoryDump
//==============================================================================
#ifdef DEBUG
void DebugMemoryDump(const byte* data, int off, int len)
{
    int x;
    int c;
    int line;
    const byte * b = data;

    for(line = 0; line < ((len % 16 != 0) ? (len / 16) + 1 : (len / 16)); line++)  {
        int line16 = line * 16;
        DBGPrintf("%4x| ", line16);

        // put hex values
        for(x = 0; x < 16; x++) {
            if(x + line16 < len) {
                c = b[off + x + line16];
                DBGPrintf("%2x ", c);
            }
            else
                DBGPrintf("   ");
        }
        DBGPrintf("| ");

        // put ascii values
        for(x = 0; x < 16; x++) {
            if(x + line16 < len) {
                c = b[off + x + line16];
                DBGPrintf("%c",((c > 0x1f) && (c < 0x7f))? c : '.');
            }
            else
                DBGPrintf(" ");
        }
        DBGPrintf("\n\r\r");
    }
}

#endif


//==============================================================================
// [ReceiveXBeePacket] - This function will try to receive a packet of information
//         from the remote control over XBee.
//
// the data in a standard packet is arranged in the following byte order:
//    0 - Buttons High
//    1 - Buttons Low
//     2 - Right Joystick L/R
//    3 - Right Joystick U/D
//    4 - Left Joystick L/R
//    5 - Left Joystick U/D
//     6 - Right Slider
//    7 - Left Slider
//==============================================================================
boolean ReceiveXBeePacket(PDIYPACKET pdiyp)
{
    byte cbRead;
    byte bDataOffset;
    word wNewDL;
    ulong ulCurrentTime;
    ulong ulTimeDiffMS;
    boolean _fPacketValidPrev = g_diystate.fPacketValid;        // Save away the previous state as this is the state if no new data...
    boolean _fNewPacketAvail = false;

    g_diystate.fPacketValid = false;
    g_diystate.fPacketTimeOut = false;
    g_diystate.fPacketForced = false;

    //    We will first see if we have a packet header waiting for us.
    //  BUGBUG:: Question should I loop after I process a package and only get out of the
    //             loop when I have no more data, or only process one possible message?
    //            Maybe depends on message?
    for (;;)
    {
        if (!XBeeSerial.available())
            break;        // no input available, break from this loop

        // The XBEE has sent us some data so try to get a packet header
        cbRead = APIRecvPacket(10000);        // Lets read in a complete packet.
        if (!cbRead)
            break;                            // Again nothing read?

#ifdef DEBUG
        s_fDisplayedTimeout = false;        // say that we got something so we can display empty again...
#endif
        if (g_diystate.bAPIPacket[0] == 0x81)
            bDataOffset = 5;                // Received packet with 16 bit addressing
        else if (g_diystate.bAPIPacket[0] == 0x80)
            bDataOffset = 11;                // Received packet with 64 bit addressing
        else if (g_diystate.bAPIPacket[0] == 0x89)
            continue;                        // API set return value, ignore and try again
        else
            break;                            // Invalid packet lets bail from this loop.

        // Change CB into the number of extra bytes...
        cbRead -= (bDataOffset + 1);        // Ph is only 1 byte long now... 

        //-----------------------------------------------------------------------------
        // [XBEE_TRANS_DATA]
        //-----------------------------------------------------------------------------
        if ((g_diystate.bAPIPacket[bDataOffset + 0] == XBEE_TRANS_DATA) && (cbRead == sizeof(DIYPACKET))) {
            pdiyp->wButtons = g_diystate.bAPIPacket[bDataOffset + 1] + (g_diystate.bAPIPacket[bDataOffset + 2] << 8);
            pdiyp->bRJoyLR = g_diystate.bAPIPacket[bDataOffset + 3];
            pdiyp->bRJoyUD = g_diystate.bAPIPacket[bDataOffset + 4];
            pdiyp->bLJoyLR = g_diystate.bAPIPacket[bDataOffset + 5];
            pdiyp->bLJoyUD = g_diystate.bAPIPacket[bDataOffset + 6];
            pdiyp->bRSlider = g_diystate.bAPIPacket[bDataOffset + 7];
            pdiyp->bLSlider = g_diystate.bAPIPacket[bDataOffset + 8];

            // process first as higher number of these come in...
            //memcpy((void*)pdiyp, (void*)&g_diystate.bAPIPacket[bDataOffset + 1], sizeof(DIYPACKET));
            g_diystate.fPacketValid = true;    // data is valid
            g_diystate.fPacketForced = g_diystate.fReqDataForced;    // Was the last request forced???
            g_diystate.fReqDataForced = 0;                // clear that state now
            g_diystate.ulLastPacket = millis();
            return true;    //          // get out quick!

            //-----------------------------------------------------------------------------
            // [XBEE_TRANS_READY]
            //-----------------------------------------------------------------------------
        } 
        else if ((g_diystate.bAPIPacket[bDataOffset + 0] == XBEE_TRANS_READY) && (cbRead == 2)) {
            wNewDL = (g_diystate.bAPIPacket[bDataOffset + 1] << 8) + g_diystate.bAPIPacket[bDataOffset + 2];    // take care of warning, probably not needed
            g_diystate.fTransReadyRecvd = true;        // OK we have received a packet saying transmitter is ready.    
            SetXBeeDL(wNewDL);


            // And tell the remote to go into New data only mode.
#ifdef XBEE_NEW_DATA_ONLY            
            SendXbeeNewDataOnlyPacket(1);
#endif            
            g_diystate.ulLastPacket = millis();
            g_diystate.fReqDataPacketSent = 0;                            // make sure we don't think we have any outstanding requests
            _fNewPacketAvail = true;                                    // and try to get the first packet of data
#ifdef DEBUG
            if (g_fDebugOutput) {
                DBGPrintf("XBee_Trans_READY: %x\n\r", wNewDL);
            }
#endif //DEBUG
            //-----------------------------------------------------------------------------
            // [XBEE_TRANS_NOTREADY]
            //-----------------------------------------------------------------------------
        } 
        else if (g_diystate.bAPIPacket[bDataOffset + 0] == XBEE_TRANS_NOTREADY) {
            g_diystate.fTransReadyRecvd = 0;            // Ok not valid anymore...
#ifdef DEBUG
            if (g_fDebugOutput) {
                DBGPrintf("XBee_Trans_NOTREADY\r\n");
            }
#endif //DEBUG

            //-----------------------------------------------------------------------------
            // [XBEE_TRANS_NEW]
            //-----------------------------------------------------------------------------
        } 
        else if (g_diystate.bAPIPacket[bDataOffset + 0] == XBEE_TRANS_NEW) {
            _fNewPacketAvail = true;
            //-----------------------------------------------------------------------------
            // [UNKNOWN PACKET]
            //-----------------------------------------------------------------------------
        } 
        else {
#ifdef DEBUG
            if (g_fDebugOutput) {
                DBGPrintf("Unknown Packet: %x %d\n\r", g_diystate.bAPIPacket[bDataOffset + 0], cbRead);
            }
#endif //DEBUG
        }
    }

    //-----------------------------------------------------------------------------
    // Exited above loop now See if we need to request data from the other side
    //-----------------------------------------------------------------------------
    // Only send when we know the transmitter is ready.  Also if we are in the New data only mode don't ask for data unless we have been 
    //    old there
    // is new data. We relax this a little and be sure to ask for data every so often as to make sure the remote is still working...
    // 
    if (g_diystate.fTransReadyRecvd) {
        ulCurrentTime = millis();

        // Time in MS since last packet
        ulTimeDiffMS = ulCurrentTime - g_diystate.ulLastPacket;

        // See if we exceeded a global timeout.  If so let caller know so they can stop themself if necessary...
        if (ulTimeDiffMS > CXBEETIMEOUTRECVMS) {
#ifdef DEBUG
            if (g_fDebugOutput) {
                if (!s_fDisplayedTimeout) {
                    DBGPrintf("XBEE Timeout\n\r"); 
                    s_fDisplayedTimeout = true;
                }
            }                
#endif //DEBUG
            g_diystate.fPacketValid = 0;
            g_diystate.fPacketForced = true;
            return false;
        }

        // see if we have an outstanding request out and if it timed out...
        if (g_diystate.fReqDataPacketSent) {
            if ((ulCurrentTime-g_diystate.ulLastRequest) > CXBEEPACKETTIMEOUTMS) {
                // packet request timed out, force a new attempt.
                _fNewPacketAvail = true;        // make sure it requests a new one    
                g_diystate.fReqDataPacketSent = false;     // make sure we send a new one...
            }
        }

        // Next see if it has been too long since we received a packet.  Ask to make sure they are there...
        if (!_fNewPacketAvail && (ulTimeDiffMS > CXBEEFORCEREQMS)) {
            _fNewPacketAvail = true;
            g_diystate.fReqDataForced = true;        // remember that this request was forced!
        }

        if (!g_diystate.fSendOnlyNewMode || (g_diystate.fSendOnlyNewMode && _fNewPacketAvail)) {
            // Now send out a prompt request to the transmitter:

            if (!g_diystate.fReqDataPacketSent) {
                SendXBeePacket (XBEE_RECV_REQ_DATA, 0, 0);        // Request data Prompt (CmdType, ChkSum, Packet Number, CB extra data)
                g_diystate.fReqDataPacketSent = true;             // yes we have already sent one.
                g_diystate.ulLastRequest = ulCurrentTime;         // remember when we sent this...
            }
        }
        g_diystate.fPacketValid = _fPacketValidPrev;    // Say the data is in the same state as the previous call...
    }

    return g_diystate.fPacketValid;
}



