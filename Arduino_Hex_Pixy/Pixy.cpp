#include <WProgram.h> 
#include "Hex_Cfg.h"
#include "Hex_Globals.h"

String PixySentance = "";
int pan_error = 0;
int tilt_error = 0;

void read_pixy_data()
{
  if(Pixy.available() > 0)
  {
    char incomingByte = Pixy.read();
    PixySentance += incomingByte;
    
    if(incomingByte == '\n'){  
      
      /* find data seperation points */
      int firstListItem = PixySentance.indexOf("Err:");
      int firstComma = PixySentance.indexOf(",", firstListItem + 1);
      int secondListItem = PixySentance.indexOf("!", firstListItem + 1 );
      
      /* store pan & title error from Pixy as strings */
      String pixy_pan_err = PixySentance.substring(firstListItem+4, firstComma);
      String pixy_tilt_err = PixySentance.substring(firstComma+1, secondListItem);
      
      /* create pan & tilt char buffer for conversion*/
      char pixy_pan[5] = "";
      char pixy_tilt[5] = "";
      
      /* calculate how many chars represent pan & tilt */
      int pan_length = pixy_pan_err.length();
      int tilt_length = pixy_tilt_err.length();
      
      /* convert pan string into char array */
      for(int i = 0; i <= pan_length; i++){
        pixy_pan[i] = pixy_pan_err[i]; 
      }
      /* convert tilt string into char array */
      for(int i = 0; i <= tilt_length; i++){
        pixy_tilt[i] = pixy_tilt_err[i]; 
      }
      /* convert char array to integers*/
      pan_error = atoi(pixy_pan);
      tilt_error = atoi(pixy_tilt);
      
      /* display pan & tilt data to debug serial */
      /*
      DBGSerial.print("Pan Error: ");   
      DBGSerial.print(pan_error);
      DBGSerial.print(" , ");
      DBGSerial.print("Tilt Error: ");
      DBGSerial.print(tilt_error);
      DBGSerial.println();
      */
      /* move the robots body to keep the object in frame */
      
      /* pitch rotation*/
      BodyRotX1 = tilt_error;
      /* yaw rotation */
      BodyRotY1 = -pan_error; 
      
      /* if the pan error is greater than 50 give the servos 150ms more to reach the destination */
      if(pan_error >= 75){
        SSCTime += 150; 
      }else{
        SSCTime = 150; 
      }
      
      /* if the tilt error is greater than 50 give the servos 100ms more to reach the destination */
      if(tilt_error >= 75){
        SSCTime += 150; 
      }else{
        SSCTime = 150;
      }

      /* if the EOL is found reset for next read */
      if(secondListItem > 0){
        PixySentance = "";
      }
      
   }
  }  
  
}
