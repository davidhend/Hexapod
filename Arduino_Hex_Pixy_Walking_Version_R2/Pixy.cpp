#include <WProgram.h> 
#include "Hex_Cfg.h"
#include "Hex_Globals.h"

String PixySentance = "";
int pan_error = 0, tilt_error = 0;
int o_width, o_height = 0;

int distance_counter = 0;

void read_pixy_data()
{
  /* if data from the pixy is availiable */
  if(Pixy.available() > 0)
  {
    /* read pixy data */
    char incomingByte = Pixy.read();
    /* store pixy data to a string for later evaluation */
    PixySentance += incomingByte;
    /* if a newline is dected for the pixy parse the data for the individual values*/
    if (incomingByte == '\n')
    {
      /* find data seperation points */
      int point_0 = PixySentance.indexOf("b");
      int point_1 = PixySentance.indexOf(",", point_0 + 1);
      int point_2 = PixySentance.indexOf(",", point_1 + 1);
      int point_3 = PixySentance.indexOf(",", point_2 + 1);
      int point_4 = PixySentance.indexOf("!", point_3 + 1); 
      
       /* store pan & tilt error, width & height from Pixy as strings */
      String pan = PixySentance.substring(point_0 + 1, point_1);
      String tilt = PixySentance.substring(point_1 + 1, point_2);
      String obj_width = PixySentance.substring(point_2 + 1, point_3);
      String obj_height = PixySentance.substring(point_3 + 1, point_4);
      
      /* create char buffers for conversion*/
      char pixy_pan[5] = "";
      char pixy_tilt[5] = "";
      char pixy_width[5] = "";
      char pixy_height[5] = "";  
      
      /* calculate how many chars represent pan, tilt, width & height */
      int pan_length = pan.length();
      int tilt_length = tilt.length();
      int width_length = obj_width.length();
      int height_length = obj_height.length();
  
      /* convert pan string into char array */
      for(int i = 0; i <= pan_length; i++){
        pixy_pan[i] = pan[i]; 
      }
      /* convert tilt string into char array */
      for(int i = 0; i <= tilt_length; i++){
        pixy_tilt[i] = tilt[i]; 
      }
      /* convert tilt string into char array */
      for(int i = 0; i <= tilt_length; i++){
        pixy_width[i] = obj_width[i]; 
      }
      /* convert tilt string into char array */
      for(int i = 0; i <= tilt_length; i++){
        pixy_height[i] = obj_height[i]; 
      }      
      
      /* convert pan & tilt char arrays to integers*/
      pan_error = atoi(pixy_pan);
      tilt_error = atoi(pixy_tilt);      

      /* convert width & height char arrays to integers*/
      o_width = atoi(pixy_width);
      o_height = atoi(pixy_height);            
      
      /* display width & height of the object we are tracking to debug serial */
      DBGSerial.print(o_width);
      DBGSerial.print(" , ");
      DBGSerial.println(o_height);   
      
      /* if the object we are trackings width gets too far away */
      if(o_width == 10){ 
        /* increment width counter */
        distance_counter += 1;
        /* if the counter reaches 10 */
        if(distance_counter == 10){
          /* move the robot forward */
          TravelLengthZ = -((256)-128); //was 206 started at 166  
          /* zero out body rotations while walking */
          BodyRotX1 = 0;
          BodyRotY1 = 0;
          /* reset width counter */
          distance_counter = 0;
        }
      /* if the object we are tracking gets too close*/
      }else if(o_width > 45){
        /* move the robot backwards */
        TravelLengthZ = -((50)-128);  
        /* zero out body rotations while walking */
        BodyRotX1 = 0;
        BodyRotY1 = 0;
      /* if the object we are tracking isnt too far away or too close then rotate the robot body */
      }else{
        /* set the walking distance to zero */
        TravelLengthZ = 0;
        /* feed tilt error into pitch rotation*/
        BodyRotX1 = tilt_error;
        /* feed pan error into yaw rotation */
        BodyRotY1 = -pan_error; 
     
        /* if the pan error is above a certain threshold */
        if(pan_error >= 75){
          /* give the servos 150ms more to reach the destination */
          SSCTime += 150; 
        }else{
          /* otherwise use default ammount of time */
          SSCTime = 150; 
        }
      
        /* if the tilt error is above a certain threshold */
        if(tilt_error >= 75){
          /* give the servos 150ms more to reach the destination */
          SSCTime += 150; 
        }else{
          /* otherwise use default ammount of time */
          SSCTime = 150;
        }
        
      }
      
      /* if the EOL is found */
      if(point_4 > 0){
        /* reset the string holding the pixy data for next read */
        PixySentance = "";
      }
      
    }
    
  }  
  
}
