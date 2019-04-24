#include "SumoRobot.h"

SumoRobot robot;
void setup(){
    robot.init();

}

    
void loop(){
    robot.run();


}





/*
void serialEvent() {
  char inChar;
  while (Serial.available()) {
    // get the new byte:
    inChar = (char)Serial.read();
    // add it to the inputString:
    dataFromMaster += inChar;
    }
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if(inChar == '\n')
    {
        //setCommand(dataFromMaster);

        if (dataFromMaster[0] == 1)
        {   
            pwmLeftWheel = dataFromMaster[2];
            pwmRightWheel = dataFromMaster[3];
            direction = (dataFromMaster[4] == 1); // forward o reverse
            goToPwm = true;
            timeArriveCommand = millis();
        }
        dataFromMaster ="";
    }
    
    
  
}


*/


/*
void setCommand(String dataFromMaster )
{
    if(int(dataFromMaster[0]) == 1) // Address
    {
        switch(int(dataFromMaster[1]) ) //command
        {
            case 1 :
                ledColor(dataFromMaster[2]); // set color
                break;
            case 2:
                int bearx = dataFromMaster[2];
                int beary = dataFromMaster[3];
                int speed = dataFromMaster[4];
                
                break;

        }
    }
 
  
}
*/

//sudo ./arduino-cli-0.3.6-alpha.preview-linuxarm compile --fqbn arduino:avr:nano SumoPontifice-Firmware/Encoder
