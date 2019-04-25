#include "SumoRobot.h"

SumoRobot robot;
void setup(){
    robot.init();

}

    
void loop(){
    robot.run();


}


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
