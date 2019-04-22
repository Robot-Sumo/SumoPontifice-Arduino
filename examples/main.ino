#include "SumoRobot.h"

    
void setup(){/*nothing to setup*/}
    
void loop(){

}



/*
ISR(TIMER1_COMPA_vect){//timer1 interrupt 21Hz toggles pin 13 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
Serial.println(micros()-timeLast);
 timeLast = micros();
  if (toggle1){
    digitalWrite(13,HIGH);
    toggle1 = 0;
  }
  else{
    digitalWrite(13,LOW);
    toggle1 = 1;
  }
 
}









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
