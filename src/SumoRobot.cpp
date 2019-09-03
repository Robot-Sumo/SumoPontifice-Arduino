#include "SumoRobot.h"


long SumoRobot::encoderRightCounter;
long SumoRobot::encoderLeftCounter;
unsigned long SumoRobot::encoderRightTime;
unsigned long SumoRobot::encoderLeftTime;

bool SumoRobot::encoderRightFlag;
bool SumoRobot::encoderLeftFlag;
int SumoRobot::direction;
bool SumoRobot::samplingEna;
bool SumoRobot::goToController;




// Timer2Isr        
int SumoRobot::encoderRightMeasure;
int SumoRobot::encoderLeftMeasure;
long SumoRobot::encoderRightSample;
long SumoRobot::encoderLeftSample;


        // buffer de salida
int SumoRobot::encoderRightBuffer[100];
int SumoRobot::encoderLeftBuffer[100];
int SumoRobot::encoderRightBufferIndex;
int SumoRobot::encoderLeftBufferIndex;
int SumoRobot::counterTimer2;



boolean SumoRobot::toggle1 ;
unsigned long SumoRobot::timeLast ;


//t's just declaration inside of class, and you have to make a space for this variable too (by adding definition).

SumoRobot::SumoRobot()
{

    
}



int SumoRobot::run()
{
    StateMachine();
}


void SumoRobot::StateMachine()
{
    
    switch (currentState){
        case states::idle:
            if( (millis()-timeArriveCommand) > 1500)
            {
                goToStop = true;
                timeArriveCommand = millis();

            }
            if(goToStop) 
            {
                goToStop = false;
                currentState = states::stop;
            }
            if(goToController) 
            {
                goToController = false;
                currentState = states::controller;
            }

            if(Serial.available())
            {
                decodeSerial();
            }
              
            break;

        case states::stop:
            setPwm(0, 0, 1);
            ledColor(rojo);
            currentState = idle;
            break;
        case states::controller:

            // Calcular set point
            Joystick2Velocity();

            // Calcular velocidad actual (rad/s)
            velocityLeftWheelMeasure = encoderLeftMeasure*constConversion/samplingPeriod; 
            velocityRightWheelMeasure = encoderRightMeasure*constConversion/samplingPeriod; 

            // Calcular error 
            errorLeftWheel =  velocityLeftWheelSetPoint-velocityLeftWheelMeasure;
            errorRightWheel =  velocityRightWheelSetPoint-velocityRightWheelMeasure;
            
            velocityLeftWheelInputController = velocityLeftWheelSetPoint+KLeftWheel*errorLeftWheel;
            velocityRightWheelInputController = velocityRightWheelSetPoint+KRightWheel*errorRightWheel;

            pwmLeftWheel = Velocity2PWMLeftWheel(velocityLeftWheelInputController);
            pwmRightWheel = Velocity2PWMRightWheel(velocityRightWheelInputController);

            setPwm(pwmLeftWheel, pwmRightWheel, direction);
            currentState = idle;
            break;
        
            

        

    }
}


void SumoRobot::init()
{
    // Setting pin modes
    // Inputs
    pinMode(diag, INPUT);
    pinMode(diagb, INPUT);
    pinMode(in1bAux, INPUT); // Debe declarase siempre como entrada
    pinMode(in2bAux, INPUT); // Debe declarase siempre como entrada
    pinMode(encRightWheel, INPUT_PULLUP); // Debe declarase siempre con INPUT_PULLUP
    pinMode(encLeftWheel, INPUT_PULLUP); // Debe declarase siempre con INPUT_PULLUP
    

    attachInterrupt(digitalPinToInterrupt(encRightWheel), EncoderRightWheel, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encLeftWheel), EncoderLeftWheel, CHANGE);

    // Outputs
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in1b, OUTPUT);
    pinMode(in2b, OUTPUT);

    pinMode(led_Rpin, OUTPUT); 
    pinMode(led_Gpin, OUTPUT);
    pinMode(led_Bpin, OUTPUT);
    pinMode(led1, OUTPUT);

    Serial.begin(57600);

    resetThisDevice();
       
    counterTimer2 = 0;
    Timer2.initialize();                  // Initialise timer 2 500 Hz m
    //Timer2.attachInterrupt( Timer2Isr );
    
    //Timer2.start();

   




    // Configura timer 1 a 25 HZ
    /*

    */
    

}

void SumoRobot::resetThisDevice()
{
    ledColor(off);
    setPwm(0, 0, 0);

    delay(1000);
        
    goToController = false;
    goToStop = true;
    currentState = idle;
    timeArriveCommand = millis();
    driverBearing = 0;
    driverVel = 0;
    direction = 0;
    samplingEna = false;

    toggle1 = 0;
    timeLast = 0;

    //Timer2.stop();

    dataFromMaster = "";
    ledColor(morado);
    delay(1000);
    ledColor(rojo);
    digitalWrite(13,LOW);
   
    


}

void SumoRobot::resetEncoder()
{
    encoderLeftCounter= 0;
    encoderLeftTime = micros();
    encoderRightCounter= 0;
    encoderRightTime = micros();
    encoderLeftFlag = false;
    encoderRightFlag = false;
    encoderRightMeasure = 0;
    encoderLeftMeasure = 0;
    encoderRightSample = 0;
    encoderLeftSample = 0;
    encoderRightBufferIndex = 0;
    encoderLeftBufferIndex = 0;
}
void SumoRobot::setPwm(uint8_t pwmLeftWheel, uint8_t pwmRightWheel, bool direction)
{

    if(direction == 0) // forward
    {
        analogWrite(in1, pwmRightWheel);
        digitalWrite(in2, LOW);
        analogWrite(in1b, pwmLeftWheel);
        digitalWrite(in2b, LOW);
        ledColor(verde);
    }
    else if (direction == 1) // reverse
    {
        analogWrite(in2, pwmRightWheel);
        digitalWrite(in1, LOW);
        analogWrite(in2b, pwmLeftWheel);
        analogWrite(led1, pwmLeftWheel);
        digitalWrite(in1b, LOW);
        ledColor(azul);
    }
    else // stop
    {
        pwmRightWheel = 0;
        pwmLeftWheel = 0;
    }
    if(pwmRightWheel == 0 && pwmLeftWheel == 0) ledColor(rojo);
    
}

void SumoRobot::ledColor(int color)
{
    switch(color)
    {
        case azul :
            digitalWrite(led_Rpin, LOW);
            digitalWrite(led_Gpin, LOW);
            digitalWrite(led_Bpin, HIGH);
            break;
        case rojo:
            digitalWrite(led_Rpin, HIGH);
            digitalWrite(led_Gpin, LOW);
            digitalWrite(led_Bpin, LOW); 
            break;
        case morado:
            digitalWrite(led_Rpin, HIGH);
            digitalWrite(led_Gpin, LOW);
            digitalWrite(led_Bpin, HIGH);
            break;
        case turquesa:
            digitalWrite(led_Rpin, LOW);
            digitalWrite(led_Gpin, HIGH);
            digitalWrite(led_Bpin, HIGH);
            break;
        case blanco:
            digitalWrite(led_Rpin, HIGH);
            digitalWrite(led_Gpin, HIGH);
            digitalWrite(led_Bpin, HIGH);
            break;
        case amarillo:
            digitalWrite(led_Rpin, HIGH);
            digitalWrite(led_Gpin, HIGH);
            digitalWrite(led_Bpin, LOW);
            break;
        case verde:
            digitalWrite(led_Rpin, LOW);
            digitalWrite(led_Gpin, HIGH);
            digitalWrite(led_Bpin, LOW);
            break;
        case off:
            digitalWrite(led_Rpin, LOW);
            digitalWrite(led_Gpin, LOW);
            digitalWrite(led_Bpin, LOW);
            break;


    }
    
}



void SumoRobot::EncoderRightWheel(){
  
 
  
  if ( (micros()-encoderRightTime) > 3000) // Si ha transcurrido mas de 1000 us desde la ultima interrupcion
  {
    

    if(direction)
        encoderRightCounter++;
    else
        encoderRightCounter--;

    encoderRightFlag = true;
    encoderRightTime = micros(); // a relacion de 15 de reduccion, maximo 200 rpm, 7.5 ms


  }
  

  
  
}


void SumoRobot::EncoderLeftWheel(){
    
   
  if ( (micros()-encoderLeftTime) > 3000) // Si ha transcurrido mas de 1000 us desde la ultima interrupcion
  {
    if(direction)
        encoderLeftCounter++;
    else
        encoderLeftCounter--;

    encoderLeftTime = micros(); // a relacion de 15 de reduccion, maximo 200 rpm, 7.5 ms
    encoderLeftFlag = true;




    
  }

  
  

}


void SumoRobot::timer2Isr()
{
       //Timer2 interrupt 21Hz toggles pin 13 (LED)
    //generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
   // Serial.println(micros()-timeLast);
    //timeLast = micros();

   
    counterTimer2++;

    if (counterTimer2 == 25) // 500 hz / 25 = 20 Hz
    {
        counterTimer2 = 0;

   

        // tomar medidas
        encoderLeftMeasure = int(encoderLeftCounter- encoderLeftSample);
        encoderRightMeasure = int(encoderRightCounter- encoderRightSample);    
        
        //
        encoderLeftSample = encoderLeftCounter;
        encoderRightSample = encoderRightCounter;
        goToController = true;
        
        

        if (samplingEna)
        {


            if (toggle1){
                    digitalWrite(13,HIGH);
                    toggle1 = 0;
                }
            else{
                    digitalWrite(13,LOW);
                    toggle1 = 1;
            }
            
            
            encoderLeftBuffer[encoderLeftBufferIndex] = encoderLeftMeasure;
            encoderRightBuffer[encoderRightBufferIndex] = encoderRightMeasure;
            encoderRightBufferIndex++;
            encoderLeftBufferIndex++;

            if(encoderLeftBufferIndex>100) // Reset buffer
            {
                encoderLeftBufferIndex = 0;
                encoderRightBufferIndex = 0;
            }

        }

    }

    

}

void SumoRobot::decodeSerial()
{
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

        if(int(dataFromMaster[0]) == 1) // Address match
        {
            setCommand();
            timeArriveCommand = millis();
        }

        /*
        if (dataFromMaster[0] == 1)
        {   
            pwmLeftWheel = dataFromMaster[2];
            pwmRightWheel = dataFromMaster[3];
            direction = (dataFromMaster[4] == 1); // forward o reverse
            goToController = true;
            
        }
        */
        dataFromMaster ="";
    }
    else
    {
            if(int(dataFromMaster[0]) != 1) // Address dont match
            {
                delay(100);
                Serial.flush();
                dataFromMaster ="";
            }
          

    }
    
}


void SumoRobot::setCommand()
{
  
    switch(int(dataFromMaster[1]) ) //command
    {
        case setLED_RGB :
            ledColor(dataFromMaster[2]); // set color
            break;
        case setBearingVector:
            driverBearing = dataFromMaster[2]-127; // to map between -127 and 127;
            driverVel = dataFromMaster[3];
            direction = dataFromMaster[4]; // forward o reverse 
            break;

        case getBatteryState:
            break;
        
        case startEncoderSampling:
            Timer2.attachInterrupt( timer2Isr );           // attach the ISR routine here
            resetEncoder();
            samplingEna = true;
            ledColor(verde);
            break;

        case setSampleFrecuency:
            
            break;

        case getBufferData:

            sendBufferData();

            break;
        case resetRobot:
            resetThisDevice();
            break;


    }
}


void SumoRobot::sendBufferData()
{
    uint8_t sizeData ;
    uint8_t indexBuffer;


    sizeData = 4*encoderLeftBufferIndex; // tamaño total en bytes
    indexBuffer = 0;


    for (int i = 0; i< sizeData; i = i+4)
    {
        Trama[i] =  ( (encoderLeftBuffer[indexBuffer]>>8) & 0xff);      // upper byte
        Trama[i+1] = (encoderLeftBuffer[indexBuffer]& 0xff);       // lower byte
        Trama[i+2] = ((encoderRightBuffer[indexBuffer]>>8) & 0xff); // upper byte
        Trama[i+3] = (encoderRightBuffer[indexBuffer] & 0xff);   // lower byte
        indexBuffer++;

    }

    // Reiniciar buffer
    encoderRightBufferIndex = 0; 
    encoderLeftBufferIndex = 0;

    
    Serial.write(Trama, sizeData);


}


void SumoRobot::Joystick2Velocity()
{


    if (driverBearing > 0) // Turn Right
    {
        velocityLeftWheelSetPoint = Kv*(-2*driverBearing+ driverVel);
        velocityRightWheelSetPoint =Kv*(driverVel); 
    }
    else // Turn Left
    {
        velocityLeftWheelSetPoint = Kv*(driverVel);
        velocityRightWheelSetPoint =Kv*(2*driverBearing+ driverVel);
    }
    
}

int SumoRobot::Velocity2PWMRightWheel(float velocity)
{
    return int(coeffR*velocity + aR);
}

int SumoRobot::Velocity2PWMLeftWheel(float velocity)
{
    return int(coeffL*velocity + aL);  
}