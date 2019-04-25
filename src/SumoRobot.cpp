#include "SumoRobot.h"


long SumoRobot::encoderRightCounter;
long SumoRobot::encoderLeftCounter;
unsigned long SumoRobot::encoderRightTime;
unsigned long SumoRobot::encoderLeftTime;

bool SumoRobot::encoderRightFlag;
bool SumoRobot::encoderLeftFlag;
bool SumoRobot::direction;




// timer1Isr        
int SumoRobot::encoderRightMeasure;
int SumoRobot::encoderLeftMeasure;
long SumoRobot::encoderRightSample;
long SumoRobot::encoderLeftSample;


        // buffer de salida
int SumoRobot::encoderRightBuffer[100];
int SumoRobot::encoderLeftBuffer[100];
int SumoRobot::encoderRightBufferIndex;
int SumoRobot::encoderLeftBufferIndex;



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
            if(goToPwm) 
            {
                goToPwm = false;
                currentState = states::setPWM;
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
        case states::setPWM:
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
    
   
    goToPwm = false;
    goToStop = true;
    currentState = idle;
    timeArriveCommand = millis();
    pwmLeftWheel = 0;
    pwmRightWheel = 0;
    direction = true;

    toggle1 = 0;
    timeLast = 0;



    
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


    setPwm(0, 0, 0);
    Timer1.initialize(50000);                  // Initialise timer 1 100 m
    Timer1.attachInterrupt( timer1Isr );           // attach the ISR routine here
    Timer1.stop();

    // Configura timer 1 a 25 HZ
    /*

    */
    

}

void SumoRobot::setPwm(uint8_t pwmLeftWheel, uint8_t pwmRightWheel, bool direction)
{

    if(direction) // forward
    {
        analogWrite(in1, pwmRightWheel);
        digitalWrite(in2, LOW);
        analogWrite(in1b, pwmLeftWheel);
        digitalWrite(in2b, LOW);
        ledColor(verde);
    }
    else  // reverse
    {
        analogWrite(in2, pwmRightWheel);
        digitalWrite(in1, LOW);
        analogWrite(in2b, pwmLeftWheel);
        digitalWrite(in1b, LOW);
        ledColor(azul);
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


void SumoRobot::timer1Isr()
{
       //timer1 interrupt 21Hz toggles pin 13 (LED)
    //generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
   // Serial.println(micros()-timeLast);
    //timeLast = micros();
    encoderLeftMeasure = int(encoderLeftCounter- encoderLeftSample);
    encoderLeftBuffer[encoderLeftBufferIndex] = encoderLeftMeasure;

    encoderRightFlag = int(encoderRightCounter- encoderRightSample);
    encoderRightBuffer[encoderRightBufferIndex] = encoderRightMeasure;
    
    if (toggle1){
        digitalWrite(13,HIGH);
        toggle1 = 0;
    }
    else{
        digitalWrite(13,LOW);
        toggle1 = 1;
    }

    encoderLeftSample = encoderLeftCounter;
    encoderRightSample = encoderRightCounter;
    encoderRightBufferIndex++;
    encoderLeftBufferIndex++;

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
            goToPwm = true;
            
        }
        */
        dataFromMaster ="";
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
            pwmLeftWheel = dataFromMaster[2];
            pwmRightWheel = dataFromMaster[3];
            direction = (dataFromMaster[4] == 1); // forward o reverse  
            break;

        case getBatteryState:
            break;
        
        case startEncoderSampling:
            Timer1.start();
            break;

        case setSampleFrecuency:
            
            break;

        case getBufferData:

            sendBufferData();
            encoderRightBufferIndex = 0;
            encoderRightBufferIndex = 0;

            break;


    }
}


void SumoRobot::sendBufferData()
{
    uint8_t size ;
    
    Serial.write(Trama, size);


}
