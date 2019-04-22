#include "SumoRobot.h"

SumoRobot::SumoRobot()
{
    
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
    encoderFlag = false;
    encoderCounter = 10;
    counter = 0;
    timeEncoder = micros();
    setPwm(0, 0, 0);
 

    // Configura timer 1 a 25 HZ
    cli();//stop interrupts

 
    //set timer1 interrupt at 21Hz
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    // set compare match register for 21hz increments
    OCR1A = 624;//= (16*10^6) / (25*1024) - 1 (must be <65536)
    //  = [ 16,000,000Hz/ (pr  ledColor(amarillo);
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS10 and CS12 bits for 1024 prescaler
    TCCR1B |= (1 << CS12) | (1 << CS10);  
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    sei();//allow interrupts

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




void SumoRobot::EncoderRightWheel(){
  /*
  
  if ( (micros()-timeEncoder) > 3000) // Si ha transcurrido mas de 1000 us desde la ultima interrupcion
  {
    timeEncoder = micros(); // a relacion de 15 de reduccion, maximo 200 rpm, 7.5 ms

      counter ++;
  
    state = !state;
    digitalWrite(LED_BUILTIN, state);
  }

  */
  
}


void SumoRobot::EncoderLeftWheel(){
    /*
   if ( (micros()-timeEncoder) > 3000) // Si ha transcurrido mas de 1000 us desde la ultima interrupcion
  {
    timeEncoder = micros(); // a relacion de 15 de reduccion, maximo 200 rpm, 7.5 ms

      counter ++;
  
    state = !state;
    digitalWrite(LED_BUILTIN, state);
  }
*/
}