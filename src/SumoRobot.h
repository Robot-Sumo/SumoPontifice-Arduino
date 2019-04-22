#ifndef _SUMOROBOT_H_
#define _SUMOROBOT_H_


// Arduino Nano
//Comandos del dispositivo
#define setLED_RGB 1 // Poner LED_RGB en un color determinado
#define setBearingVector 2 // Poner punto de referencia de direccion y velocidad
#define getBatteryState 3  // Leer voltaje de las baterias y enviar al master
#define startDistanceMeasure 4 // comando que solicita empezar a medir el dezplazamiento en las ruedas
#define setSampleFrecuency 6 // Configurar frecuencia de muestreo del dispositivo

// direcciones dispositivos conectados al puerto serial

#define masterDeviceAddress 2
#define pantiltDeviceAddress 255
#define thisDeviceAddress 1 

// *** Configuracion de pines ***
// Drivers Motores
// Driver 1 , izquierdo (Motor rueda derecha)
const int in1= 11; // pwm1
const int in2 = 9; // pwm2
const int diag = 10; // Diag (señal de sobre corriente)
// Driver 2 , derecho(B) (Motor rueda izquierda)
const int in1bAux= A0; // pwm1
const int in2bAux = A1; // pwm2
const int in1b = 6; // pwm1, activado hacia adelante
const int in2b = 5; // pwm2, activado hacia atrás
const int diagb = 4; // Diag (señal de sobre corriente)
// Voltaje de las baterias
const int vbatt1 = A6;
const int vbatt2 = A7;
// Encoders
const int encRightWheel = 3; // Encoder rueda izquierda
const int encLeftWheel = 2; // Encoder rueda derecha
// LEDS
const int led1 = 13;
const int led_Rpin = 7;
const int led_Gpin = 8; // 8
const int led_Bpin = 12;





// estados del dispositivo
int currentState, lastState;
bool goToPwm, goToStop;



// Variables
bool direction;
int pwmRightWheel = 0;
int pwmLeftWheel= 0;
unsigned long timeArriveCommand;


unsigned long encoderCounter;
float encoderPeriod;
const int maxCount = 70;
bool encoderFlag;
unsigned long encoderTime = 0;
bool ledState = LOW;
unsigned long counter = 0;

unsigned long timeEncoder;

bool state = LOW;

// VariablesRobot
String dataFromMaster= "";         // a String to hold incoming data



boolean toggle1 = 0;
unsigned long timeLast = 0;


enum colores
{
rojo,
verde,
azul,
amarillo,
turquesa,
morado,
blanco,
off
};

enum states
{
    setPWM,
    idle,
    stop
};

class SumoRobot {
    public:
        SumoRobot();
        void init();
        void EncoderRightWheel();
        void EncoderLeftWheel();
        void ledColor(int color);
        void setPwm(uint8_t pwmLeftWheel, uint8_t pwmRightWheel, bool direction);
        void StateMachine();

    private:
    


};


#endif