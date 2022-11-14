/*

FBOT VSSS RADIO PROTOCOL

numbers 1,2,3,129,130,131 are ID exclusive
0 is exclusive for no movement
128 is not used 

*/

//Radio Messages Ranges:
#define NUMOFBYTES 3
#define MAXCLOCKWISE 127
#define MINCLOCKWISE 4
#define MAXANTICLOCKWISE 255
#define MINANTICLOCKWISE 132

//Ranges
#define MAXPWM 65535
#define MINPWM 0

//Orientations
#define CLOCKWISE 0
#define ANTICLOCKWISE 1

//Motor ID
#define LEFTMOTOR 0
#define RIGHTMOTOR 1

//Pin Definitions
#define MOTOR0PWM PB0 
#define MOTOR1PWM PB1

#define MOTOR0A PB3
#define MOTOR0B PB4
#define MOTOR1A PB5
#define MOTOR1B PB6

#define IDSWITCH1 PB12
#define IDSWITCH2 PB13
#define IDSWITCH3 PB14

//Radio Pins
#define RX PB11
#define TX PB10

uint8_t id=1;
short radioData[2]={0,0};
uint16_t motorSetPoint[2]={0,0};
bool motorOrientation[2];

HardwareSerial Serial3(RX, TX);

void GetId(){
    short switchRead;

    switchRead = digitalRead(IDSWITCH1);
    switchRead += digitalRead(IDSWITCH2)*2;
    switchRead += digitalRead(IDSWITCH3)*3;

    switch (switchRead){
    case 1:
        id=1;
        break;
    
    case 2:
        id=2;
        break;
    
    case 3:
        id=3;
        break;
    
    case 4:
        id=129;
        break;
    
    case 5:
        id=130;
        break;
    
    case 6:
        id=131;
        break;
    }
}

bool RadioRead(){
    if (Serial3.available() >= NUMOFBYTES){

        if(Serial3.read() == id){

            for(short i=0; i < NUMOFBYTES; i++){
                
                radioData[i]=Serial3.read();
                
                if (radioData[i] >= MINCLOCKWISE && radioData[i] <= MAXCLOCKWISE){
                    motorOrientation[i]=CLOCKWISE;
                    motorSetPoint[i]=map(radioData[i],MINCLOCKWISE,MAXCLOCKWISE,MINPWM,MAXPWM);
                    //motorSetPoint[i]=(MAXPWM*radioData[i])/(MAXCLOCKWISE);
                }

                else if (radioData[i] >= MINANTICLOCKWISE && radioData[i] <= MAXANTICLOCKWISE){
                    motorOrientation[i]=ANTICLOCKWISE;
                    motorSetPoint[i]=map(radioData[i],MINANTICLOCKWISE,MAXANTICLOCKWISE,MINPWM,MAXPWM);
                    //motorSetPoint[i]=(MAXPWM*radioData[i])/(MAXANTICLOCKWISE);
                }

                else if(radioData[i] == 0){
                    motorOrientation[i]=0;
                    motorSetPoint[i]=0;
                }
            }           
            Serial3.flush();
            return true;
        }
    }
    return false;
}

void MotorSetup(bool motorID, bool motorOrientation, uint16_t setPoint){
    if(motorID == LEFTMOTOR){
        if(motorOrientation == CLOCKWISE){
            digitalWrite(MOTOR0A,HIGH);
            digitalWrite(MOTOR0B,LOW);
            analogWrite(MOTOR0PWM,setPoint);
        }
        else{
            digitalWrite(MOTOR0A,LOW);
            digitalWrite(MOTOR0B,HIGH);
            analogWrite(MOTOR0PWM,setPoint);
        }
    }
    else{
        if(motorOrientation == CLOCKWISE){
            digitalWrite(MOTOR1A,HIGH);
            digitalWrite(MOTOR1B,LOW);
            analogWrite(MOTOR1PWM,setPoint);
        }
        else{
            digitalWrite(MOTOR1A,HIGH);
            digitalWrite(MOTOR1B,LOW);
            analogWrite(MOTOR1PWM,setPoint);
        }
    }
}

void MoveRobot(){
    for(short i=0;i<2;i++){
        MotorSetup(i,motorOrientation[i],motorSetPoint[i]);
    }
}

void setup(){
    
    //disableDebugPorts();
    
    Serial3.begin(9600);
    Serial.begin(9600);
    
    pinMode(MOTOR0PWM, OUTPUT);
    pinMode(MOTOR1PWM, OUTPUT);

    pinMode(MOTOR0A, OUTPUT);
    pinMode(MOTOR0B, OUTPUT);
    pinMode(MOTOR1A, OUTPUT);
    pinMode(MOTOR1B, OUTPUT);

    pinMode(IDSWITCH1, INPUT);
    pinMode(IDSWITCH2, INPUT);
    pinMode(IDSWITCH3, INPUT);

}

void loop(){
    
    if (RadioRead() == true){
        MoveRobot();
    }

}