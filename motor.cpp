#include "mbed.h"
#include "rtos.h"

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12
#include "mbed.h"
#include "rtos.h"

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8  

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

RawSerial pc(SERIAL_TX, SERIAL_RX);
Thread thread1(osPriorityNormal, 1024, NULL);
Thread thread2(osPriorityNormal, 1024, NULL);
Thread accuracy;
InterruptIn interrupt_cha(CHA);
Timer t;

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
const int8_t lead = 2;  //2 for forwards, -2 for backwards

//global variable counting CHA
float speedChanger = 1;
int counter_cha = 0;
int counter = 0;
float duty = 0.6;
float speed;
int counterA = 0;
float kp = 1;
float ki = -0.01;
float kd = 200;
float currentDuty = 0;
float e_error = 0;
float errorT;
float lastError = 0;
float proportional;
float integral;
float derivative;
float integralZone = 30;

//speed control
float espeed=0;
float checkDuty = 0;
float p;
float d;

float dutyS=0;
float currentDutys = 0;

int state=1;
int fstate=4;

 // string inputmessage="Please enter the command \n";
  bool isRotation = false;
  bool isSpeed = false;
  bool numberStart=false;
  int counterintrot = 0;
  int counterfloatrot =0;
  int counterintvel = 0;
  int counterfloatvel =0;
  int rot = 0;
  int vel= 0;
  int floatrot=0;
  int floatvel=0;
  
  //OUTPUTS
  bool backward=false;
  float outrotation=0;
  float outspeed=8;
  
  int desiredRotation = 0;
  float desiredSpeed=0;
//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
DigitalIn I1(I1pin);
DigitalIn I2(I2pin);
DigitalIn I3(I3pin);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//Set a given drive state
void motorOut(int8_t driveState){
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];  
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L = duty;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = duty;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = duty;
    if (driveOut & 0x20) L3H = 0;
    }
    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(1.0);
    
    //Get the rotor state
    return readRotorState();
}

void increment(){
    counter_cha++;
    if(counter_cha>117){
       t.stop();
       speed = 1/t.read();
       t.reset();
       t.start();
       counter_cha=1;
       counterA++;
        }
    }

bool isNumber (char c){
    if(c=='0'|| c=='1'|| c=='2'|| c=='3'|| c=='4'|| c=='5'|| c=='6'|| c=='7'|| c=='8'|| c=='9'){
        return true;
    }
    else{
        return false;
    }
}
float getVal(int intval, int floatval, int floatnumber, bool cond){
        if (!cond){
            return 10000.0;
        }else{
            if (floatnumber==0){
                return intval;
            }else if(floatnumber==1){
                return (intval + (float)floatval/10);
            }else{
                  return (intval + (float)floatval/100);  
            }
        } 
}

void readInput(bool numberStart,bool backward,int state, int fstate, bool isRotation,bool isSpeed,int counterintrot,int counterfloatrot, int counterintvel,int counterfloatvel,int rot,int vel,int floatrot,int floatvel,float outrotation, float outspeed){
    while(state!=fstate){
    char c = pc.getc();
    pc.printf("%c",c);
    
    if (c=='R' && state == 1){
        isRotation = true ;
        state=2;
    }else if(isNumber(c) && state==2 && (counterintrot<3 || counterintvel <3) ){
        numberStart=true;
        if (isRotation && !isSpeed){
            rot = 10*rot + c - '0';
            counterintrot++;
        }else{
            vel=10*vel + c - '0';
            counterintvel++;
            }
    }else if(isNumber(c) && state==3 && (counterfloatrot<2 || counterfloatvel <2)){
        if (isRotation && !isSpeed){
            floatrot = 10*floatrot + c - '0';
            counterfloatrot++;
        }else{
            floatvel=10*floatvel + c - '0';
            counterfloatvel++;
        }
    }else if(c=='.' && state==2){
        state=3; 
    }else if(c=='V' && (state== 1 || state==2 || state==3)){
        isSpeed = true;
        counterintvel = 0;
        counterfloatvel =0;
        state=2;
    }else if(c=='\r' && (state==2 || state==3)   && (counterintrot<4 && counterintvel <4) && (counterfloatrot<3 && counterfloatvel <3) ){
      state= 4;   
    }else if(c=='-' && state==2 && numberStart==false){
        backward=true;
    }else{
        state=1;
        pc.printf("\n\rPlease enter the command \n\r");
        isRotation = false;
        isSpeed = false;
        numberStart=false;
        backward=false;
        counterintrot = 0;
        counterfloatrot =0;
        counterintvel = 0;
        counterfloatvel =0;
        rot = 0;
        vel= 0;
        floatrot=0;
        floatvel=0;    
    }
  }
  
  desiredRotation = getVal(rot, floatrot, counterfloatrot, isRotation);
  desiredSpeed = getVal(vel, floatvel, counterfloatvel, isSpeed);
  pc.printf("Number of rotation: %i\n\r", desiredRotation);
  pc.printf("Speed: %f\n\r", desiredSpeed);   
}

void speedControl(){
    while(1){
        if((desiredSpeed != 10000) && (counterA < 0.5*desiredRotation)){
            espeed = desiredSpeed - speed;
            if(counterA<10){
                duty = 1;
            }
            else{
                if(espeed > 0){
                    duty = 1;
                }
                if(espeed < 0){
                    duty= 0;
                }
            }
        } 
        Thread::wait(18);
    }
}

//Control function
void control(){ 
    while(1){
        if((desiredRotation != 10000) && (counterA >= 0.5*desiredRotation)){
            integralZone = 0.5*desiredRotation;  
            e_error = desiredRotation - counterA;
            if(e_error < integralZone && e_error != 0){
                errorT += e_error;
            }
            else{
                errorT = 0;
            }
            if(e_error > abs(50 / ki)){
                errorT = -50 / ki;
            }
            if(e_error == 0){
                derivative = 0;
            }
            proportional = e_error * kp;
            integral = errorT * ki;
            derivative = (e_error - lastError) * kd;
            lastError = e_error;
            currentDuty = proportional + integral + derivative;
            if(currentDuty < 0){
                currentDuty = 0;
            }
            else if(currentDuty > 1){
                currentDuty = 1;
            }
            duty = currentDuty;
            if(currentDuty < 0.2){
                duty = 0;
            }
            if(counter>desiredRotation){
                duty=0;
            }
        }
        Thread::wait(18);
    }
}

int main() {
    interrupt_cha.rise(&increment);
    int8_t orState = 0;    //Rotot offset at motor state 0
    pc.printf("\n\rPlease enter the command \n\r");
    readInput(numberStart,backward,state, fstate, isRotation, isSpeed, counterintrot,counterfloatrot, counterintvel,counterfloatvel,rot,vel,floatrot,floatvel,desiredRotation,desiredSpeed);
    thread1.start(control);
    thread2.start(speedControl);
    //Initialise the serial port
    int8_t intState = 0;
    int8_t intStateOld = 0;
    //pc.printf("Hello\n\r");
    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states
    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    while (1){
        intState = readRotorState();//currentstate
        if (intState != intStateOld) {
            intStateOld = intState;
            motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
           if(intStateOld==orState){
               // t.stop();
                //speed = 1/t.read();
                //pc.printf("timer: %f\n\r", speed);
                //t.reset();
                //t.start();
                //counter++;
                //pc.printf("counter: %i\n\r", counterA);
            }
        }
    }
}

