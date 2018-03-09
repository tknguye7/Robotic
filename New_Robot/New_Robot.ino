//#include <Adafruit_GFX.h>
//#include <Adafruit_LEDBackpack.h>

#include <QTRSensors.h>

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2
int highcount=0;

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {3, 4, 5, 6, 7, 8, 9, 10},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

volatile boolean flag1=0;
volatile boolean flag2=0;
volatile unsigned long cnt1=0;
volatile unsigned long cnt2=0;
volatile boolean diagflag=0;
volatile boolean markflag=0;
volatile boolean stopflag=0;
volatile boolean R1flag=0;

long distGlobal;
long usespe;
long speedl;
long speedr;
int marker;
int boardpos;
int targetpos;
int turnang;
byte turndir;
int markcount;
int coinspicked=0;

byte diagonalmotion;

void initializeTimerCounters(){
  noInterrupts();  
  
  TCCR1A = 0;                             /**Initializes Timer 1**/
  TCCR1B = 0;                             //
  TCNT1  = 0;                             //
  TCCR1B |= (1 << WGM12);                 // CTC mode
  TCCR1B |= (1 << CS10);                  // prescaler, no prescaling
  TIMSK1 |= (1 << OCIE1A);                // enable timer compare interrupt
  TCCR3A = 0;                             /**Initializes Timer 3**/
  TCCR3B = 0;                             //
  TCNT3  = 0;                             //
  TCCR3B |= (1 << WGM32);                 // CTC mode
  TCCR3B |= (1 << CS30);                  // prescaler, no prescaling
  TIMSK3 |= (1 << OCIE3A);                // enable timer compare interrupt
  
  interrupts();
}

//  Adafruit_BicolorMatrix matrix = Adafruit_BicolorMatrix();

//global vars

  const float min_distance = .1;

//calibration
  const float pi = 3.14159;
  const float rotation = (60 * pi)/25.4;
  const float steps_per_inch = 1590/rotation;
  float steps_per_degree = 25.8 ; //15.98;
  
//motion pins
  byte motion_all = B01010101; //0x55 
  byte motion_45_right = B00010100; //0x11 
  byte motion_45_left = B01000001; //0x44 
  byte motion_left_wheels = B01010000; //0x05
  byte motion_right_wheels = B00000101; //0x50  //only checked all right and left  others may be wrong
  byte motion_rear = B00010001; //0x14
  byte motion_front= B01000100; //0x41

/*wheel configurations
back left: 6-7, 1=ccw
front left: 4-5, 1=cw
back right: 2-3, 1=ccw
front right: 01, 1=cw
*/
//direction pins
  byte fwd = B10000010; //0x80 forward  back left 1
  byte rev = B00101000; //0x2A reverse
  byte cw = B10001000; //0x8A clockwise
  byte ccw = B00100010; //0x20 counterclockwise
  byte left = B10101010; //0x02 //00001010
  byte right = B00000000; //0xA8 //10100000

//for grid_search()
  int x = 0;
  int y = 0;
  float set_dist = 13.8;
  float wall_dist = 0;
  float distance_btwn_sensors = 7.25;
  float sos = 0.006756;
  float sos_over_dbs = sos / distance_btwn_sensors; //sos -> speed of sound 
//  float find_angle(NewPing front_sensor, NewPing rear_sensor, int N=16);

  int token[] = {0,0,0,0,0,0,0,0};



ISR(TIMER1_COMPA_vect){
//toggle motor 1
if((flag1==1)&& (stopflag==0)){
  if(diagflag==0){
    PORTL ^= motion_left_wheels;
    cnt1++;
  }
  else{
    PORTL ^= diagonalmotion;
    cnt1++;
  }
}
if(cnt1>distGlobal|cnt2>distGlobal){
  flag1=0;
}
}  

ISR(TIMER3_COMPA_vect){
// if motorFlag == true
//toggle motor 2
if((flag2==1)&& (stopflag==0)){
  if(diagflag==0){
    PORTL ^= motion_right_wheels;
    cnt2++;
  }
}
if(cnt2>distGlobal|cnt1>distGlobal){
flag2=0;
}
}
  
void setup() {
  
  Serial.begin(9600); // Open serial monitor at 115200 baud to see ping results.
  byte allOutputs = B11111111;
  DDRL = allOutputs;
  initializeTimerCounters();//exactly what it sounds like

    noInterrupts();  //prevent interrupts
    OCR1A= 900000;
    OCR3A= 900000;
    
    interrupts();  //allow interrupts

manualCalibrate();
//sensorLogic();
//pickUp(120, 200, fwd);
//follow(120, 200, fwd);
//rotate(720,100,ccw);
//go(36,450,fwd);
Round1();
  /*  PORTL = fwd;
    for (int i=0;i<2000;i++){
      PORTL ^= B01010101;
      delayMicroseconds(1600);

    
    }*/
    
/*accrobot(36,200,fwd,8);
accrobot(36,200,rev,8);
accrobot(36,200,fwd,8);
accrobot(36,200,rev,8);
accrobot(36,200,fwd,8);
accrobot(36,200,rev,8);
accrobot(36,200,fwd,8);
accrobot(36,200,rev,8);
accrobot(36,200,fwd,8);
accrobot(36,200,rev,8);
accrobot(36,200,fwd,8);
accrobot(36,200,rev,8);*/
  
}

//function defs


/*
 * FUNCTION
 * 
 * go
 * 
 * move forward or reverse
 * distance in inches
 * speed == delay -> lower is faster
 */

void rotate(float distance, int speed, byte direction){
    PORTL = direction;

    //distance calibration calculation
    float steps_f = distance * steps_per_degree;
    distGlobal = steps_f;
    usespe = 900000/speed;
      
    noInterrupts();  //prevent interrupts
    OCR1A= usespe;
    OCR3A= usespe;
    
    interrupts();  //allow interrupts
    //reset count
    cnt1=0;
    cnt2=0;
    //start interrupts
    flag1=1;
    flag2=1;
    //wait until finished
    while(flag1==1){
    }
 }


void decideTurn(){
  int diffPos = targetpos - boardpos;
  if (diffPos > 0){
    if(diffPos > 4){
      turndir = cw;
    }
    else{
      turndir = ccw;
    }
  }
  else{
    if(diffPos < -4){
      turndir = ccw;
    }
    else{
      turndir = cw;
    }
  }
  diffPos = abs(diffPos);
  if (diffPos > 4){
    diffPos = 8 - diffPos;
  }
  turnang =180 - (diffPos * 45);
}

void followenter(){
  if(boardpos % 2 == 1){
    go(7.5,100,fwd);
  }
  else{
    go(6,100,fwd);
  }
}

void spokes2(){
  boardpos = 0;
  accrobot(42,250,rev,12);
  boardpos = 7;
  rotate(45,100,cw);
  followenter();
  follow(50,200,fwd);
  targetpos = 2;
  decideTurn();
  delay(100);
  rotate(turnang,100,turndir);
  boardpos = targetpos;
  followenter();
  follow(50,200,fwd);
  delay(100);
  rotate(180,100,cw);
    followenter();
  follow(50,200,fwd);
  targetpos = 1;
  decideTurn();
  delay(100);
  rotate(turnang,100,turndir);
  boardpos = targetpos;
  followenter();
  follow(50,200,fwd);
  delay(100);
  rotate(180,100,cw);
    followenter();
  follow(50,200,fwd);
  targetpos = 5;
  decideTurn();
  delay(100);
  rotate(turnang,100,turndir);
  boardpos = targetpos;
  followenter();
  follow(50,200,fwd);
  delay(100);
  rotate(180,100,cw);
    followenter();
  follow(50,200,fwd);
  targetpos = 6;
  decideTurn();
  delay(100);
  rotate(turnang,100,turndir);
  boardpos = targetpos;
  followenter();
  follow(50,200,fwd);
  delay(100);
  rotate(180,100,cw);
    followenter();
  follow(50,200,fwd);
  targetpos = 3;
  decideTurn();
  delay(100);
  rotate(turnang,100,turndir);
  boardpos = targetpos;
  followenter();
  follow(50,200,fwd);
  delay(100);
  rotate(180,100,cw);
    followenter();
  follow(50,200,fwd);
  targetpos = 7;
  decideTurn();
  delay(100);
  rotate(turnang,100,turndir);
  boardpos = targetpos;
  followenter();
  follow(50,200,fwd);
  delay(100);
  rotate(180,100,cw);
}

//void spokes(){
//  boardpos = 0;
//  accrobot(42,250,rev,12);
//  boardpos = 7;
//  rotate(45,100,cw);
//  go(7.5,100,fwd);
//  follow(50,200,fwd);
//  targetpos = 1;
//  decideTurn();
//  delay(100);
//  rotate(turnang,100,turndir);
//  go(7.5,100,fwd);
//  follow(50,200,fwd);
//  delay(100);
//  rotate(180,100,cw);
//}
 
void sensorLogic()
{
    highcount=0;
    //qtrrc.read(sensorValues);
 if(cnt1 > markcount && markflag == 0){
 for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    if(sensorValues[i] > 750){
      //Serial.print("High");
      highcount++;  
    }
     
    /*else{
      Serial.print("Low");
    }*/
    //Serial.print(sensorValues[i]);
    //Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  }
  if(highcount > 2){
      marker++;
      markcount = cnt1 + 750;
    } 
  if (marker > 3){
      noInterrupts();
      cnt1 = 0;
      cnt2 = 0;
      distGlobal = 1500;
      interrupts();
      markflag = 1;
      //flag2 = 0;
      //flag1 = 0;
    }
  //Serial.print(highcount);
  //Serial.println();
  
  //delay(250);
 }
}

 
void manualCalibrate()
{
qtrrc.calibrate();
for(int i = 0; i < NUM_SENSORS; i++){
    qtrrc.calibratedMinimumOn[i] = 250;
    qtrrc.calibratedMaximumOn[i] = 1100;
  }
}

void follow(float distance, int spe, byte dir){
    markcount = 0;
    marker = 0;
    markflag = 0;
    PORTL = dir; //B10000000;
  
    //distance calibration calculation
    long steps_f = distance * steps_per_inch;
    distGlobal = steps_f;
    usespe = 900000/spe;
      
    noInterrupts();  //prevent interrupts
    OCR1A= usespe;
    OCR3A= usespe;
    interrupts();  //allow interrupts
    //reset count
    cnt1=0;
    cnt2=0;
    //start interrupts
    flag1=1;
    flag2=1;
    //wait until finished
    int poserror = 0;
    int lasterror = 0;
    float followspeed = 0;
    int modspe1 = 0;
    int modspe3 = 0;
    float multspe = 1;
    while(flag1==1){
      lasterror = poserror;
      poserror = qtrrc.readLine(sensorValues)-3500;
      sensorLogic();
      followspeed = 12 * poserror + 15 * (poserror - lasterror);
      multspe = abs(followspeed) / 10000 + 1;
      if(multspe > 2.2){
        multspe = 2.2;
      }
      //modspe1 = usespe - followspeed / spe * 50;
      //modspe1 = (900000 - followspeed / 50) / spe;
      //modspe3 = usespe + followspeed / spe * 50;
      //modspe3 = (900000 + followspeed / 50) / spe;
      if (followspeed > 0){
        modspe1 = usespe / multspe;
        modspe3 = usespe * multspe;
      }
      else {
        modspe1 = usespe * multspe;
        modspe3 = usespe / multspe;
      }
      //noInterrupts();  //prevent interrupts
        OCR1A= modspe1;
        OCR3A= modspe3;
      //interrupts();  //allow interrupts
      delayMicroseconds(50);
    }
    if(boardpos % 2 == 1){
      go(11.5,100,fwd);
    }
    else{
      go(6.5,100,fwd);
    }
 }

 void pickUp(float distance, int spe, byte dir){
    markcount = 0;
    marker = 0;
    markflag = 0;
    PORTL = dir; //B10000000;
  
    //distance calibration calculation
    long steps_f = distance * steps_per_inch;
    distGlobal = steps_f;
    usespe = 900000/spe;
      
    noInterrupts();  //prevent interrupts
    OCR1A= usespe;
    OCR3A= usespe;
    interrupts();  //allow interrupts
    //reset count
    cnt1=0;
    cnt2=0;
    //start interrupts
    flag1=1;
    flag2=1;
    //wait until finished
    int picked = token[boardpos];
    int goalSpoke = 0;
    int poserror = 0;
    int lasterror = 0;
    float followspeed = 0;
    int modspe1 = 0;
    int modspe3 = 0;
    float multspe = 1;
    int coinFlag =0;
    int cntcnt1=0;
    int cntcnt2=0;
    if(picked == 0){
      goalSpoke = 2;
    }else if(picked == 1){
      goalSpoke = 4;
    }else{
      goalSpoke = 6;
    }
    while(flag1==1){
      lasterror = poserror;
      poserror = qtrrc.readLine(sensorValues)-3500;
      sensorLogic();
      if((marker == goalSpoke) && (coinFlag == 0 ) ){
        stopflag=1;
        for (int i=0; i<400; i++){
          PORTL ^= motion_all;
          delayMicroseconds(900);
        }
        delay(500);
        for (int i=0; i<200; i++){
          PORTL ^= motion_all;
          delayMicroseconds(900);
        }
        stopflag=0;
        token[boardpos] ++;
        coinFlag++;
        coinspicked ++;
      }
      followspeed = 12 * poserror + 15 * (poserror - lasterror);
      multspe = abs(followspeed) / 10000 + 1;
      if(multspe > 2.2){
        multspe = 2.2;
      }

      if (followspeed > 0){
        modspe1 = usespe / multspe;
        modspe3 = usespe * multspe;
      }
      else {
        modspe1 = usespe * multspe;
        modspe3 = usespe / multspe;
      }
      //noInterrupts();  //prevent interrupts
        OCR1A= modspe1;
        OCR3A= modspe3;
      //interrupts();  //allow interrupts
      delayMicroseconds(50);
    }
    if(boardpos % 2 == 1){
      go(11.5,100,fwd);
    }
    else{
      go(6.5,100,fwd);
    }
 }
 
void go(float distance, int speed, byte direction){
    PORTL = direction; //B10000000;
  
    //distance calibration calculation
    long steps_f = distance * steps_per_inch;
    distGlobal = steps_f;
    usespe = 900000/speed;
      
    noInterrupts();  //prevent interrupts
    OCR1A= usespe;
    OCR3A= usespe;
    interrupts();  //allow interrupts
    //reset count
    cnt1=0;
    cnt2=0;
    //start interrupts
    flag1=1;
    flag2=1;
    //wait until finished
    while(flag1==1){
    }

 }


/*
 * FUNCTION
 * 
 * diagonal
 * 
 * moves on 45 degree diagonal
 * 
 * distance in inches
 * speed == delay -> lower is faster
 * direction = fwd or rev
 * motion = motion_45_right or motion_45_left
 * 
 * northwest => direction = fwd, motion = motion_45_left
 * northeast => direction = fwd, motion = motion_45_right
 * southwest => direction = rev, motion = motion_45_right
 * southeast => direction = rev, motion = motion_45_left
 */

void diagonal(float distance, int speed, byte direction, byte motion){
    PORTL = direction;
    diagonalmotion = motion;

    //distance calibration calculation
    float steps_f = distance * steps_per_inch * 2.0;
    distGlobal = steps_f;
    usespe = 900000/speed;
    diagflag=1;
    
    noInterrupts();  //prevent interrupts
    OCR1A= usespe;
    OCR3A= usespe;
    
    interrupts();  //allow interrupts
    //reset count
    cnt1=0;
    cnt2=0;
    //start interrupts
    flag1=1;
    flag2=1;
    //wait until finished
    while(flag1==1){
    }
    diagflag=0;
 }

/*
 * FUNCTION
 * 
 * arc
 * 
 * goes in an arc
 * 
 * distance in inches
 * speed == delay -> lower is faster
 * direction = fwd or rev
 * motion = motion_left_wheels or motion_right_wheels
 * 
 * arc left => motion_right_wheels
 * arc right => motion_left_wheels
 */

void arcright(float distance, int speed, byte direction){
    PORTL = direction;

    //distance calibration calculation
    float steps_f = distance * steps_per_inch;
    distGlobal = steps_f;
    usespe = 900000/speed;
    speedl = usespe;
    speedr = usespe*4;
    noInterrupts();  //prevent interrupts
    OCR1A= speedl;
    OCR3A= speedr;
    interrupts();  //allow interrupts
    //reset count
    cnt1=0;
    cnt2=0;
    //start interrupts
    flag1=1;
    flag2=1;
    //wait until finished
    while(flag1==1){
    }
    
} 

void arcleft(float distance, int speed, byte direction){
    PORTL = direction;

    //distance calibration calculation
    float steps_f = distance * steps_per_inch;
    distGlobal = steps_f;
    usespe = 900000/speed;
    speedl = usespe*4;
    speedr = usespe;
    noInterrupts();  //prevent interrupts
    OCR1A= speedl;
    OCR3A= speedr;
    interrupts();  //allow interrupts
    //reset count
    cnt1=0;
    cnt2=0;
    //start interrupts
    flag1=1;
    flag2=1;
    //wait until finished
    while(flag1==1){
    }
    
} 




void accrobot(float distance, int speed, byte direction, float nsteps){  //step by step acceleration in a linear direction
usespe = 900000/speed;
long steps_f = distance * steps_per_inch;
long distmoved = 0;
//choosing direction
PORTL = direction;
//set distance of acceleration steps
float stepsize = 12;
float accspe;

for (int i = 0; i < 2 * nsteps + 1; i++){

if(i <= nsteps){  //ramp up speed
accspe = (i + 1)/nsteps;
}
if(i > nsteps){  //ramp down speed
accspe = 1-((i - nsteps)/nsteps);  
}
  
noInterrupts();  //puts hold on interrupts
//set speed at particular acceleration step
OCR1A= usespe / accspe;
OCR3A= usespe / accspe;

interrupts();  //allow interrupts again
if(i<nsteps){
  distGlobal = stepsize *(i + 1);  
  distmoved += distGlobal;
}

if(i>nsteps){
  distGlobal = stepsize * (2* nsteps + 1 - i);
}

if(i==nsteps){  //set distance for constant speed step
  distmoved = distmoved * 2;
  distGlobal = steps_f - distmoved;
}

//reset count
cnt1=0;
cnt2=0;
//start interrupts
flag1=1;
flag2=1;
//wait until finished
while(flag1==1){
}
}
}


void Round1(){
  boardpos = 0;
  accrobot(42,250,rev,12);
  boardpos = 7;
  rotate(45,100,cw); //gets to yellow facing spoke

    while (R1flag==0){
      if (coinspicked==12){
        R1flag=1;
      }
      followenter();
       pickUp(50,200,fwd);
       //targetpos = 2;
       if (coinspicked % 6 == 1){
        targetpos=1; //hard coding color of coin
        }
       else if (coinspicked % 6 ==2){
          targetpos=2;
          }
       else if (coinspicked % 6 ==3){
          targetpos=3;
          }
       else if (coinspicked % 6 ==4){
          targetpos=5;
          }
       else if (coinspicked % 6 ==5){
          targetpos=6;
          }
       else if (coinspicked % 6 ==0){
          targetpos=7;
          }
       decideTurn();
       delay(100);
       rotate(turnang,100,turndir);
       boardpos = targetpos;
       followenter();
       follow(50,200,fwd);
       delay(100);
       rotate(180,100,cw);
    }
    follow(50,200,fwd);
    targetpos=0;
    decideTurn();
    rotate(turnang,100,turndir);
    accrobot(42,250,fwd,12);
}


void loop() {
   


}
