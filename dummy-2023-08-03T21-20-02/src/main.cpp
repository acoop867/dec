/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Student                                          */
/*    Created:      Thu Jul 13 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// l1                   motor         5               
// l2                   motor         20              
// l3                   motor         17              
// r1                   motor         3               
// r2                   motor         12              
// r3                   motor         14              
// Inertial4            inertial      18              
// lr                   rotation      11              
// rr                   rotation      21              
// Controller1          controller                    
// Intake               motor         4               
// cata                 motor         8               
// LimitSwitchA         limit         A               
// Arm                  motor         19              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;


int i = 0;
int prev=0;
void reset() {
  i=0;
  prev=0;
}

bool testl(){
  int t =0;
  float l=lr.position(degrees);
  for(int i=0; i<3;i++) {
    if(fabs(lr.position(degrees)-l)<3) {
      t++;
    }
    else return false;
  }
  if(t>3) return true;
  else return false;
}
bool testr(){
  int t =0;
  float l=rr.position(degrees);
  for(int i=0; i<3;i++) {
    if(fabs(rr.position(degrees)-l)<3) {
      t++;
    }
    else return false;
  }
  if(t>3) return true;
  else return false;
}

int pid(int dist,int sense,int s) {
  double kp,ki,kd,max;
  if (s==1) {
     kp=.03;
     ki=0.0;
     kd=0.1;
     max=50;
  }
  else{
     kp=.5;
     ki=0;
     kd=.05;
     max=60;
  }
  int p=dist-sense;
  if(p<50){
  i +=p;
  }
  else{
    i=0;
  }
  int d=p-prev;
  prev=p;
  if(p*kp+i*ki+d*kd>max) return max;
  else return p*kp+i*ki+d*kd;
}


void sl(int speed) {
  l1.spin(forward,speed,pct);
  l2.spin(forward,speed,pct);
  l3.spin(forward,speed,pct);
}
void sr(int speed) {
  r1.spin(forward,speed,pct);
  r2.spin(forward,speed,pct);
  r3.spin(forward,speed,pct);
}

void arc(int dist, int diff) {
  lr.resetPosition();
  rr.resetPosition();
  
  while(fabs((lr.position(degrees)+rr.position(degrees))/2-dist)>50||fabs(Inertial4.rotation(degrees)+diff)>4) {
  sl(pid(dist,lr.position(degrees),1)-pid(diff,Inertial4.rotation(degrees),2));
  sr(pid(dist,rr.position(degrees),1)+pid(diff,Inertial4.rotation(degrees),2));
  wait(10,msec);
  }
  sl(0);
  sr(0);
}

int thr() {
  while(true) {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1,1);
  Controller1.Screen.print(lr.position(degrees));
  Controller1.Screen.setCursor(2,1);
  Controller1.Screen.print(rr.position(degrees));
  Controller1.Screen.setCursor(3,1);
  Controller1.Screen.print(Inertial4.rotation(degrees));
  wait(25,msec);
  }return 0;
}

void swingl(float tim) {
  sl(60);
  wait(tim,sec);
  sl(0);
}

void swingrb(float tim) {
  
  sl(-60);
  sr(-10);
  wait(tim,sec);
  sl(0);
  sr(0);
}

void turnm(int deg) {
  while(fabs(deg-Inertial4.rotation())>1.5&&!testr()&&!testl()) {
  sl(pid(deg,Inertial4.rotation(degrees),2));
  sr(-pid(deg,Inertial4.rotation(degrees),2));
  }
  sl(0);
  sr(0);
}
void df(float tim,bool s) {int speed=60;
  if(s) {
    speed=30;
  }
  sl(speed);
  sr(speed);
  wait(tim,sec);
  sl(0);
  sr(0);
}
void df(float tim) {
  sl(60);
  sr(60);
  wait(tim,sec);
  sl(0);
  sr(0);
}
void db(float tim,bool sp) {int spp = -60; if(sp) {spp=-30;}
  sl(spp);
  sr(spp);
  wait(tim,sec);
  sl(0);
  sr(0);
}
void db(float tim) {
  sl(-60);
  sr(-60);
  wait(tim,sec);
  sl(0);
  sr(0);
}

void intake(bool s,bool dir) {
  if(s&&dir) {
    Intake.spin(forward,100,pct);
  }
  else if(s) {
    Intake.spin(reverse,100,pct);
  }
  else{
    Intake.stop();
  }
}

void cat() {
  cata.spin(forward,100,pct);
  wait(.6,seconds);
  cata.stop(hold);
}
bool n=true;
void cataspin() {
  while(true){
    while(n) {
    if(LimitSwitchA.pressing() and !Controller1.ButtonL2.pressing()){
      cata.stop(hold);
    }else{
      cata.spin(fwd,100,pct);
    }
    wait(1,msec);
    }
  wait(1,msec);
  }
}

bool f=true;

void cataspinauto(){
  if(LimitSwitchA.pressing()){
    f=false;
  }
  if(!LimitSwitchA.pressing()){
    f=true;
  }
  if(f){
    while(!LimitSwitchA.pressing()){
    cata.spin(fwd, 100, pct);
  }
  }
  if(!f){
    cata.spin(fwd, 100, percent);
    wait(0.2, seconds);
    while(!LimitSwitchA.pressing()){
      cata.spin(fwd, 100, pct);
    }
  }
  
  cata.stop(hold);
}

void wingss(){
    wait(.6, seconds);

    wingL.set(1);
    wingR.set(1);
  
}




void dfl() {
  sl(-40);
  sr(-40);
  wait(.7,seconds);
  int x=0;
  while(lines.reflectivity()<18&&x<50) {
    wait(10,msec);
    x++;
  }
  sl(0);
  sr(0);
}

competition Comp;


int choice=0;
void auton() {
  
  if(choice == 4) { //test
    turnPID(90,100,-1);
  intake(true,false);
  wait(.5,sec);
  turnPID(-45,100,-1);
  intake(false, true);
  wait(.5,sec);
  turnPID(135, 100, -1);
  
  intake(true,true);

  wait(1,sec);
  }
  if(choice == 0) { //defense
    /*df(.5);
    turnm(-10);
    db(.4);
    turnm(40);
    df(.8);
    db(.3);
    turnm(95);
    df(.5);
    db(.2);
    turnm(240);
    df(1);
    db(.2);
    turnm(150);
    df(.4);
    turnm(100);
    df(.5);
    db(.01);*/
    thread t (cataspinauto);
    db(0.7);
    turnPID(-45, 100, 400);
    forwardPID(13, 100, -1);
    turnPID(45, 100, 400);
    forwardPID(10, 100, -1);
    turnPID(45, 100, 200);
   
    intake(true, true);
    db(0.8);
    wait(1, seconds);
    cataspinauto();
    
    turnPID(190, 180, -1);
    forwardPID(10, 100, -1);
    turnPID(90, 90, -1);
    wingR.set(1);
    db(0.5);
    wingR.set(0);
    turnPID(45, 100, -1);
    cataspinauto();
    forwardPID(12, 100, -1);

    turnPID(-45, 100, -1);
    
    df(0.45);
    turnPID(-45, 100, -1);
   


  }
  if(choice ==1) { //offense
    intake(true,false);
    db(.7);
    
    
    wait(.2,seconds);
    df(.4,true);
    //turnm(101);
    turnPID(108, 100, -1);

    intake(true,true);
    df(1.9,true);
    wait(.5,seconds);
    db(.2);
    turnPID(180, 100, -1);
    //df(.285);
    forwardPID(11, 100, -1);
    intake(false, true);
    //turnm(-88);
    turnPID(270, 100, -1);
    intake(true,false);
    wait(.6,seconds);
    df(.27);
    //turnm(-85);
    
    df(.3,true);

    db(.6);
    turnPID(270, 100, 300);
    wingL.set(1);
    df(.35,true);
    wingL.set(0);
    df(.3);

    db(.6);
    turnPID(305, 100, 500);
    
    forwardPID(46, 100, -1);
 
    db(.5);
    // turnm(1);
    // intake(false,true);

    // dfl();
    // turnm(90);
    // df(.2);
    

    
    // wingL.set(1);
    // db(.4);
    // armtime(false, .5);
    // db(.7);
    // wingL.set(0);

    // df(.3);
    // turnm(-90);
    
    // intake(true,false);
    // df(.3);
    // wait(.4,seconds);
    // db(.1);
    // turnm(0);
    // wingL.set(1);
    // wait(.2,sec);
    // wingL.set(0);
    // df(.4);
  }
  if(choice == 3) { //offense bad

    
     intake(true,false);
    db(.7);
    
    
    wait(.2,seconds);
    df(.4,true);
    //turnm(101);
    turnPID(108, 100, -1);

    intake(true,true);
    df(1.9,true);
    wait(.5,seconds);
    db(.2);
    turnPID(180, 100, -1);
    //df(.285);
    forwardPID(11, 100, -1);
    intake(false, true);
    //turnm(-88);
    turnPID(270, 100, -1);
    intake(true,false);
    wait(.6,seconds);
    df(.27);
    //turnm(-85);
    
    df(.3,true);

    db(.6);
    turnPID(270, 100, 300);
    wingL.set(1);
    df(.35,true);
    wingL.set(0);
    df(.2);

    turnPID(45, 100, -1);
    df(.4);
    
     
    
    

  }
  if(choice == 2) { //skills
    cata.spin(forward, 100, pct);
    wait(35, seconds);
    cata.stop();
    
    db(.2);

    turnPID(-15, 100, -1);

    db(.6);
    df(.4);

    turnPID(40, 100, -1);



    


    db(2,true);
    turnPID(90, 100, -1);
    thread g(wingss);
    
    
    
    db(2);

    df(.5);
    db(1);
  }



}

void tog() {
  n=!n;
}

void driver() {
  bool ArmButtonP= true;//if both buttons are pressed for toggle
  
  
  int wingvar[2] = {0,0};
  int wingbutton[3] = {0,0,0};

  while (true) {
    sl(Controller1.Axis3.position()+Controller1.Axis1.position()/1.5);
    sr(Controller1.Axis3.position()-Controller1.Axis1.position()/1.5);

    cata.stop();
    if(Controller1.ButtonL2.pressing()){
      cata.spin(forward,100,pct);
    }

    Controller1.ButtonDown.pressed(tog);
    if(Controller1.ButtonB.pressing()){
      lift.set(true);
    }
    if(Controller1.ButtonX.pressing()){
      lift.set(false);
    }

    if(Controller1.ButtonR2.pressing() and wingbutton[0] == 0){
      wingbutton[0] = 1;
      if (wingvar[0] == 0 and wingvar[1] == 0){
        wingR.set(true);
        wingL.set(true);
        wingvar[0] = 1;
        wingvar[1] = 1;//ignore this comment
      }else{
        wingR.set(false);
        wingL.set(false);
        wingvar[0] = 0;
        wingvar[1] = 0;
      }
    }else if(!Controller1.ButtonR2.pressing()){
      wingbutton[0] = 0;
    }
    if(Controller1.ButtonY.pressing() and wingbutton[1] == 0){
      wingbutton[1] = 1;
      if (wingvar[0] == 0){
        wingL.set(true);
        wingvar[0] = 1;
      }else{
        wingL.set(false);
        wingvar[0] = 0;
      }
    }else if(!Controller1.ButtonY.pressing()){
      wingbutton[1] = 0;
    }
    if(Controller1.ButtonA.pressing() and wingbutton[2] == 0){
      wingbutton[2] = 1;
      if (wingvar[1] == 0){
        wingR.set(true);
        wingvar[1] = 1;
      }else{
        wingR.set(false);
        wingvar[1] = 0;
      }
    }else if(!Controller1.ButtonA.pressing()){
      wingbutton[2] = 0;
    }
    

    /*
    if(Controller1.ButtonR2.pressing()) {
      while(Controller1.ButtonR2.pressing()) {
        wait(10,msec);
      }
      wstate++;
      if(wstate>1) {
        wstate=0;
      }
      wingR.set(wstate);
      wingL.set(wstate);
    }
    */
    
    if (Controller1.ButtonL1.pressing()) {
      Intake.spin(forward,100,pct);
    }else if (Controller1.ButtonR1.pressing()) {
      Intake.spin(reverse,100,pct);
    }else{
      Intake.stop();
    }
    if (Controller1.ButtonR1.pressing() and Controller1.ButtonL1.pressing() and ArmButtonP){
      if (arms.value() !=1){
        arms.set(true);
        
      }else{
        arms.set(false);
      }
      ArmButtonP= false;
    }


    //wait(10,msec);
  }
}

void pre() {
  vexcodeInit();
  Inertial4.calibrate();
  choice = 1;
  while(!Comp.isEnabled()) {
    if(Brain.Screen.pressing()) {
      while(Brain.Screen.pressing()) {
        wait(10,msec);
      }
      choice++;
      if(choice>3) {
        choice =0;
      }
    }
    
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    if(choice == 0) {
      Brain.Screen.print("Defense");
    }
    if(choice == 1) {
      Brain.Screen.print("Offense");
    }
    if(choice == 2) {
      Brain.Screen.print("Skills");
    }
    if(choice == 3) {
      Brain.Screen.print("Offense touch");
    }


  }
  
}


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  Comp.autonomous(auton);
  Comp.drivercontrol(driver);

  pre();

  
  
  while(1){
    wait(100,msec);
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print(Intake.velocity(rpm));
  }

}