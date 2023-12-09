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


int li(){
  return (l1.position(degrees)+l2.position(degrees)+l3.position(degrees))/3;
}
int ri(){
  return (r1.position(degrees)+r2.position(degrees)+r3.position(degrees))/3;
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
void sl1(int speed) {
  l2.spin(forward,speed,pct);
  l3.spin(forward,speed,pct);
}
void sr1(int speed) {
  r1.spin(forward,speed,pct);
  r2.spin(forward,speed,pct);
}

void wings(int i) {
  if(i==1) {
    wingL.set(true);
    wingR.set(false);
  }
  else if (i==2) {
  wingR.set(true);
  wingL.set(false);
  }
  else if(i==3){
    wingL.set(true);
    wingR.set(true);
  }
  else if(i==0){
    wingL.set(false);
    wingR.set(false);
  }

}

void pid(int ang) {
  float p = ang-inert.rotation();
  float i = 0.000001;
  float d = ang;
  float kp;
  if(fabs(p)<91){
  kp = .299;
  }
  else{
  kp = 0.29;
  }
  float kd=.17;
  while((fabs(d)>.3||fabs(p)>5)&&fabs(p)>.5) {
    float prev = p;
    p=ang-inert.rotation();
    d=p-prev;

    l1.spin(reverse,p*kp+d*kd,pct);
    l2.spin(forward,p*kp+d*kd,pct);
    l3.spin(forward,p*kp+d*kd,pct);
    r1.spin(reverse,p*kp+d*kd,pct);
    r2.spin(reverse,p*kp+d*kd,pct);
    r3.spin(forward,p*kp+d*kd,pct);

    //sl(p*kp+d*kd);
    //sr(-p*kp+-d*kd);
    //Controller1.Screen.clearScreen();
    //Controller1.Screen.setCursor(1,1);
    //Controller1.Screen.print(inert.rotation());
    wait(10,msec);
  }
  sl(0);
  sr(0);
}

void intout() {
  Intake.spin(reverse,100,pct);
  wait(1,sec);
  Intake.stop();
}

void pidswingr(int ang) {
  float p = ang;
  float i = 0;
  float d = ang;

  float kp = .6;
  float kd=0.05;
  while((fabs(d)>.3||fabs(p)>40)&&fabs(p)>.5) {
    float prev = p;
    p=ang-inert.rotation();
    d=p-prev;

    //sl(p*kp+d*kd);
    sr(-p*kp+-d*kd);
    //Controller1.Screen.clearScreen();
    //Controller1.Screen.setCursor(1,1);
    //Controller1.Screen.print(inert.rotation());
    wait(10,msec);
  }
  
  sl(0);
  sr(0);
}

void pidswingl(int ang) {
  float p = ang;
  float i = 0;
  float d = ang;

  float kp = .6;
  float kd=0.05;
  while((fabs(d)>.3||fabs(p)>40)&&fabs(p)>.5) {
    float prev = p;
    p=ang-inert.rotation();
    d=p-prev;

    l1.spin(reverse,p*kp+d*kd,pct);
    l2.spin(forward,p*kp+d*kd,pct);
    l3.spin(forward,p*kp+d*kd,pct);
    //sr(-p*kp+-d*kd);
    //Controller1.Screen.clearScreen();
    //Controller1.Screen.setCursor(1,1);
    //Controller1.Screen.print(inert.rotation());
    wait(10,msec);
  }
  pid(ang);
  sl(0);
  sr(0);
}

void pidd(int dist, int ang) {
  l1.resetPosition();
  l2.resetPosition();
  l3.resetPosition();
  r1.resetPosition();
  r2.resetPosition();
  r3.resetPosition();
double kp=.08;


double kd = 0.11;
//dist=dist*4*3.25*3.14159/(5*360);
float p = dist;
float d=dist;

double kap = .03;

while((fabs(d)>.3||fabs(p)>40)&&fabs(p)>.5) {
    float prev = p;
    p=dist-(li()+ri())/2;
    d=p-prev;

    float ap = ang-inert.rotation();

    sl(p*kp+d*kd+ap*kap);
    sr(p*kp+d*kd-ap*kap);
    //Controller1.Screen.clearScreen();
    //Controller1.Screen.setCursor(1,1);
    //Controller1.Screen.print(kp);
    wait(10,msec);
  }
  sl(0);
  sr(0);
}

void pidd(int dist, int ang,int max) {
  l1.resetPosition();
  l2.resetPosition();
  l3.resetPosition();
  r1.resetPosition();
  r2.resetPosition();
  r3.resetPosition();
double kp=.08;


double kd = 0.11;
//dist=dist*4*3.25*3.14159/(5*360);
float p = dist;
float d=dist;

double kap = .03;

while((fabs(d)>.3||fabs(p)>40)&&fabs(p)>.5) {
    float prev = p;
    p=dist-(li()+ri())/2;
    d=p-prev;

    float ap = ang-inert.rotation();
    int pow = p*kp+d*kd;
    if(pow>max) {
      pow=max;
    }
    if(pow<-max) {
      pow = -max;
    }
    sl(pow+ap*kap);
    sr(pow-ap*kap);
    //Controller1.Screen.clearScreen();
    //Controller1.Screen.setCursor(1,1);
    //Controller1.Screen.print(kp);
    wait(10,msec);
  }
  sl(0);
  sr(0);
}

void arc(int dist, int ang) {
  l1.resetPosition();
  l2.resetPosition();
  l3.resetPosition();
  r1.resetPosition();
  r2.resetPosition();
  r3.resetPosition();
double kp=.09;


double kd = 0.05;
//dist=dist*4*3.25*3.14159/(5*360);
float p = dist;
float d=dist;

double kap = 2;

while((fabs(d)>.3||fabs(p)>40)&&fabs(p)>.5) {
    float prev = p;
    p=dist-(li()+ri())/2;
    d=p-prev;

    float ap = ang-inert.rotation();

    sl(p*kp+d*kd+ap*kap);
    sr(p*kp+d*kd-ap*kap);
    //Controller1.Screen.clearScreen();
    //Controller1.Screen.setCursor(1,1);
    //Controller1.Screen.print(kp);
    wait(10,msec);
  }
  sl(0);
  sr(0);
}

int catathing(){
  while(true) {
    if(Controller1.ButtonL2.pressing()) {
      cata.spin(forward,12,volt);
    }
    else {
    cata.stop();
    }
    /*if(cata.torque()>.1) {
      cata.setPosition(0, deg);
      cata.spin(forward,100,pct);
      while(cata.position(deg)<400) {
        wait(10,msec);
      }
      if(di.objectDistance(inches)>2) {
      cata.stop();
      waitUntil(di.objectDistance(inches)<2);
      }
      cata.spin(forward,100,pct);
    }
    else{
      cata.spin(forward,100,pct);
    }
    */
  }
  return 0;
}

void cataauto() {
  int times=0;
  cata.spin(forward,12,volt);
  while(times<44){
  if(di.objectDistance(inches)<2) {
      times++;
      while(di.objectDistance(inches)<2){
      wait(10,msec);
      }
  }
  }
  cata.stop();
}

void driver() {
  thread th(catathing);
  bool n = false;
  bool t= false;
  bool m = false;
  bool z = false;
  bool l = false;
  bool r = false;
  bool rr = false;
  bool lr = false;
  bool inta = false;
  int toggle = 0;
  while(true) {
    if(Controller1.ButtonX.pressing()) {
      toggle =1;
      pto.set(true);
    }
    if(Controller1.ButtonB.pressing()) {
      toggle =0;
      pto.set(false);
    }
    if(Controller1.ButtonDown.pressing()&&Controller1.ButtonLeft.pressing()) {
      toggle =1;
      pto.set(true);
      endgame.set(true);
    }
    
    if(toggle ==0) {
    l1.setBrake(coast);
    r3.setBrake(coast);
    l1.spin(forward,Controller1.Axis3.position()-Controller1.Axis1.position(),pct);
    l2.spin(forward,Controller1.Axis3.position()+Controller1.Axis1.position(),pct);
    l3.spin(forward,Controller1.Axis3.position()+Controller1.Axis1.position(),pct);
    r1.spin(forward,Controller1.Axis3.position()-Controller1.Axis1.position(),pct);
    r2.spin(forward,Controller1.Axis3.position()-Controller1.Axis1.position(),pct);
    r3.spin(forward,Controller1.Axis3.position()+Controller1.Axis1.position(),pct);
    // sl(Controller1.Axis3.position()+Controller1.Axis1.position());
    // sr(Controller1.Axis3.position()-Controller1.Axis1.position());
    
    }
    else if (toggle == 1) {
      if(Controller1.ButtonR1.pressing()&&!Controller1.ButtonL1.pressing()) {
      l1.spin(forward,100,pct);
      r3.spin(forward,100,pct);
      }
      else if(Controller1.ButtonL1.pressing()&&!Controller1.ButtonR1.pressing()) {
      l1.spin(reverse,60,pct);
      r3.spin(reverse,60,pct);
      }
      else{
        r3.stop(hold);
        l1.stop(hold);
      }
      
      l2.spin(forward,Controller1.Axis3.position()+Controller1.Axis1.position(),pct);
    l3.spin(forward,Controller1.Axis3.position()+Controller1.Axis1.position(),pct);
    r1.spin(forward,Controller1.Axis3.position()-Controller1.Axis1.position(),pct);
    r2.spin(forward,Controller1.Axis3.position()-Controller1.Axis1.position(),pct);
    
    
      
      
      
    }

    if(Controller1.ButtonL1.pressing()&&Controller1.ButtonR1.pressing()&&inta==false) {
        bwingL.set(!bwingL.value());
        bwingR.set(!bwingL.value());
        inta=true;
      }
      else if(!Controller1.ButtonL1.pressing()&&!Controller1.ButtonR1.pressing()){
        inta=false;
      }
    
    if(Controller1.ButtonL1.pressing()&&!Controller1.ButtonR1.pressing()&&toggle==0) {
      Intake.spin(forward,100,pct);
    }
    else if(Controller1.ButtonR1.pressing()&&!Controller1.ButtonL1.pressing()&&toggle==0) {
      Intake.spin(reverse,100,pct);
    }
    else{
      Intake.stop(hold);
    }

    if(Controller1.ButtonL1.pressing()&&Controller1.ButtonR1.pressing()) {
      if(n&&!z) {
        n=false;
      }
      else if(!n&&!z) {
        n=true;
      }
      //arms.set(n);
      z=true;
    }
    if(!Controller1.ButtonL1.pressing()&&!Controller1.ButtonR1.pressing()) {
      z=false;
    }

    

    if(Controller1.ButtonR2.pressing()) {
      
      if(t&&m==false) {
        t=false;
      }
      else if(!t&&m==false) {
        t=true;
      }
      wingL.set(t);
      wingR.set(t);
      m=true;
    }
    if(!Controller1.ButtonR2.pressing()) {
      m=false;
    }

     if(Controller1.ButtonY.pressing()) {
      
      if(l&&lr==false) {
        l=false;
      }
      else if(!l&&lr==false) {
        l=true;
      }
      wingL.set(0);
      wingR.set(l);
      lr=true;
    }
    if(!Controller1.ButtonY.pressing()) {
      lr=false;
    }
    if(Controller1.ButtonA.pressing()) {
      
      if(r&&rr==false) {
        r=false;
      }
      else if(!r&&rr==false) {
        r=true;
      }
      wingL.set(r);
      wingR.set(0);
      rr=true;
    }
    if(!Controller1.ButtonA.pressing()) {
      rr=false;
    }
    
  }
}

void intin(){
  Intake.spin(forward,100,pct);
}
void inout() {
  Intake.spin(reverse,100,pct);
}
int distin() {
  while(li()<200){
    wait(10,msec);
  }
  intout();
  return 0;
}
void intstop() {
  Intake.stop(hold);
}
competition Comp;
int select=1;
void pre() {
  inert.calibrate();
  while(!Comp.isAutonomous()) {
    
  if(Brain.Screen.pressing()) {
    while(Brain.Screen.pressing()){
    
    wait(10,msec);
  }
  select++;
  if(select>4) {
    select=1;
  }

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  if(select == 1) {
    Brain.Screen.print("Defense awp");
  }
  if(select == 2) {
    Brain.Screen.print("Defense Elims");
  }
  if(select == 3) {
    Brain.Screen.print("Offense");
  }
  if(select==4) {
    Brain.Screen.print("skills");
  }
  }
  }
}

void db(int degs){
  double ang= inert.rotation();
  l1.resetPosition();
  l2.resetPosition();
  l3.resetPosition();
  r1.resetPosition();
  r2.resetPosition();
  r3.resetPosition();
  
  while(li()+ri()>-degs*2) {
    float ap = ang-inert.rotation();
    sl(-70+ap*.05);
    sr(-70-ap*.05);
    wait(20,msec);
  }
  sl(0);
  sr(0);
}

void auton1() {
  pidd(-1400,0);
  pidswingl(-50);
  pidd(-500,-45);
  pidswingl(-90);
  
  pid(90);
  pidd(400,90);
  pidd(-500,90);
  pid(25);
  pidd(2200,25);
  pidd(-300,25);
  pid(160);
  pid(65);
  pidd(1100,65);
  pid(180);
  pidd(1500,180);
}

void auton2() {
  pidswingl(35);
  pidd(1950,35);
  pidswingr(90);
  pidd(800,90);
  pidswingl(45);
  pidd(-1800,45);
  pidswingl(-45);
  pidd(-300,-45);
  pidswingl(-90);
  pidswingl(-45);
  pidd(800,-45);
  pidswingl(0);
  pidd(400,0);
  pidswingl(-80);
  pidd(-1600,-80);
}

void auton4() {
  pidswingr(160);
  wait(.5,sec);
  pid(15);
  pidd(700,10);
  pid(0);
  pidd(3000,0);
}

void aut2(){
  pidd(1500,0);
  pid(90);
  inout();
  
  pid(-90);
  intin();
  pidd(1000,-90);
  intstop();
  pid(80);
  inout();
  pidd(1000,80);
  intstop();
  pidd(-1000,80);
  pid(-30);
  intin();
  pidd(500,-30);
  wingR.set(true);
  wingL.set(true);
  pid(90);
}

void autonoffense() {
  intin();
  pidd(80,0);
  pidd(-1600,0);
  
  pidswingl(-45);
  intstop();
  bwingR.set(true);
  pidd(-600,-45);
  bwingR.set(false);
  pidswingl(-90);
  pidd(-600,-90);
  pidd(400,-90);
  pid(90);
  
  inout();
  pidd(300,90);
}

void autond1() {
  thread t(distin);
  pidd(400,0);
  bwingR.set(true);
  pidd(-200,0);
  bwingR.set(false);
  pidswingl(-30);
  pidswingl(-20);
  
  pidd(-600,-25);
  pid(-45);
  pidd(-1500,-45);
}

void auton() {
  pidd(-1600,0,50);
  
  
  pidswingl(-91);
  pidd(-200,-90);
  wingR.set(true);
  pidd(1400,-90);
}

void skills(){
  pidd(-700,0);
  pid(-10);
  pidd(600,-10);
  pid(-90);
  bwingL.set(true);
  cataauto();
  bwingL.set(false);
  pid(-10);
  pidd(1000,-10);
  pid(-80);
  pidd(2500,-80);
  pid(-110);
  pidd(-300,-110);
  pid(-160);
  pidd(-300,-160);
  pidd(300,-160);
  pidd(-300,-60);
  pidd(200,-160);
  pid(-90);
  pidd(1000,-90);
  pid(45);
  wingL.set(true);
  wingR.set(true);
  pidd(500,45);
  pidd(-500,45);
  pidd(500,45);
  wingL.set(false);
  wingR.set(false);
  pid(135);
  pidd(-600,135);
  pid(90);
  wingL.set(true);
  wingR.set(true);
  pidd(400,90);
  pidd(-400,90);
  pidd(400,90);

}


void autonomousprogram() {
  if(select == 1) {
    
    autond1();
  }
  if(select==2) {
    auton();

  }
  if(select==3){
    autonoffense();
  }
  if(select==4){
    skills();
  }
}
int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
   Comp.autonomous(autonomousprogram);
   Comp.drivercontrol(driver);

  pre();
  
  
  
  

  while(1){
    wait(100,msec);
    
  }

}