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
  return (l1.rotation(degrees)+l2.rotation(degrees)+l3.rotation(degrees))/3;
}
int ri(){
  return (r1.rotation(degrees)+r2.rotation(degrees)+r3.rotation(degrees))/3;
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
  float p = ang;
  float i = 0;
  float d = ang;

  float kp = .3;
  float kd=0.05;
  while((fabs(d)>.3||fabs(p)>40)&&fabs(p)>.5) {
    float prev = p;
    p=ang-inert.rotation();
    d=p-prev;

    sl(p*kp+d*kd);
    sr(-p*kp+-d*kd);
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print(inert.rotation());
    wait(10,msec);
  }
  sl(0);
  sr(0);
}

void pidswingr(int ang) {
  float p = ang;
  float i = 0;
  float d = ang;

  float kp = .68;
  float kd=0.05;
  while((fabs(d)>.3||fabs(p)>40)&&fabs(p)>.5) {
    float prev = p;
    p=ang-inert.rotation();
    d=p-prev;

    //sl(p*kp+d*kd);
    sr(-p*kp+-d*kd);
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print(inert.rotation());
    wait(10,msec);
  }
  pid(ang);
  sl(0);
  sr(0);
}

void pidswingl(int ang) {
  float p = ang;
  float i = 0;
  float d = ang;

  float kp = .68;
  float kd=0.05;
  while((fabs(d)>.3||fabs(p)>40)&&fabs(p)>.5) {
    float prev = p;
    p=ang-inert.rotation();
    d=p-prev;

    sl(p*kp+d*kd);
    //sr(-p*kp+-d*kd);
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print(inert.rotation());
    wait(10,msec);
  }
  pid(ang);
  sl(0);
  sr(0);
}

void pidd(int dist, int ang) {
l1.resetRotation();
l2.resetRotation();
l3.resetRotation();
r1.resetRotation();
r2.resetRotation();
r3.resetRotation();
double kp=0.1;
double kd = 0.05;
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
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print(p);
    wait(10,msec);
  }
  sl(0);
  sr(0);
}



int catathing(){
  while(true) {
    
    if(Controller1.ButtonL2.pressing()) {
      cata.spin(forward,100,pct);
    }
    else{
      cata.stop();
    }
  }
  return 0;
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
  while(true) {
    sl(Controller1.Axis3.position()+Controller1.Axis1.position());
    sr(Controller1.Axis3.position()-Controller1.Axis1.position());


    
    if(Controller1.ButtonL1.pressing()&&!Controller1.ButtonR1.pressing()) {
      Intake.spin(forward,100,pct);
    }
    else if(Controller1.ButtonR1.pressing()&&!Controller1.ButtonL1.pressing()) {
      Intake.spin(reverse,100,pct);
    }
    else{
      Intake.stop();
    }

    if(Controller1.ButtonL1.pressing()&&Controller1.ButtonR1.pressing()) {
      if(n&&!z) {
        n=false;
      }
      else if(!n&&!z) {
        n=true;
      }
      arms.set(n);
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


competition Comp;
int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  // Comp.autonomous(auton);
   //Comp.drivercontrol(driver);

  // pre();
  inert.calibrate();
  wait(3,sec);
  pidd(-1500,0);
  pidswingl(-50);
  pidd(-700,-45);
  pidswingl(-90);
  pidd(-400,-90);

  while(1){
    wait(100,msec);
    
  }

}