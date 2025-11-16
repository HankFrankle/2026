#include "vex.h"

using namespace vex;

brain Brain;
competition Competition;

controller Controller1 = controller(primary);
motor intake_1 = motor(PORT2, ratio6_1, true);
motor intake_2 = motor(PORT12, ratio18_1, true);
motor intake_3 = motor(PORT1, ratio18_1, true);

inertial Gyro1 = inertial(PORT13);
digital_out descoreLeft = digital_out(Brain.ThreeWirePort.A);
digital_out descoreRight = digital_out(Brain.ThreeWirePort.C);

digital_out tongue = digital_out(Brain.ThreeWirePort.H);


motor backLeft = motor(PORT18, ratio6_1, true);
motor midLeft = motor(PORT17, ratio6_1, true);
motor frontRight = motor(PORT15, ratio6_1);
motor backRight = motor(PORT20, ratio6_1);
motor midRight = motor(PORT19, ratio6_1);
motor frontLeft = motor(PORT16, ratio6_1, true); 

motor_group leftDrive = motor_group(backLeft, midLeft, frontLeft);
motor_group rightDrive = motor_group(backRight, midRight, frontRight);

motor_group intake = motor_group(intake_1, intake_2, intake_3);

int auton = 4;

#define cs Controller1.Screen
#define Button Controller1.Button

struct Values 
{
  float v1;
  float v2;
  float v3;
};


// This is essentially a parent PID function that we can use for all motor control shit, and we'll just have to tune the  kP, kI, and kD or each thing

Values universalPID(float target, float current, float kp, float ki, float kd, float prevErr, float integral_accum, float interval) {

  float err = target - current;
  float integral = integral_accum + (err * interval);
  float derivative = (err - prevErr) / interval;
  float control = kp * err + ki * integral + kd * derivative;

  return {control, err, integral};
}


void pre_auton(void) {

  while (1) {

    if(Controller1.ButtonB.pressing()) {
      auton += 1;
      waitUntil(!Controller1.ButtonB.pressing());
    }

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(2,1);

    switch (auton) {
      case 1 :
        cs.print("longTube");
        break;
      case 2 :
        cs.print("middleTop");
        break;
      case 3 :
        cs.print("middleBottom");
        break;
      case 4 :
        cs.print("SOLOAWP");
        break;
      case 5 :
        cs.print("4BALL");
        break;
      case 6 :
        cs.print("SKILLS");
        break;
    
      }
  }
}


void pid(int dir) {
  leftDrive.setStopping(brake);
  rightDrive.setStopping(brake);

  float kp = 0.8f;   // This is what we have to tune for an actual PID
  float ki = 0.0f;
  float kd = 0.02f;

  float p = 0.0f;
  float i = 0.0f;
  float d = 0.0f;

  float PIDFinal = 0.0f;

  float iErr = 0.0f;
  float pastErr = 0.0f;
  float Err = 0.0f;

  Err = (float)dir - (float)Gyro1.rotation(degrees);

  const float dt = 0.02f; 
  const float integralLimit = 1000.0f;

  while (fabsf(Err) > 2.0f) {     
    pastErr = Err;
    Err = (float)dir - (float)Gyro1.rotation(degrees);
    iErr += Err * dt;
    
    if (iErr > integralLimit) iErr = integralLimit;
    if (iErr < -integralLimit) iErr = -integralLimit;

    p = Err * kp;
    i = iErr * ki;
    d = ((Err - pastErr) / dt) * kd;

    PIDFinal = p + i + d;

    if (PIDFinal > 80.0f) PIDFinal = 80.0f;
    if (PIDFinal < -80.0f) PIDFinal = -80.0f;

    leftDrive.setVelocity(-PIDFinal, percent);
    rightDrive.setVelocity(PIDFinal, percent);

    leftDrive.spin(forward);
    rightDrive.spin(forward);

    wait(dt * 1000, msec);
  }

  leftDrive.stop(brake);
  rightDrive.stop(brake);
  wait(75, msec);
}


void PIDturn(int target) {

  (void)target;
}

#define NGw ((float)48)
#define NGm ((float)36)
#define circ ((float)10.61)

#define driveWidth ((float)14)

#define wheelRatio ((float)(NGw / NGm) * (360 / circ))
#define gyro (int)(round(Gyro1.rotation(degrees)))
#define posn ((rightDrive.position(degrees) + leftDrive.position(degrees)) / 2) 
#define driveGain ((float)0.55)
#define turnGain ((float)0.415)
#define angleError (-(dir - gyro))

#define LP ( Kd * (turnRad + (driveWidth / 2)) / driveWidth)
#define RP ( Kd * (turnRad - (driveWidth / 2)) / driveWidth)

#define Lturn (turnSpeed * (turnRad + (driveWidth / 2)) / driveWidth)
#define Rturn (turnSpeed * (turnRad - (driveWidth / 2)) / driveWidth)

#define Kd (turnGain * angleError)

void reset_posn(void) {
  rightDrive.setPosition(0, degrees);
  leftDrive.setPosition(0, degrees);
}

void hawk(int time) {
  intake.setVelocity(100, percent);
  intake.spinFor(time, sec);
}

void tuah(int time) {
  intake.setVelocity(-100, percent);
  intake.spinFor(time, sec);
}

void setVel(int vel) {

  leftDrive.setVelocity(-vel, percent);
  rightDrive.setVelocity(vel, percent);
}


void go(float dir, float dist, int speed, float turnSpeed = 20, float turnRad = 0) {
  leftDrive.setVelocity(0, percent);
  rightDrive.setVelocity(0, percent);
  leftDrive.spin(forward);
  rightDrive.spin(forward);

  if (dir < gyro) {
    turnRad = 0 - turnRad;
  }

 
  if (abs(angleError) > 5) {
    if (abs(angleError) > 4) {

      leftDrive.setStopping(brake);
      rightDrive.setStopping(brake);

      float error = angleError;
      float motorPower = 0;

   
      while (fabs(error) > 1.0f) {
        error = angleError; 
        motorPower = error * turnGain;  

        if (motorPower > 40) motorPower = 40;
        if (motorPower < -40) motorPower = -40;

        if (fabs(motorPower) < 1) motorPower = 1;

        
        float leftPower  = - motorPower;
        float rightPower = motorPower;

        leftDrive.setVelocity(leftPower, percent);
        rightDrive.setVelocity(rightPower, percent);

        leftDrive.spin(forward);
        rightDrive.spin(forward);

        wait(15, msec);
      }


      leftDrive.stop(brake);
      rightDrive.stop(brake);
      wait(75, msec); 
    }
  }


  if (dist == 0) return;

  wait(100, msec);
  dist *= wheelRatio;
  reset_posn();

  if (dist > 0) {
    while (posn < dist) {

      rightDrive.setVelocity((float)speed + angleError * driveGain, percent);
      leftDrive.setVelocity((float)speed - angleError * driveGain, percent);

      leftDrive.spin(forward);
      rightDrive.spin(forward);

      wait(10, msec);
    }
    leftDrive.setVelocity(0, percent);
    rightDrive.setVelocity(0, percent);
  } else {
    while (posn > dist) {
      rightDrive.setVelocity(-(float)speed, percent);
      leftDrive.setVelocity(-(float)speed, percent);

      leftDrive.spin(forward);
      rightDrive.spin(forward);
      wait(10, msec);
    }
    leftDrive.setVelocity(0, percent);
    rightDrive.setVelocity(0, percent);
  }

  leftDrive.stop();
  rightDrive.stop();
}





bool gateState = 0;
bool tongueState = 0;

void gate(void) {
  gateState = !gateState;
  descoreLeft.set(gateState);
  descoreRight.set(gateState);
}

void lick(void) {
  tongueState = !tongueState;
  tongue.set(tongueState);
}

void take() {
  intake.setVelocity(100, percent);
  intake.spin(forward);
}

void outake() {
  intake.setVelocity(-100, percent);
  intake.spin(forward);
}

void untake() {
  intake.stop();
}

void autonomous(void) {
  leftDrive.setStopping(brake);
  rightDrive.setStopping(brake);

  switch (auton) {
    case 1 :
      go(0, 25.5, 65);
      wait(100, msec);
      go(-90, 2, 50);
      hawk(2.5);
      go(-90, -27, 60);
      wait(100, msec);
      hawk(1);
      break;

    case 2 :
      gate();
      go(0, 26.5, 60);
      go(90, 0, 0);
      lick();
      take();
      wait(300, msec);
      go(90, 8, 20);
      wait(450, msec);
      go(90, -19, 65);
      wait(100, msec);
      go(90, -5, 15);
      gate();
      wait(100, msec);
      go(90, 0.75, 10);
      wait(1, sec);
      untake();
      lick();
      go(90, 15.5, 40);
      take();
      gate();
      go(225, 35, 70);
      lick();
      go(45, -17, 30);
      wait(100, msec);
      untake();
      outake();
      
    break;
    case 3 :
      gate();
      go(0, 26.5, 60);
      go(90, 0, 0);
      lick();
      take();
      wait(300, msec);
      go(90, 8, 20);
      wait(450, msec);
      go(90, -19, 65);
      wait(100, msec);
      go(90, -4.5, 15);
      gate();
      wait(100, msec);
      go(90, 1, 10);
      go(90, -0.5, 10);
      wait(1.1, sec);
      untake();
      lick();
      go(90, 16, 40);
      take();
      gate();
      go(225, 31, 70);
      go(227, 20, 30);
      wait(100, msec);
      untake();
      outake();
      break;

    case 4 :
      gate();
      go(0, 26.5, 60);
      go(90, 0, 0);
      lick();
      take();
      wait(300, msec);
      go(90, 8.5, 20);
      wait(450, msec);
      go(90, -19, 65);
      wait(100, msec);
      go(90, -4.5, 15);
      gate();
      wait(100, msec);
      go(90, 1, 10);
      go(90, -0.5, 10);
      lick();
      wait(1.1, sec);
      untake();
      go(90, 16, 40);
      take();
      gate();
      go(226, 31, 70);
      go(226, 20, 30);
      wait(100, msec);
      untake();
      outake();
      wait(4, sec);
      gate();
      take();


      // go(227, -13, 50);
      // take();
      // wait(250, msec);
      // go(180, 42, 80);
      // lick();
      // go(140, -15, 60);
      // intake_3.setVelocity(-100, percent);
      // gate();
      
    break;
    case 5 :
      gate();
      take();
      go(0, 26, 40);
      wait(250, msec);
      go(-125, -20, 30);
      intake_3.setVelocity(-100, percent);
      gate();
      break;

    case 6 :
      gate();
      go(0, 26.5, 60);
      go(90, 0, 0);
      lick();
      take();
      wait(300, msec);
      go(90, 8.5, 20);
      wait(650, msec);
      go(90, -19, 65);
      wait(100, msec);
      go(90, -4.5, 15);
      gate();
      wait(100, msec);
      go(90, 1, 10);
      go(90, -0.5, 10);
      lick();
      wait(2, sec);
      untake();
      go(90, 16, 40);
      go(135, 30, 40);
      go(175, 0, 0);
      wait(250, msec);
      take();
      go(170, 35, 100);


      break;
  }
}

void usercontrol(void) {
  int intake1 = 0;
  int intake2 = 0;
  int intake3 = 0;

  bool tonguePis = 0;
  bool leftPis = 0;
  bool rightPis = 0;

  intake_1.spin(forward);
  intake_2.spin(forward);
  intake_3.spin(forward);

  leftDrive.setStopping(coast);
  rightDrive.setStopping(coast);

  while (1) {
    if(Controller1.ButtonR1.pressing() || Controller1.ButtonR2.pressing() || Controller1.ButtonL1.pressing()) {
      intake1 = 100;
      intake2 = 100;
    } else {
      intake1 = 0;
      intake2 = 0;
    }

    if(Controller1.ButtonR1.pressing() || Controller1.ButtonR2.pressing() || Controller1.ButtonUp.pressing()) {
      intake3 = 100;
    } else if(Controller1.ButtonL1.pressing() || Controller1.ButtonL2.pressing()) {
      intake3 = -100;
    } else {
      intake3 = 0;
    } 

    if(Controller1.ButtonL2.pressing()) {
      intake1 = -100;
      intake2 = -100;
    }

    if(Controller1.ButtonA.pressing()) {
      leftPis = !leftPis;
      rightPis = !rightPis;
      waitUntil(!Controller1.ButtonA.pressing());
    }

    if(Controller1.ButtonDown.pressing()) {
      tonguePis = !tonguePis;
      waitUntil(!Controller1.ButtonDown.pressing());
    }

    descoreLeft.set(leftPis);
    descoreRight.set(rightPis);
    tongue.set(tonguePis);

    intake_1.setVelocity(intake1, percent);
    intake_2.setVelocity(intake2, percent);
    intake_3.setVelocity(intake3, percent);

    leftDrive.spin(forward);
    rightDrive.spin(forward);

    leftDrive.setVelocity(Controller1.Axis3.value(), percent);
    rightDrive.setVelocity(Controller1.Axis2.value(), percent);

    wait(5, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) {
    wait(20, msec);
  }
}