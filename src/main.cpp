#include "vex.h"

using namespace vex;

brain Brain;
competition Competition;

controller Controller1 = controller(primary);
motor intake_1 = motor(PORT17, ratio6_1);
motor intake_2 = motor(PORT20, ratio6_1, true);

inertial Gyro1 = inertial(PORT1);
digital_out dih = digital_out(Brain.ThreeWirePort.C);
digital_out foreskin = digital_out(Brain.ThreeWirePort.B);
digital_out tongue = digital_out(Brain.ThreeWirePort.A);


motor frontLeft = motor(PORT14, ratio6_1); 
motor midLeft = motor(PORT13, ratio6_1, true);
motor backLeft = motor(PORT12, ratio6_1, true);
motor frontRight = motor(PORT16, ratio6_1, true);
motor midRight = motor(PORT15, ratio6_1);
motor backRight = motor(PORT11, ratio6_1);

motor_group leftDrive = motor_group(backLeft, midLeft, frontLeft);
motor_group rightDrive = motor_group(backRight, midRight, frontRight);

motor_group intake = motor_group(intake_1, intake_2);

int auton = 1;

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

float mid(float f1, float f2, float f3) {

  return (f1 > f2 && f1 < f3) || (f1 < f2 && f1 > f3) ? f1 :
         (f2 > f1 && f2 < f3) || (f2 < f1 && f1 > f3) ? f2 :
         f3;

}


float low(float f1, float f2) {

  return f1 < f2 ? f1 : f2;

}

float high(float f1, float f2) {

  return f1 > f2 ? f1 : f2;

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
        cs.print("SOLOAWP");
        break;
      case 2 :
        cs.print("RIGHT9");
        break;
      case 3 :
        cs.print("LEFT9");
        break;
      case 4 :
        cs.print("RIGHT4-3");
        break;
      case 5 :
        cs.print("LEFT4-4");
        break;
      case 6 :
        cs.print("SKILLS");
        break;
    
      }

    if (Controller1.ButtonY.pressing()) {
      return;
    }

    wait(20, msec);
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
#define turnGain ((float)0.4)
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



void arcRight(float dir, float turnRad, float speed = 100) {
  float error = dir - gyro;
  float base  = (speed / 100) * error * turnGain;

  leftDrive.spin(forward);
  rightDrive.spin(forward);

  while (abs(error) > 1) {

  error = dir - gyro;
  base  = error * turnGain;


  base = mid(base, 50, (turnRad / 5) *15);

  float leftPower = base * ((turnRad + (driveWidth/2)) / turnRad);
  float rightPower = base * ((turnRad - (driveWidth/2)) / turnRad);


  leftDrive.setVelocity(leftPower, percent);
  rightDrive.setVelocity(rightPower, percent);

  }

  leftDrive.setStopping(brake);
  rightDrive.setStopping(brake);

  
  leftDrive.setVelocity(0, percent);
  rightDrive.setVelocity(0, percent);


}


void arcLeft(float dir, float turnRad, float speed = 100) {
  float error = dir - gyro;
  float base  = (speed / 100 ) * error * turnGain;

  leftDrive.spin(forward);
  rightDrive.spin(forward);

  while (abs(error) > 1) {

  error = dir - gyro;
  base  = error * turnGain;

  base = mid(base, 50, (turnRad / 5) * 15);

  float rightPower = base * ((turnRad + (driveWidth/2)) / turnRad);
  float leftPower = base * ((turnRad - (driveWidth/2)) / turnRad);


  leftDrive.setVelocity(leftPower, percent);
  rightDrive.setVelocity(rightPower, percent);

  }

  leftDrive.setStopping(brake);
  rightDrive.setStopping(brake);

  
  leftDrive.setVelocity(0, percent);
  rightDrive.setVelocity(0, percent);


}





void go(float dir, float dist, int speed, float turnSpeed = 1, float turnRad = 0) {
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

        
        float leftPower  = - turnSpeed * motorPower;
        float rightPower = turnSpeed * motorPower;

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
  // gateState = !gateState;
  //dih.set(gateState);
}

void lick(void) {
  tongueState = !tongueState;
  tongue.set(tongueState);
}

void take() {
  intake.setVelocity(100, percent);
  intake.spin(forward);
}

void outake(float sped = 95) {
  intake.setVelocity(-sped, percent);
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


      go(90, 0, 0);


      // gate();
      // go(0, 26.5, 60);
      // go(90, 0, 0);
      // lick();
      // take();
      // wait(100, msec);
      // go(90, 8, 20);
      // wait(400, msec);
      // go(90, -19.5, 65);
      // wait(150, msec);
      // go(90, -2.5, 10);
      // gate();
      // wait(100, msec);
      // go(90, -1, 10);
      // wait(1.1, sec);
      // untake();
      // lick();
      // gate();
      // arcRight(228, 8);
      // take();
      // go(225, 29, 40);
      // outake();
      // wait(1, sec);
      // untake();
      // go(226, -8, 50);
      // take();
      // wait(100, msec);
      // go(180.5, 41, 60);
      // lick();
      // wait(50,msec);
      // go(136, -14, 60);
      // wait(100, msec);
      // gate();
      
      break;

    case 2 :



        
    gate();
    take();
    go(0, 13, 40);
    arcLeft(-140, 13);
    go(-140, -20, 40);
    go(-140, 1, 20);

    gate(); 

    // take();
    // gate();
    // go(0, 13, 40);
    // arcRight(140, 13);
    // go(140, 17, 40);
    // go(180, -4, 60);
    // wait(150, msec);
    // go(180, -2, 15);
    // gate();
    // wait(100, msec);
    // go(180, -1, 10);
    // wait(1.1, sec);
    // untake();
    // lick();
    // take();
    // gate();
    // go(180, 18, 70);
    // wait(50, msec);
    // go(180, 5, 30);
    // wait(475, msec);
    // go(181, -19.5, 45);
    // wait(150, msec);
    // go(181, -2.5, 15);
    // gate(); 
    // wait(100, msec);
    // go(181, -1.1, 10);
    // wait(1, sec);
    // untake();
    // lick();
    // gate();



    // go(180, 8, 60);
    // go(220, -8, 70);
    // go(165, -5, 30);
    // gate();
    // go(165, -15, 30);

      
    break;
    case 3 :


    gate();
    take();
    go(0, 13, 40);
    arcLeft(-140, 13);
    go(-140, 19, 40);
    go(-180, -5.5, 65);
    wait(150, msec);
    go(-180, -2, 10);
    gate();
    wait(100, msec);
    go(-180, -1.5, 10);
    wait(1.1, sec);
    untake();
    go(-180, 14, 60);
    lick();
    take();
    go(-180, 10, 12);
    gate();
    wait(400, msec);
    go(-180, -19.5, 65);
    wait(150, msec);
    go(-180, -2.5, 10);
    gate();
    wait(100, msec);
    go(-180, -1, 10);
    wait(1.1, sec);
    untake();
    go(-180, 2, 30);

 


      break;

    case 4 :
      
      gate();
      go(0, 26.5, 60);
      go(-90, 0, 0);
      lick();
      take();
      wait(100, msec);
      go(-90, 8, 20);
      wait(430, msec);
      go(-90, -19.5, 65);
      wait(150, msec);
      go(-90, -3,5, 15);
      gate();
      wait(100, msec);
      go(-90, -1, 10);
      wait(1.1, sec);
      untake();
      lick();
      gate();
      arcRight(228, 8);
      take();
      go(-225, 20, 40);
      wait(150, msec);
      go(-45, -7.5, 30);
      wait(1, sec);
      untake();
      go(-45, 7.5, 50);
      take();
      wait(100, msec);
      go(0, 40, 60);
      lick();
      wait(50,msec);
      go(46, 15, 60);
      wait(100, msec);
      outake();
      
    break;
    case 5 :
      gate();
    
      break;

    case 6 :
      gate();
      go(0, 26.5, 60);
      go(90, 0, 0);
      lick();
      take();
      wait(100, msec);
      go(90, 8, 20);
      wait(650, msec);
      go(90, -10, 65);
      go(135, -10, 30);
      go(90, -50, 50);
      go(45, -10, 30);
      go(-90, -10, 30);
      wait(100, msec);
      go(-90, -3,5, 15);
      gate();
      wait(100, msec);
      go(-90, -1, 10);
      wait(1.1, sec);
      untake();
      go(-90, 25, 40);
      wait(650, msec);
      go(-90, -22, 30);
      wait(100, msec);
      go(-90, -3,5, 15);
      gate();
      wait(100, msec);
      go(-90, -1, 10);
      wait(1.1, sec);
      untake();
      lick();
      go(-90, 10, 40);
      go(-180, 80, 60);
      lick();
      go(-90, 15, 40);
      wait(650, msec);
      go(-90, -10, 30);
      wait(100, msec);
      go(-45, -10, 30);
      go(-90, 50, 40);
      go(-135, -10, 30);
      go(-270, -8, 30);
      wait(100, msec);
      go(-270, -3,5, 15);
      gate();
      wait(100, msec);
      go(-270, -1, 10);
      wait(1.1, sec);
      untake();
      go(-270, 25, 40);
      wait(650, msec);
      go(-270, -22, 30);
      wait(100, msec);
      go(-270, -3,5, 15);
      gate();
      wait(100, msec);
      go(-270, -1, 10);
      wait(1.1, sec);
      untake();
      lick();
      go(-270, 10, 30);
      go(-315, 20, 35);
      go(-360, 0, 0);
      lick();
      wait(100, msec);
      take();
      go(-360, 30, 30);

      break;
        case 7 :


      gate();
      go(0, 26.5, 60);
      go(90, 0, 0);
      lick();
      take();
      wait(100, msec);
      go(90, 8, 20);
      wait(1000, msec);
      go(90, -1, 20);
      wait(100, msec);
      go(90, 1, 20);
      wait(100, msec);
      go(90, -19.5, 65);
      wait(150, msec);
      go(90, -2.5, 10);
      gate();
      wait(100, msec);
      go(90, -1, 10);
      wait(3, sec);
      outake();
      wait(250, msec);
      take();
      wait(2, sec);
      untake();
      lick();
      gate();
      arcRight(228, 8);
      take();
      go(225, 29, 40);
      outake();
      wait(1, sec);
      untake();
      go(226, -8, 50);
      take();
      wait(100, msec);
      go(180.5, 41, 60);
      lick();
      wait(50,msec);
      go(136, -14, 60);
      wait(4000, msec);
      gate();
      lick();
      go(136, 15, 60);
      go(90, 40, 60);
      go(5, 40, 50);
      go(5, 2, 60);
      go(5, -4, 60);
      go(5, 4, 60);


      break;

    case 8:
      take();
      go(0, 15, 60);
      go(-45, 30, 60);
      go(-125, 40, 60);
      go(-125, 2, 50);
      go(-125, -4, 50);
      go(-125, 4, 50);




  }
}

  bool tonguePis = 0;
  bool dihState = 0;
  bool foreskinState = 1;





void afterMiddle (void) {
  intake_1.spin(forward);
  intake_2.spin(forward);
  dihState = 1;
  foreskinState = 1;
}

void intakeWithGate (void) {
  intake_1.spin(forward);
  intake_2.spin(forward);
  intake_1.setVelocity(100, percent);
  intake_2.setVelocity(100, percent);
  dihState = 1;
  foreskinState = 1;
}

void scoreHigh (void) {
  

  intake_1.spin(forward);
  intake_2.spin(forward);
  intake.setVelocity(100, percent);
  dihState = 0;
  foreskinState = 1;
}

void scoreMid (void) {
  intake_1.spin(forward);
  intake_2.spin(forward);
  intake.setVelocity(100, percent);
  dihState = 0;
  foreskinState = 0;
}

void outtake(void) {
  intake_1.spin(forward);
  intake_2.spin(forward);
  intake.setVelocity(-100, percent);
  dihState = 0;
}

void intakeStop(void) {
  intake.setStopping(brake);
  intake.setVelocity(0, percent);
}



void left_drive(void) {
  leftDrive.spin(forward);
  leftDrive.setVelocity(Controller1.Axis3.value(), percent);
}

void right_drive(void) {
  rightDrive.spin(forward);
  rightDrive.setVelocity(Controller1.Axis2.value(), percent);
}




void toggleHood (void) {
  dihState = !dihState;
  dih.set(dihState);
}







void usercontrol(void) {
  int intake1 = 0;
  int intake2 = 0;
  int intake3 = 0;

 



  leftDrive.setStopping(coast);
  rightDrive.setStopping(coast);

  // while (1) {
  //   if(Controller1.ButtonR1.pressing() || Controller1.ButtonR2.pressing() || Controller1.ButtonL1.pressing()) {
  //     intake1 = 100;
  //     intake2 = 100;
  //   } else {
  //     intake1 = 0;
  //     intake2 = 0;
  //   }

  //   if(Controller1.ButtonR1.pressing() || Controller1.ButtonR2.pressing() || Controller1.ButtonUp.pressing()) {
  //     intake3 = 100;
  //   } else if(Controller1.ButtonL1.pressing() || Controller1.ButtonL2.pressing()) {
  //     intake3 = -100;
  //   } else {
  //     intake3 = 0;
  //   } 

  //   if(Controller1.ButtonL2.pressing()) {
  //     intake1 = -100;
  //     intake2 = -100;
  //   }




    if (Controller1.ButtonL1.pressing() || Controller1.ButtonR1.pressing()) {
      intake.setVelocity(100, percent);
    }else if (Controller1.ButtonL2.pressing()) {
      intake.setVelocity(-100, percent);
    } else {
      intake.setVelocity(0, percent);
    }
    

    if (Controller1.ButtonL1.pressing()) {



    }






    

    if(Controller1.ButtonDown.pressing()) {
      tonguePis = !tonguePis;
      waitUntil(!Controller1.ButtonDown.pressing());
    }

    dih.set(dihState);

    tongue.set(tonguePis);

    wait(5, msec);
  }


  
  int main() {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);
    pre_auton();
    Controller1.ButtonR1.pressed(intakeWithGate);
    Controller1.ButtonR1.released(intakeStop);
    Controller1.ButtonR2.pressed(scoreHigh);
    Controller1.ButtonR2.released(intakeStop);
    Controller1.ButtonL1.pressed(scoreMid);
    Controller1.ButtonL1.released(afterMiddle);
    Controller1.ButtonL1.released(intakeStop);
    Controller1.ButtonL2.pressed(outtake);
    Controller1.ButtonL2.released(intakeStop);
    Controller1.Axis3.changed(left_drive);
    Controller1.Axis2.changed(right_drive);
    Controller1.ButtonA.pressed(toggleHood);

  while (true) {
    wait(20, msec);
  }
}