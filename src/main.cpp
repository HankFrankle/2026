#include "vex.h"

using namespace vex;

brain Brain;
competition Competition;

controller Controller1 = controller(primary);
motor intake_1 = motor(PORT17, ratio6_1);
motor intake_2 = motor(PORT20, ratio6_1, true);

inertial Gyro1 = inertial(PORT8);
rotation odom = rotation(PORT9, true);

digital_out dih = digital_out(Brain.ThreeWirePort.H);
digital_out lift = digital_out(Brain.ThreeWirePort.B);
digital_out tongue = digital_out(Brain.ThreeWirePort.A);
digital_out intakeL = digital_out(Brain.ThreeWirePort.F);


motor frontLeft = motor(PORT14, ratio6_1); 
motor midLeft = motor(PORT13, ratio6_1, true);
motor backLeft = motor(PORT12, ratio6_1, true);
motor frontRight = motor(PORT16, ratio6_1, true);
motor midRight = motor(PORT15, ratio6_1);
motor backRight = motor(PORT11, ratio6_1);

motor_group leftDrive = motor_group(backLeft, midLeft, frontLeft);
motor_group rightDrive = motor_group(backRight, midRight, frontRight);

motor_group intake = motor_group(intake_1, intake_2);

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
        cs.print("SOLOAWP(13)");
        break;
      case 2 :
        cs.print("COUNTERAWP(10-13)");
        break;
      case 3 :
        cs.print("LEFT3-4(7)");
        break;
      case 4 :
        cs.print("RIGHT3-4(7)");
        break;
      case 5 :
        cs.print("4BALLFINGER(4)");
        break;
      case 6 :
        cs.print("SKILLS(75)");
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

class uniPID {
private:
  double kp;
  double ki;
  double kd;
  double prevErr;
  double integ;
  double interval;
  double integ_limit;
  double output_clamp;

public :
  uniPID(double p, double i, double d, double wait_time, double integ_limit = 1000.0, double clamp = 100.0)
    : kp(p), ki(i), kd(d), interval(wait_time), integ_limit(wait_time), output_clamp(clamp), prevErr(0), integ(0) {}

  double calculate(double target, double current) {
    double err = target - current;

    integ += err * integ;
    integ = mid(-integ_limit, integ, integ_limit);

    double deriv = (err - prevErr) / interval;

    double control = kp * err + ki * integ + kd * deriv;

    prevErr = err;

    control = mid(-output_clamp, control, output_clamp);

    return control;

  }

  bool atTarget(double tolerance) {
    return fabs(prevErr) < tolerance;
  }

   
  void reset() {
    prevErr = 0;
    integ = 0;
  }

  void setKp(double p) {kp = p;}
  void setKi(double i) {kp = i;}
  void setKd(double d) {kd = d;}
  void setLimits(double int_limit, double out_limit) {
    integ_limit = int_limit;
    output_clamp = out_limit;
  }
  double getErr() { return prevErr; }
  double getInteg() { return integ; }



};



void FullMid(void) {
  dih.set(1);
  lift.set(0);

  intake.setVelocity(60, percent);
  intake.spin(forward);
  wait(5, sec);
  intake.stop();


  leftDrive.setVelocity(10, percent);
  rightDrive.setVelocity(10, percent);

  leftDrive.spinFor(75, degrees, false);
  rightDrive.spinFor(75, degrees);
  wait(100, msec);
  leftDrive.spinFor(-50, degrees, false);
  rightDrive.spinFor(-50, degrees);



}

void PIDturn(int target) {

  (void)target;
}

  #define NGw ((float)48)
  #define NGm ((float)36)
  #define circ ((float)10.21)

  #define driveWidth ((float)14)

  #define wheelRatio ((float)(NGw / NGm) * (360 / circ))
  #define gyro (int)(round(Gyro1.rotation(degrees)))
  #define posn ((rightDrive.position(degrees) + leftDrive.position(degrees)) / 2) 
  #define driveGain ((float)0.3)
  #define turnGain ((float)0.41)
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



void arcRight(float dir, float turnRad, float clamp = 50, float speed = 100) {
  float error = dir - gyro;
  float base  = (speed / 100) * error * turnGain;

  leftDrive.spin(forward);
  rightDrive.spin(forward);

  while (abs(error) > 1) {

  error = dir - gyro;
  base  = error * turnGain;


  base = mid(base, clamp, (turnRad / 5) *15);

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


void arcLeft(float dir, float turnRad, int clamp = 50, float speed = 100) {
  float error = dir - gyro;
  float base  = (speed / 100 ) * error * turnGain;

  leftDrive.spin(forward);
  rightDrive.spin(forward);

  while (abs(error) > 1) {

  error = dir - gyro;
  base  = error * turnGain;

  base = mid(base, clamp, (turnRad / 5) * 15);

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



void pidTurn(double target) {
  uniPID turn(0.45, 0.05, 0.3, 0.02, 50, 40);

  while(!turn.atTarget(1.5)) {
    double angle = Gyro1.rotation(degrees);
    double power = turn.calculate(target, angle);

    leftDrive.setVelocity(power, percent);
    rightDrive.setVelocity(-power, percent);

    wait(10, msec);

  }

  leftDrive.stop();
  rightDrive.stop();
}


void go(float dir, float dist, int clamp = 60, int timeO = 150, float turnClamp = 40) {

   leftDrive.setVelocity(0, percent);
  rightDrive.setVelocity(0, percent);
  leftDrive.spin(forward);
  rightDrive.spin(forward);

  

  if (abs(angleError) > 5) {

    int timeoutCount = 0;

    if (abs(angleError) > 4 && timeoutCount < timeO) {

      leftDrive.setStopping(brake);
      rightDrive.setStopping(brake);

      float error = angleError;
      float motorPower = 0;

  
      while (fabs(error) > 1.0f) {
        error = angleError; 
        motorPower = error * turnGain;  

        motorPower = mid(-turnClamp, motorPower, turnClamp);

        
        float leftPower  = -motorPower;
        float rightPower = motorPower;

        leftDrive.setVelocity(leftPower, percent);
        rightDrive.setVelocity(rightPower, percent);

        leftDrive.spin(forward);
        rightDrive.spin(forward);

        timeoutCount += 1;
        wait(15, msec);
      }


      leftDrive.stop(brake);
      rightDrive.stop(brake);
      wait(75, msec); 
    }
  }




  if (dist == 0) return;

  

  leftDrive.setStopping(brake);
  rightDrive.setStopping(brake);

  wait(100, msec);
  dist *= wheelRatio;
  reset_posn();
  odom.setPosition(0, rev); 
  odom.setPosition(0, deg); 

  wait(100, msec);

  double trvld = 0.0;

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Target: %.2f", dist);

  if (dist > 0) {

    int timeoutCount = 0;

    while (trvld < dist && timeoutCount < timeO) {

      double pot = odom.position(degrees);  
      trvld = pot;
      

      Brain.Screen.setCursor(2, 1);
      Brain.Screen.print("pot: %.2f trvld: %.2f", pot, trvld);

      double driveErr = dist - trvld;
      double driveClamp = clamp;
      double driveSpeed = mid(-driveClamp, (driveErr * driveGain), driveClamp);


      Brain.Screen.setCursor(3, 1);
      Brain.Screen.print("err: %.2f speed: %.2f", driveErr, driveSpeed);

      rightDrive.setVelocity((double)driveSpeed + angleError * 0.3, percent);
      leftDrive.setVelocity((double)driveSpeed - angleError * 0.3, percent);

      leftDrive.spin(forward);
      rightDrive.spin(forward);

      wait(10, msec);



      timeoutCount += 1;

      
    
    }
      leftDrive.setVelocity(0, percent);
      rightDrive.setVelocity(0, percent);
    } else {


      int timeoutCount = 0;
      
      while (trvld > dist && timeoutCount < timeO) {



      double pot = odom.position(degrees);  
      trvld = pot;
      

      Brain.Screen.setCursor(2, 1);
      Brain.Screen.print("pot: %.2f trvld: %.2f", pot, trvld);

      double driveErr = dist - trvld;
      double driveClamp = clamp;
      double driveSpeed = mid(-driveClamp, (driveErr * driveGain), driveClamp);


      Brain.Screen.setCursor(3, 1);
      Brain.Screen.print("err: %.2f speed: %.2f", driveErr, driveSpeed);

      rightDrive.setVelocity((double)driveSpeed + angleError * 0.3, percent);
      leftDrive.setVelocity((double)driveSpeed - angleError * 0.3,  percent);

      leftDrive.spin(forward);
      rightDrive.spin(forward);

      wait(10, msec);
      timeoutCount += 1;

      
      }
      leftDrive.setVelocity(0, percent);
      rightDrive.setVelocity(0, percent);
    
  }


    leftDrive.stop();
    rightDrive.stop();
  }




















bool tongueState = 0;
bool dihState = 0;
bool liftState = 0;
bool intakeLift = 0;



void flick(void) {
  dihState = !dihState;
  dih.set(dihState);
}

void lick(void) {
  tongueState = !tongueState;
  tongue.set(tongueState);
}

void take(int sped = 100) {
  intake.setVelocity(sped, percent);
  intake.spin(forward);
}

void outake(float sped = 95) {
  intake.setVelocity(-sped, percent);
  intake.spin(forward);
}

void untake() {
  intake.stop();
}

void proBono (void) {
  if(lift.value() == 0) {
    lift.set(1);
  } else if(lift.value() == 1) {
    lift.set(0);
  }
}




void autonomous(void) {
  leftDrive.setStopping(brake);
  rightDrive.setStopping(brake);

  switch (auton) {
    case 1 :


proBono();

    proBono();

    flick();

    go(0, 33);

    go(90, 0);

    lick();

    wait(200, msec);

    take();

    go(90, 6, 20);

    leftDrive.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);
    rightDrive.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);
  
    wait(350, msec);

    go(91, -30, 40);

    wait(100, msec);

    flick();
    
    wait(925, msec);

    arcRight(225, 8);
    
    flick();
    
    lick();
    
    take();

    wait(50, msec);

    go(225, 23, 40);

    wait(25, msec);

    lick();

    // wait(100, msec);

    go(180, 0);

    lick();

    go(180, 46, 70);

    lick();

    // go(180, 4)j;

    go(135, -15.5, 50, 150, 50);

    wait(100, msec);

    take(45);

    proBono();

    wait(750, msec);

    proBono();

    go(135, 45, 70);

    take();

    arcLeft(90, 19, 35);

    leftDrive.spin(vex::directionType::fwd, 4, vex::voltageUnits::volt);
    rightDrive.spin(vex::directionType::fwd, 4, vex::voltageUnits::volt);

    wait(350, msec);

    go(90, -28, 80);
    
    flick();

      
      break;
    case 2 :


      
      break;
    case 3 :

    proBono();

    proBono();

    flick();

    go(0, 24);

    take();

    arcLeft(-40, 30, 20);

    lick();

    wait(150, msec);

    go(-135, -16, 50);

    wait(100, msec);

    take(45);

    proBono();

    wait(750, msec);

    proBono();

    go(-133, 49);

    take();

    arcLeft(-180, 17, 15);

    // go(-180, 1, 20);

    // wait(100, msec);

    // go(-180, 0.5, 10);

    leftDrive.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);
    rightDrive.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);

    wait(600, msec);

    go(-178, -30);

    wait(100, msec);
    
    flick();

    wait(900, msec);

    take();

    go(-180, 10, 40 );

    untake();

    go(-130, -18, 35);

    wait(50, msec);

    go(-180, -22, 35);



      break;
    case 4 :


    proBono();

    proBono();

    flick();

    go(0, 24);

    take();

    arcRight(45, 31, 20);

    wait(150, msec);

    go(-45, 14.5, 50);

    wait(100, msec);

    outake();

    wait(800, msec);

    // take();

    go(-47, -52);

    take();

    wait(250, msec);

    go(180, 0);

    lick();

    wait(200, msec);

    // go(-180, 1, 20);

    // wait(100, msec);

    // go(-180, 0.5, 10);

    leftDrive.spin(vex::directionType::fwd, 4, vex::voltageUnits::volt);
    rightDrive.spin(vex::directionType::fwd, 4, vex::voltageUnits::volt);

    wait(800, msec);

    go(180, -30, 40);

    wait(100, msec);
    
    flick();

    wait(900, msec);

    take();

    go(180, 10.5, 40 );

    untake();

    go(220, -19, 35);

    wait(50, msec);

    go(180, -22, 35);
      
      break;
    case 5 :
      
    
      break;

    case 6 :



 
    proBono();

    proBono();

    flick();

    go(0, 33);

    go(90, 0);

    lick();

    wait(200, msec);

    take();

    go(90, 6, 20);

    leftDrive.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);
    rightDrive.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);

    wait(1.25, sec);  

    go(90, -1, 25);

    wait(0.25, sec);

    go(90, 2, 15);

    wait(0.5, sec);

    go(91, -15, 40);

    arcRight(266, 7);

    untake();

    go(270, 80);

    arcRight(310, 19);

    arcLeft(266, 12, 32);

    go(270, -23, 30);

    outake();

    wait(50, msec);

    take();

    flick();

    wait(2.5, sec);

    go(269, 26, 30);

    flick();

    leftDrive.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);
    rightDrive.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);

    wait(850, msec);

    go(270, -1);

    wait(50, msec);

    go(270, 3, 20);

    wait(1.5, sec);

    go(268, -30, 40);

    wait(50, msec);

    outake();

    wait(100, msec);

    take();

    flick();

    wait(2.75, sec);

    go(270, 3, 10);

    wait(100, msec);

    flick();
  
    go(270, -5, 25);

    wait(100, msec);

    go(270, 8);

    wait(100, msec);

    lick();

    go(180, 109, 75, 300);

    wait(250, msec);

    go(270, 0);

    lick();

    wait(200, msec);

    take();

    go(270, 6, 20);

    leftDrive.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);
    rightDrive.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);

    wait(2.25, sec);  

    go(270, -1, 25);

    wait(0.25, sec);

    go(270, 2, 15);

    wait(0.5, sec);

    go(271, -15, 40);

    arcRight(446, 8);

    untake();

    go(450, 80);

    arcRight(490, 19);

    arcLeft(446, 12, 41);

    go(450, -23, 30);

    outake();

    wait(50, msec);

    take();

    flick();

    wait(2.5, sec);

    go(449, 26, 30);

    flick();

    leftDrive.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);
    rightDrive.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);

    wait(1.25, sec);

    go(450, -1);

    wait(50, msec);

    go(450, 3, 20);

    wait(1.5, sec);

    go(450, -30, 40);

    wait(50, msec);

    outake();

    wait(100, msec);

    take();

    flick();

    wait(2.75, sec);

    go(450, 3, 10);

    wait(100, msec);

    flick();
  
    go(450, -5, 25);

    wait(100, msec);

    lick();

    go(450, 10);

    go(580, -40);

    go(550, -30, 40);





      break;
        case 7 :




      break;

    case 8:
      // take();
      // go(0, 15, 60);
      // go(-45, 30, 60);
      // go(-125, 40, 60);
      // go(-125, 2, 50);
      // go(-125, -4, 50);
      // go(-125, 4, 50);


      go(0, -10);


  }
}




void afterMiddle (void) {
  intake_1.spin(forward);
  intake_2.spin(forward);
  dihState = 1;
  liftState = 1;
}

void intakeWithflick (void) {
  intake_1.spin(forward);
  intake_2.spin(forward);
  intake_1.setVelocity(100, percent);
  intake_2.setVelocity(75, percent);
  dihState = 1;
  liftState = 1;
}

void scoreHigh (void) {
  

  intake_1.spin(forward);
  intake_2.spin(forward);
  intake_1.setVelocity(100, percent);
  intake_2.setVelocity(70, percent);
  dihState = 0;
  liftState = 1;
}

void scoreMid (void) {
  intake_1.spin(forward);
  intake_2.spin(forward);
  intake_1.setVelocity(100, percent);
  intake_2.setVelocity(50, percent);
  dihState = 1;
  liftState = 0;
}

void outtake(void) {
  intake_1.spin(forward);
  intake_2.spin(forward);
  intake.setVelocity(-100, percent);
  dihState = 1;
}

void intakeStop(void) {
  intake.setStopping(brake);
  intake.setVelocity(0, percent);
}

void afterHighScore(void) {
  dihState = 1;
  liftState = 1;
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

void toggleIntakeLift(void) {
  intakeLift = !intakeLift;
  intakeL.set(intakeLift);


}

void toggleTongue (void) {
  tongueState = !tongueState;
  tongue.set(tongueState);
}





void usercontrol(void) {
  int intake1 = 0;
  int intake2 = 0;
  int intake3 = 0;

  proBono();

 

  leftDrive.setStopping(coast);
  rightDrive.setStopping(coast);

  while (1) {

    // if(Controller1.ButtonL2.pressing() && Controller1.ButtonR2.pressing()) {
    //   intake_1.spin(forward);
    //   intake_2.spin(forward);
    //   intake.setVelocity(-100, percent);
    //   dihState = 1;
      
    // } else{
    //   intake.stop();
    // }

    dih.set(dihState);
    tongue.set(tongueState);
    lift.set(liftState);

    wait(5, msec);
    }
  }


  
  int main() {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);
    pre_auton();
    Controller1.ButtonR1.pressed(intakeWithflick);
    Controller1.ButtonR1.released(intakeStop);
    Controller1.ButtonR2.pressed(scoreHigh);
    Controller1.ButtonR2.released(intakeStop);
    Controller1.ButtonR2.released(afterHighScore);
    Controller1.ButtonL1.pressed(scoreMid);
    Controller1.ButtonL1.released(afterMiddle);
    Controller1.ButtonL1.released(intakeStop);
    Controller1.ButtonL2.pressed(toggleHood);
    Controller1.ButtonL2.released(intakeStop);
    Controller1.Axis3.changed(left_drive);
    Controller1.Axis2.changed(right_drive);
    Controller1.ButtonA.pressed(toggleIntakeLift);
    Controller1.ButtonUp.pressed(outtake);
    Controller1.ButtonDown.pressed(toggleTongue);
    Controller1.ButtonLeft.pressed(FullMid);

  while (true) {
    wait(20, msec);
  }
  return(0);
}