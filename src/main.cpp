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
  #define turnGain ((float)0.3925)
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




// void go(float dir, float dist, int speed, float turnSpeed = 1, float turnRad = 0) {
//   leftDrive.setVelocity(0, percent);
//   rightDrive.setVelocity(0, percent);
//   leftDrive.spin(forward);
//   rightDrive.spin(forward);

//   if (dir < gyro) {
//     turnRad = 0 - turnRad;
//   }


//   if (abs(angleError) > 5) {
//     if (abs(angleError) > 4) {

//       leftDrive.setStopping(brake);
//       rightDrive.setStopping(brake);

//       float error = angleError;
//       float motorPower = 0;

  
//       while (fabs(error) > 1.0f) {
//         error = angleError; 
//         motorPower = error * turnGain;  

//         if (motorPower > 40) motorPower = 40;
//         if (motorPower < -40) motorPower = -40;

//         if (fabs(motorPower) < 1) motorPower = 1;

        
//         float leftPower  = - turnSpeed * motorPower;
//         float rightPower = turnSpeed * motorPower;

//         leftDrive.setVelocity(leftPower, percent);
//         rightDrive.setVelocity(rightPower, percent);

//         leftDrive.spin(forward);
//         rightDrive.spin(forward);

//         wait(15, msec);
//       }


//       leftDrive.stop(brake);
//       rightDrive.stop(brake);
//       wait(75, msec); 
//     }
//   }


//   if (dist == 0) return;


//   leftDrive.setStopping(brake);
//   rightDrive.setStopping(brake);

//   wait(100, msec);
//   dist *= wheelRatio;
//   reset_posn();
//   odom.resetPosition();


//   double trvld = 0.0;


//   if (dist > 0) {

//     while (trvld < dist) {

//         double pot = odom.value();  

//       double driveErr = dist - trvld;
//       double driveClamp = 75;
//       trvld = pot / wheelRatio;

//       double driveSpeed = mid(-driveClamp, (driveErr * driveGain), driveClamp);



//       rightDrive.setVelocity((double)driveSpeed + angleError, percent);
//       leftDrive.setVelocity((double)driveSpeed - angleError, percent);

//       leftDrive.spin(forward);
//       rightDrive.spin(forward);

//       wait(10, msec);
//     }
//     leftDrive.setVelocity(0, percent);
//     rightDrive.setVelocity(0, percent);
//   } else {
//     while (posn > dist) {
//       rightDrive.setVelocity(-(float)speed, percent);
//       leftDrive.setVelocity(-(float)speed, percent);

//       leftDrive.spin(forward);
//       rightDrive.spin(forward);
//       wait(10, msec);
//     }
//     leftDrive.setVelocity(0, percent);
//     rightDrive.setVelocity(0, percent);
//   }

//   leftDrive.stop();
//   rightDrive.stop();
// }






void go(float dir, float dist, int clamp = 60, float turnSpeed = 1, float turnRad = 0) {

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
    while (trvld < dist) {
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
    }
      leftDrive.setVelocity(0, percent);
      rightDrive.setVelocity(0, percent);
    } else {
      while (trvld > dist) {



      double pot = odom.position(degrees);  
      trvld = pot;
      

      Brain.Screen.setCursor(2, 1);
      Brain.Screen.print("pot: %.2f trvld: %.2f", pot, trvld);

      double driveErr = dist - trvld;
      double driveClamp = clamp;
      double driveSpeed = mid(-driveClamp, (driveErr * driveGain), driveClamp);


      Brain.Screen.setCursor(3, 1);
      Brain.Screen.print("err: %.2f speed: %.2f", driveErr, driveSpeed);

      rightDrive.setVelocity((double)driveSpeed + angleError, percent);
      leftDrive.setVelocity((double)driveSpeed - angleError, percent);

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




















bool tongueState = 0;
bool dihState = 0;
bool liftState = 0;



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

    go(90, 6.5, 20);

    wait(50, msec);

    go(90, -0.5);

    wait(50, msec);

    go(90, 1);
  
    wait(325, msec);

    go(91, -30, 40);

    wait(100, msec);

    flick();
    
    wait(925, msec);

    flick();

    proBono();

    lick();

    arcRight(225, 8);

    proBono();

    take(75);

    go(225, 8);

    arcLeft(150, 9);

    go(180, 52);

















    // go(224, 37, 40);

    // outake(60);

    // wait(750, msec);

    // untake();

    // go(225, -12, 40);

    // wait(100, msec);

    // take();

    // go(180, 52);

    // lick();

    // wait(150, msec);

    // go(135, -18, 30);

    // wait(100, msec);

    // take();

    // proBono();

    // wait(600, msec);

    // untake();

    // go(135, 3);

    // wait(250, msec);

    // go(115, -3, 30);










    // proBono();
    //   proBono();
    //   flick();
    //   go(0, 27.75, 60);
    //   go(90, 0, 0);
    //   lick();
    //   take();
    //   wait(100, msec);
    //   go(90, 8, 20);
    //   wait(400, msec);
    //   go(90, -19, 65);
    //   wait(150, msec);
    //   go(90, -2.5, 10);
    //   flick();
    //   wait(100, msec);
    //   go(90, -1, 10);
    //   wait(1.1, sec);
    //   untake();
    //   lick();
    //   flick();
    //   arcRight(228, 8);
    //   take();
    //   go(225, 30, 40);
    //   outake(50);
    //   wait(1, sec);
    //   untake();
    //   go(226, -10.5, 50);
    //   take();
    //   wait(100, msec);
    //   go(180.5, 46, 60);
    //   lick();
    //   wait(50,msec);
    //   go(134, -11, 60);
    //   wait(250, msec);
    //   proBono();
    //   take();
      
      break;

    case 2 :



        
    flick();
    take();
    go(0, 13, 40);
    arcLeft(-140, 13);
    go(-140, -20, 40);
    go(-140, 1, 20);

    flick(); 

    // take();
    // flick();
    // go(0, 13, 40);
    // arcRight(140, 13);
    // go(140, 17, 40);
    // go(180, -4, 60);
    // wait(150, msec);
    // go(180, -2, 15);
    // flick();
    // wait(100, msec);
    // go(180, -1, 10);
    // wait(1.1, sec);
    // untake();
    // lick();
    // take();
    // flick();
    // go(180, 18, 70);
    // wait(50, msec);
    // go(180, 5, 30);
    // wait(475, msec);
    // go(181, -19.5, 45);
    // wait(150, msec);
    // go(181, -2.5, 15);
    // flick(); 
    // wait(100, msec);
    // go(181, -1.1, 10);
    // wait(1, sec);
    // untake();
    // lick();
    // flick();



    // go(180, 8, 60);
    // go(220, -8, 70);
    // go(165, -5, 30);
    // flick();
    // go(165, -15, 30);

      
    break;
    case 3 :

    proBono();

    proBono();
    
    flick();

    go(0, 34);

    go(90, 0);

    lick();

    wait(150, msec);

    take();

    go(90, 7, 30);

    wait(50, msec);

    go(90, -0.5);

    wait(50, msec);

    go(90, 0.5);
  
    wait(500, msec);

    go(91, -30, 40);

    wait(100, msec);

    flick();
    
    wait(925, msec);

    flick();

    proBono();

    lick();

    arcRight(225, 7.5);

    proBono();

    take(40);

    go(224, 35, 40);

    outake(40);


 


      break;

    case 4 :
      
    proBono();

    proBono();
  
    flick();

    go(0, 31.5);

    go(-90, 0);

    lick();

    take();

    wait(150, msec);

    go(-90, 7, 20);

    wait(50, msec);

    go(-90, -0.5);

    wait(50, msec);

    go(-90, 0.5);
  
    wait(350, msec);

    go(-91, -30, 40);

    wait(100, msec);

    flick();
    
    wait(925, msec);

    flick();

    proBono();

    lick();

    arcLeft(-225, 8.5);

    proBono();

    take(75);

    go(-224, 26, 40);

    lick();

    wait(100, msec);

    go(-44, -8);

    wait(100, msec);

    take();

    proBono();


      
    break;
    case 5 :
      flick();
    
      break;

    case 6 :
      flick();
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
      flick();
      wait(100, msec);
      go(-90, -1, 10);
      wait(1.1, sec);
      untake();
      go(-90, 25, 40);
      wait(650, msec);
      go(-90, -22, 30);
      wait(100, msec);
      go(-90, -3,5, 15);
      flick();
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
      flick();
      wait(100, msec);
      go(-270, -1, 10);
      wait(1.1, sec);
      untake();
      go(-270, 25, 40);
      wait(650, msec);
      go(-270, -22, 30);
      wait(100, msec);
      go(-270, -3,5, 15);
      flick();
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





        take();

      // proBono();

      // proBono();
      
      // flick();
      // go(0, 27.75, 60);
      // go(90, 0, 0);
      // lick();
      // take();
      // wait(100, msec);
      // go(90, 8, 20);
      // wait(50, msec);
      // go(90, -1, 30);
      // wait(50, msec);
      // go(90, 1, 30);
      // wait(50, msec);
      // go(90, -1, 30);
      // wait(50, msec);
      // go(90, 1, 30);
      // wait(200, msec);
      // go(90, -19, 65);
      // wait(150, msec);
      // go(90, -2.5, 10);
      // flick();
      // wait(100, msec);
      // go(90, -1, 10);
      // wait(1.1, sec);
      // untake();
      // lick();
      // flick();
      // arcRight(228, 8);
      // take();
      // go(225, 27.5, 40);
      // outake(35);
      // wait(1, sec);
      // untake();
      // go(226, -8, 50);
      // take();
      // wait(100, msec);
      // go(180.5, 42, 60);
      // lick();
      // wait(50,msec);
      // go(134, -11, 60);
      // wait(250, msec);
      // proBono();
      // take();
      // wait(4000, msec);
      // go(136, 15, 60);
      // go(90, 40, 15);
      // go(5, 40, 15);
      // go(5, 2, 15);
      // go(5, -4, 15);
      // go(5, 4, 15);


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
  intake_2.setVelocity(75, percent);
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
    Controller1.ButtonL2.pressed(outtake);
    Controller1.ButtonL2.released(intakeStop);
    Controller1.Axis3.changed(left_drive);
    Controller1.Axis2.changed(right_drive);
    Controller1.ButtonA.pressed(toggleHood);
    Controller1.ButtonDown.pressed(toggleTongue);

  while (true) {
    wait(20, msec);
  }
  return(0);
}