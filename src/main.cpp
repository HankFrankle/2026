#include "vex.h"

using namespace vex;

brain Brain;
competition Competition;

controller Controller1 = controller(primary);
motor intake_1 = motor(PORT17, ratio6_1);
motor intake_2 = motor(PORT18, ratio6_1, true);

inertial Gyro1 = inertial(PORT20);
rotation odom = rotation(PORT19, true);

digital_out dih = digital_out(Brain.ThreeWirePort.H);
digital_out lift = digital_out(Brain.ThreeWirePort.F);
digital_out tongue = digital_out(Brain.ThreeWirePort.E);
digital_out intakeL = digital_out(Brain.ThreeWirePort.G);


motor frontLeft = motor(PORT11, ratio6_1); 
motor midLeft = motor(PORT12, ratio6_1, true);
motor backLeft = motor(PORT13, ratio6_1, true);
motor frontRight = motor(PORT14, ratio6_1, true);
motor midRight = motor(PORT15, ratio6_1);
motor backRight = motor(PORT16, ratio6_1);

motor_group leftDrive = motor_group(frontLeft, midLeft, backLeft);
motor_group rightDrive = motor_group(frontRight, midRight, backRight);

motor_group intake = motor_group(intake_1, intake_2);

int auton = 6;

#define cs Controller1.Screen
#define Button Controller1.Button 

struct Values 
{
 float v1;
 float v2; 
 float v3;
};


// This is essentially a parent PID function that we can use for all motor control shit, and we'll just have to tune the kP, kI, and kD or each thing

// Values universalPID(float target, float current, float kp, float ki, float kd, float prevErr, float integral_accum, float interval) {

//  float err = target - current;
//  float integral = integral_accum + (err * interval);
//  float derivative = (err - prevErr) / interval;
//  float control = kp * err + ki * integral + kd * derivative;

//  return {control, err, integral};
// }

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

 float kp = 0.8f; // This is what we have to tune for an actual PID
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



 #define NGw ((float)48)
 #define NGm ((float)36)
 #define circ ((float)10.21)

 #define driveWidth ((float)14)

 #define wheelRatio ((float)(NGw / NGm) * (360 / circ))
 #define gyro (int)(round(Gyro1.rotation(degrees)))
 #define posn ((rightDrive.position(degrees) + leftDrive.position(degrees)) / 2) 
 #define driveGain ((float)0.27)
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



void arcRight(float dir, float turnRad, float clamp = 50, float speed = 100) {
 float error = dir - gyro;
 float base = (speed / 100) * error * turnGain;

 leftDrive.spin(forward);
 rightDrive.spin(forward);

 while (abs(error) > 1) {

 error = dir - gyro;
 base = error * turnGain;


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


void arcBackRight(float dir, float turnRad, float clamp = 50, float speed = 100) {
 float error = dir - gyro;
 float base = (speed / 100) * error * turnGain;

 leftDrive.spin(forward);
 rightDrive.spin(forward);

 while (abs(error) > 1) {

 error = dir - gyro;
 base = error * turnGain;


 base = -mid(base, clamp, (turnRad / 5) *15);

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


void arcLeft(float dir, float turnRad, int clamp = 50, float speed = 100) {
 float error = dir - gyro;
 float base = (speed / 100 ) * error * turnGain;

 leftDrive.spin(forward);
 rightDrive.spin(forward);

 while (abs(error) > 1) {

 error = dir - gyro;
 base = error * turnGain;

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


void arcBackLeft(float dir, float turnRad, int clamp = 50, float speed = 100) {
 float error = dir - gyro;
 float base = (speed / 100 ) * error * turnGain;

 leftDrive.spin(forward);
 rightDrive.spin(forward);

 while (abs(error) > 1) {

 error = dir - gyro;
 base = error * turnGain;

 base = -mid(base, clamp, (turnRad / 5) * 15);

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


void go(float dir, float dist, int clamp = 70, int timeO = 150, float turnClamp = 40) {

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

 
 float leftPower = -motorPower;
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
 leftDrive.setVelocity((double)driveSpeed - angleError * 0.3, percent);

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

void mid(void) {
    intake.setVelocity(70, percent);
    intake.spin(forward);
    liftState = 0;
    lift.set(liftState);
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

bool antiJamEnable = 0;

int antiJamThread(void*) {
    while (true) {
    
            if(antiJamEnable == 1) {
                if(abs(intake_1.velocity(rpm)) <= 15 && intake_1.power(watt) >= 2.5) {

                    intake_1.spin(forward);
                    intake_1.setVelocity(-75, percent);
                    wait(300, msec);
                    intake_1.setVelocity(0, percent);

                    Brain.Screen.clearScreen();
                    Brain.Screen.setCursor(1, 1);
                    Brain.Screen.print("Jammed");

                }
            }


        task::sleep(20);
    }
    return 0;
}


bool midMoveLickEnable = false;
int midMoveLickWaitTime = 0;

int midMoveLick(void*) {
    while (true) {
        if (midMoveLickEnable) {
            Brain.Screen.clearScreen();
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("thread fired");   // does this appear?
            wait(midMoveLickWaitTime, msec);
            tongueState = 1;
            tongue.set(1);
            midMoveLickEnable = false;
        }
        task::sleep(20);
    }
    return 0;
}

void swallow(int time) {
    midMoveLickWaitTime = time;
    midMoveLickEnable = true;
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("swallow called");   // does this appear?
}


bool midMoveUnLickEnable = false;
int midMoveUnLickWaitTime = 0;


int midMoveUnLick(void*) {
    while (true) {
        if (midMoveUnLickEnable) {
            Brain.Screen.clearScreen();
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("thread fired");   // does this appear?
            wait(midMoveUnLickWaitTime, msec);
            tongueState = 0;
            tongue.set(0);
            midMoveUnLickEnable = false;
        }
        task::sleep(20);
    }
    return 0;
}

void spit(int time) {
    midMoveUnLickWaitTime = time;
    midMoveUnLickEnable = true;
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("swallow called");   // does this appear?
}



void autonomous(void) {
 leftDrive.setStopping(brake);
 rightDrive.setStopping(brake);

 switch (auton) {
 case 1 :


 proBono();

 proBono();

 flick();

 go(0, 34, 65);

 go(90, 0, 70, 150, 45);

 lick();

 wait(200, msec);

 take();

 go(90, 6, 20);

 leftDrive.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);
 rightDrive.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);
 
 wait(240, msec);

 go(91, -30, 40);

 wait(100, msec);

 flick();
 
 wait(700, msec);

 lick();

 arcRight(200, 8.5);

 flick();

 antiJamEnable = 1;
 
 arcLeft(190, 43, 68);

 arcLeft(160, 105, 76);

 lick();

 wait(50, msec);

 go(138, -45, 25);

 mid();

 antiJamEnable = 0;




//  arcRight(192, 4.5);
 
//  flick();
 
//  take();

//  wait(25, msec);

//  go(217, 20, 40);

//  wait(50, msec);

//  go(170, 0, 70, 150, 25);

//  wait(25, msec);

//  go(180, 56, 80);

//  lick();

//  // go(180, 4)j;

//  go(135, -15, 50, 150, 50);

//  wait(100, msec);

//  take(45);

//  proBono();

//  wait(750, msec);

//  proBono();

//  go(135, 48, 70);

//  take();

//  arcLeft(80, 19, 35);

//  leftDrive.spin(vex::directionType::fwd, 4, vex::voltageUnits::volt);
//  rightDrive.spin(vex::directionType::fwd, 4, vex::voltageUnits::volt);

//  wait(300, msec);

//  go(90, -28, 65);
 
//  flick();

 
 break;
 case 2 :

 take();

 proBono();

 flick();

 go(0, 8, 60);

 swallow(300);

 arcRight(130, 13.6);

 go(130, 25);

 wait(250, msec);

 go(180, 0);

 wait(100, msec);

 leftDrive.spin(vex::directionType::fwd, 4, vex::voltageUnits::volt);
 rightDrive.spin(vex::directionType::fwd, 4, vex::voltageUnits::volt);\

 wait(850, msec);

 go(182, -32, 60);

 flick();

 take();

 wait(1.75, sec);

 lick();

//  go(180, -0.5);

 arcLeft(95, 10);

 go(180, -23, 30);

 leftDrive.setStopping(hold);
 rightDrive.setStopping(hold);







 
 break;
 case 3 :

 take();

 proBono();

 flick();

 go(0, 8, 60);

 swallow(300);

 arcLeft(-130, 13.6);

 go(-130, 25);

 wait(250, msec);

 go(-180, 0);

 wait(100, msec);

 leftDrive.spin(vex::directionType::fwd, 4, vex::voltageUnits::volt);
 rightDrive.spin(vex::directionType::fwd, 4, vex::voltageUnits::volt);

 wait(850, msec);

 go(-182, -32, 60);

 flick();

 take();

 wait(1.75, sec);

 lick();

//  go(180, -0.5);

 arcLeft(-275, 10);

 go(-180, -23, 30);

 leftDrive.setStopping(hold);
 rightDrive.setStopping(hold);


 break;
 case 4 :


 take();

 proBono();

 flick();

 go(0, 8, 60);

 swallow(300);

 arcRight(130, 13.6);

 go(130, 25);

 wait(250, msec);

 go(180, 0);

 go(180, -24, 45);

 flick();

 take();

 wait(1, sec);

 lick();

 arcLeft(95, 10);

 go(180, -27, 30);

 leftDrive.setStopping(hold);
 rightDrive.setStopping(hold);

 
 break;
 case 5 :
 
  take();

 proBono();

 flick();

 go(0, 8, 60);

 swallow(300);

 arcLeft(-130, 13.6);

 go(-130, 25);

 wait(250, msec);

 go(-180, 0);

 go(-180, -24, 45);

 flick();

 take();

 wait(1, sec);

 lick();

 arcLeft(-275, 10);

 go(-180, -27, 30);

 leftDrive.setStopping(hold);
 rightDrive.setStopping(hold);

 
 break;

 case 6 :

 proBono();

 flick();

 take();

 go(0, -32, 60);

 wait(250, msec);

 swallow(280);

 go(-90, 6, 20);

 leftDrive.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);
 rightDrive.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);
 
 wait(350, msec);

 go(-91, -33, 40);

 wait(100, msec);

 flick();
 
 wait(1050, msec);

 lick();

 arcBackRight(5, 2.5);

 spit(400);

 flick();

 swallow(1350);

 go(2, 65, 60);

 wait(350, msec);

 go(-49, -13.5, 45);

 wait(100, msec);

 proBono();
 intake_2.setVelocity(65, percent);

 wait(500, msec);
 
 proBono();

 go(-49, 48, 60);

 arcLeft(-76, 20);

 leftDrive.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);
 rightDrive.spin(vex::directionType::fwd, 3, vex::voltageUnits::volt);
 
 wait(650, msec);

 go(-91, -33, 50);

 wait(100, msec);

 flick();
 
 break;
 case 7 :


 break;

 case 8:

 
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

void scoreLow (void) {
    intake_1.setVelocity(-50, percent);
    intake_2.setVelocity(-100, percent);
    intake.spin(forward);
    intakeL.set(1);
}

void outtake(void) {
 intake_1.spin(forward);
 intake_2.spin(forward);
 intake.setVelocity(-100, percent);
 dihState = 1;
}

void intakeStop(void) {
 intake.setStopping(coast);
 intake.setVelocity(0, percent);
 intakeL.set(0);
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
 lift.set(1);
 dih.set(1);


 

 leftDrive.setStopping(coast);
 rightDrive.setStopping(coast);

 while (1) {

 // if(Controller1.ButtonL2.pressing() && Controller1.ButtonR2.pressing()) {
 // intake_1.spin(forward);
 // intake_2.spin(forward);
 // intake.setVelocity(-100, percent);
 // dihState = 1;
 
 // } else{
 // intake.stop();
 // }

 dih.set(dihState);
 tongue.set(tongueState);
 lift.set(liftState);

 wait(5, msec);
 }
 }


 
 int main() {
 thread moveLick(midMoveLick, nullptr);
 thread antiJam(antiJamThread, nullptr);
 thread moveUnLick(midMoveUnLick, nullptr);
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
 Controller1.ButtonL2.pressed([]() { dihState = 0; dih.set(1); });
 Controller1.ButtonL2.released([]() { dihState = 1; dih.set(0); });
 Controller1.Axis3.changed(left_drive);
 Controller1.Axis2.changed(right_drive);
 Controller1.ButtonA.pressed(toggleIntakeLift);
 Controller1.ButtonUp.pressed(outtake);
 Controller1.ButtonUp.released(intakeStop);
 Controller1.ButtonDown.pressed(toggleTongue);
 Controller1.ButtonLeft.pressed(scoreLow);
 Controller1.ButtonLeft.released(intakeStop);

 while (true) {
 wait(20, msec);
 }
 return(0);
}