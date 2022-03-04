/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Woojin Lee                                                */
/*    Created:      Thu Aug 24 2021                                           */
/*    Description:  507A 2021-2022 Competition Template                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath> 
using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
controller Controller;

//drivetrain motors
motor backLeft(PORT20, gearSetting::ratio6_1,true);
motor backRight(PORT15, gearSetting::ratio6_1,false);
motor frontLeft(PORT19, gearSetting::ratio6_1,true);
motor frontRight(PORT17, gearSetting::ratio6_1,false);
motor middleLeft(PORT6,gearSetting::ratio6_1,true);
motor middleRight(PORT1,gearSetting::ratio6_1,false);


//motor groupings
motor_group leftSide(backLeft,frontLeft,middleLeft);
motor_group rightSide(backRight,frontRight,middleRight);
drivetrain driveTrain(leftSide,rightSide);

//ring conveyor motor
motor conveyorBelt(PORT8,gearSetting::ratio6_1,true);

//motor for four bars in front and back
motor backMogo(PORT7,gearSetting::ratio36_1,true);

//position sensors
inertial Inertial(PORT18);
rotation parallelSensor(PORT14);
rotation perpSensor(PORT11);

//bumper sensors for end limit detection
bumper backMogoUp(Brain.ThreeWirePort.A);
bumper backMogoDown(Brain.ThreeWirePort.B);

//pneumatics for lock and clamp
pneumatics frontMogo(Brain.ThreeWirePort.H);
pneumatics frontTilt1(Brain.ThreeWirePort.F);
pneumatics frontTilt2(Brain.ThreeWirePort.G);
pneumatics backMogoClamp(Brain.ThreeWirePort.E);
pneumatics spike(Brain.ThreeWirePort.C);

// ---- END VEXCODE CONFIGURED DEVICES ----

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started. This        */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

// All activities that occur before the competition starts
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  //calibrates inertial sensor
  Inertial.calibrate();
  wait(2, sec);
  // Example: clearing encoders, setting servo positions, ...
}

//variables for conveyorIntake task
directionType conveyorDirection=directionType::fwd;
bool conveyorToggle=0;
//task to toggle conveyorBelt during driver control
int conveyorIntake(){
  while(conveyorToggle){
    conveyorBelt.spin(conveyorDirection,100,pct);
  }
  conveyorBelt.stop(coast);
  return 1;
}

//task to flip out back four bar arms
int start(){
  backMogo.stop(brake);
  spike.close();
  backMogo.resetRotation();
  conveyorBelt.resetRotation();
  backMogoClamp.open();
  frontMogo.open();
  frontTilt1.open();
  frontTilt2.open();
  parallelSensor.resetPosition();
  perpSensor.resetPosition();
  leftSide.resetPosition();
  rightSide.resetPosition();
  Inertial.resetRotation();
  return 1;
}
double currentX=0;
double currentY=0;
double dB=3.4;
double previousParallel=0;
double previousPerp=0;
double previousTheta=0;
double totalPerp=0;
double currentParallel=0;
double currentPerp=0;;
double currentTheta=0;
double deltaParallel=0;
double deltaPerp=0;
double deltaTheta=0;
double deltaXLocal=0;
double deltaYLocal=0;
double deltaXGlobal=0;
double deltaYGlobal=0;
double averageTheta=0;
int odometry(){
  
  while(true){
    currentParallel=(parallelSensor.position(deg));
    currentPerp=(perpSensor.position(deg));

    deltaParallel=(((currentParallel-previousParallel)*M_PI)/180)*2.75;
    deltaPerp=(((currentPerp-previousPerp)*M_PI)/180)*2.75;

    previousParallel=currentParallel;
    previousPerp=currentPerp;

    currentTheta=Inertial.rotation()*M_PI/180.0;
    deltaTheta=currentTheta-previousTheta;
    previousTheta=currentTheta;

    if (deltaTheta<1){
      deltaXLocal=deltaPerp;
      deltaYLocal=deltaParallel;
    }
    else{
      deltaXLocal=2*sin(deltaTheta/2.0)*((deltaPerp/deltaTheta)+dB);
      deltaYLocal=2*sin(deltaTheta/2.0)*(deltaParallel/deltaTheta);
    }

    averageTheta=currentTheta-(deltaTheta/2.0);

    deltaXGlobal=(deltaYLocal*cos(averageTheta))-(deltaXLocal*sin(averageTheta));
    deltaYGlobal=(deltaYLocal*sin(averageTheta))+(deltaXLocal*cos(averageTheta));

    currentX+=deltaXGlobal;
    currentY=deltaYGlobal;

    wait(10,msec);
  }
  return 1;
}
task Odometry(odometry);

void arcPID(double leftFinal, double rightFinal){
  double kP=0.015;
  double kD=0.0;
  leftSide.resetPosition();
  rightSide.resetPosition();
  double leftPreviousError=0;
  double rightPreviousError=0;
  double leftError=(leftFinal*(360/(2.75*M_PI)));
  double rightError=(rightFinal*(360/(2.75*M_PI)));
  while (fabs(leftError)>=5&&fabs(rightError)>=5){
      //Get robot turn position
      double leftPosition=(backLeft.position(deg)+middleLeft.position(deg)+frontLeft.position(deg)/3);
      double rightPosition=(backRight.position(deg)+middleRight.position(deg)+frontRight.position(deg)/3);

      //proportional
      leftError=leftFinal-leftPosition;
      rightError=rightFinal-rightPosition;
      //derivative
      double leftDerivative=leftError-leftPreviousError;
      double rightDerivative=rightError-rightPreviousError;


      //sets up the motor power for each side
      double leftMotorPower=leftError*kP+leftDerivative*kD;
      double rightMotorPower=rightError*kP+rightDerivative*kD;

      //spins in opposite directions for pinpoint turns
      leftSide.spin(fwd,leftMotorPower,volt);
      rightSide.spin(fwd,rightMotorPower,volt);

      //updates the previousError for the next while loop iteration for accurate derivatives
      leftPreviousError=leftError;
      rightPreviousError=rightError;

      // Sleep the PD for a short amount of time to prevent wasted resources.
      wait(10,msec);
    }
    leftSide.stop(brake);
    rightSide.stop(brake);
}

double driveRemainder=0;
void drivePD(double finalDrive){
  parallelSensor.resetPosition();
  double turnFinalValue=Inertial.rotation();
  double driveError=0;
  double drivePreviousError=0;
  double turnPreviousError=0;
  bool condition=true;
  double driveFinalValue=(finalDrive*(360/(2.75*M_PI))+driveRemainder);


  while (condition){
    double drivekP=.017;
    double drivekD=.03;
    double turnkP=.24;
    double turnkD=.7;
    double drivePosition=parallelSensor.position(deg);
    double turnPosition=Inertial.rotation(deg);
    //proportional
    driveError=driveFinalValue-drivePosition;
    double turnError=turnFinalValue-turnPosition;
    
    //derivative
    double driveDerivative=driveError-drivePreviousError;
    double turnDerivative=turnError-turnPreviousError;


    //sets up the motor power for each side
    double driveMotorPower=driveError*drivekP+driveDerivative*drivekD;
    double turnMotorPower=turnError*turnkP+turnDerivative*turnkD;

    rightSide.spin(fwd,driveMotorPower-turnMotorPower,volt);
    leftSide.spin(fwd,driveMotorPower+turnMotorPower,volt);
    Brain.Screen.printAt(20,80,"%f",middleLeft.velocity(velocityUnits::pct));

    //updates the previousErrors for the next while loop iteration for accurate derivatives
    drivePreviousError=driveError;
    turnPreviousError=turnError;
    if ((fabs(driveError)<5)&&fabs(turnError)<2){
      condition=false;
    }
    Brain.Screen.printAt(20,20,"%f",driveError);
    Brain.Screen.printAt(20,40,"%f",turnError);

    //Sleep the PD for a short amount of time to prevent wasted resources.
    wait(10,msec);
  }
  driveRemainder=driveError;
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(20,20,"%f",Inertial.rotation());
  driveTrain.stop(brake);
  //stops motors once error is within margin of error
}

//PD for accurate turns using inertial sensor
  void turnPD(double finalTurnValue){
    //turn PD tuning values
    double turnkP=24;
    double turnkD=13;
    double turnkI=0;
    //doesn't reset value to use absolute positioning for more accurate positioning
    //sets up while loop condition
    double turnError=finalTurnValue;
    double toggler=false;
    if (fabs(turnError)<1){
      turnError=1;
      toggler=true;
    }
    double previousTurnError=0;
    double integral=0;

    //margin of error of 2 degrees
    while (fabs(turnError)>=1){
      //Get robot turn position
      if (toggler){
        turnError=finalTurnValue;
        toggler=0;
      }
      double turnPosition=Inertial.rotation();

      //proportional
      turnError=finalTurnValue-turnPosition;

      //derivative
      double turnDerivative=turnError-previousTurnError;

      if(turnError<5){
        integral+=turnError;
      }
      //sets up the motor power for each side
      double turnMotorPower=turnError*turnkP+turnDerivative*turnkD+integral*turnkI;

      //spins in opposite directions for pinpoint turns
      leftSide.spin(fwd,turnMotorPower,dps);
      rightSide.spin(reverse,turnMotorPower,dps);

      //updates the previousError for the next while loop iteration for accurate derivatives
      previousTurnError=turnError;

      // Sleep the PD for a short amount of time to prevent wasted resources.
      wait(10,msec);
    }
    //stops motors once error is within margin of error
    leftSide.stop(brake);
    rightSide.stop(brake);
  }

//PD for accurate back four bar movements using potentiometer sensor
double finalBackValue;
int selfBalance(){
  //use pitch, back pointing down = +, front pointing down= -.
  driveTrain.driveFor(reverse,24,inches,50,velocityUnits::pct,true);
  directionType platform=reverse;
  int speed=40;
  while (true){
    if (Inertial.pitch(deg)>=7){
      platform=fwd;
      speed=40;
    }
    else if (Inertial.pitch(deg)<=-7){
      platform=reverse;
      speed=40;
    }
    else{
      speed=0;
    }
    driveTrain.drive(platform,speed,velocityUnits::pct);
  }
  driveTrain.stop(brake);
  return 1;
}
void frontMogoGetter(){
  frontMogo.close();
  wait(300,msec);
  frontTilt1.close();
  frontTilt2.close();

}

//moves both backMogo motors for autonomous in case PD isn't used
void backMogoLift(double degrees, bool wait){
  backMogo.rotateTo(degrees,rotationUnits::deg,100,velocityUnits::pct,wait);
}
//constantly spins ring intake
void ringPickUp(bool toggle){
  if (toggle==1){
    conveyorBelt.spin(fwd,100,pct);
  }
  else{
    conveyorBelt.stop(coast);
  }
}
int timerparameter=500;
int frontClampTimed(){
  //vestigial
  wait(timerparameter,msec);
  frontMogo.close();
  return 1;
}

// ............................................................................
// ............................................................................
// ............................................................................

void rightAutonWP(){
  task Start=task(start);
  drivePD(-38);
  backMogoClamp.close();
  backMogoLift(120,0);
  drivePD(30);
  turnPD(-90);
  drivePD(24);
  frontMogoGetter();
  // drivePD(-1420, -1420,0,0);
  // Start.stop();
  // backMogoClamp.close();
  // backMogoLift(40,false);
  // drivePD(770, 770,0,0);
  // turnPD(-88);
  // backMogoClamp.open();
  // drivePD(580,580,1,130);
  // driveTrain.driveFor(-2.5, distanceUnits::in, 50, velocityUnits::pct);
  // backMogoLift(150, false);
  // ringPickUp(1);
  // turnPD(0);
  // driveTrain.driveFor(-24.75, distanceUnits::in, 18, velocityUnits::pct);
  // drivePD(1080,1080,0,0); 
  // ringPickUp(0);
  // backMogoLift(0,0);
  // turnPD(-107);
  // drivePD(-470,-470,0,0);
  // backMogoClamp.close();
  // driveTrain.stop(coast);
}

// ............................................................................
// ............................................................................
// ............................................................................

void rightAutonMA(){
  // Inertial.resetRotation();
  // task Start=task(start);
  // turnPD(-32);
  // drivePD(-1835,-1835,0,0);
  // backMogoClamp.close();
  // backMogoLift(40, 0);
  // drivePD(940,940,0,0);
  // turnPD(-88);
  // backMogoClamp.open();
  // drivePD(1150,1150,1,130);
  // driveTrain.driveFor(-2, distanceUnits::in, 50, velocityUnits::pct);
  // ringPickUp(1);
  // backMogoLift(150, false);
  // turnPD(0);
  // driveTrain.driveFor(-24.75, distanceUnits::in, 15, velocityUnits::pct);
  // drivePD(1080,1080,0,0); 
  // driveTrain.stop(coast);
}

// ............................................................................
// ............................................................................
// ............................................................................

void leftAutonWP(){}
void auton5(){}
void right2N2(){
  // Inertial.resetRotation();
  // task Start=task(start);
  // drivePD(-1420, -1420,0,0);
  // Start.stop();
  // backMogoClamp.close();
  // backMogoLift(120,false);
  // drivePD(300,300,0,0);
  // turnPD(-180);
  // drivePD(-700,-700,0,0);
  // backMogoClamp.open();
  // wait(100,msec);
  // drivePD(620, 620,0,0);
  // wait(100,msec);
  // backMogoLift(0, 0);
  // turnPD(-52);
  // drivePD(-1060,-1060,0,0);
  // backMogoClamp.close();
  // backMogoLift(40,0);
  // turnPD(-30);
  // drivePD(1230,1230,0,0);
  // turnPD(-88);
  // backMogoClamp.open();
  // drivePD(810,810,true,110);
  // ringPickUp(1);
  // backMogoLift(0, 0);
  // drivePD(-750,-750,0,0);
  // backMogoClamp.close();
  // backMogoLift(120, 0);
  // driveTrain.stop(coast);
}

// ............................................................................
// ............................................................................
// ............................................................................

void auton7(){}
void skillAuton(){
  // Inertial.resetRotation();
  // task Start(start);
  // turnPD(11);
  // drivePD(500,500,1,100);
  // Start.stop();
  // drivePD(-800,-800,0,0);
  // turnPD(109.6);
  // drivePD(-1210,-1210,0,0);
  // drivePD(-370,-370,0,0);
  // backMogoClamp.close();
  // backMogoLift(850, false);
  // turnPD(117);
  // ringPickUp(1);
  // wait(200,msec);
  // drivePD(-2335,-2335,0,0);
  // turnPD(112);
  // backMogoLift(450, true);
  // backMogoClamp.open();
  // backMogoLift(800,1);
  // drivePD(310,310,0,0);
  // backMogoLift(0, 0);
  // ringPickUp(0);
  // turnPD(0);
  // drivePD(-490,-490,0,0);
  // frontMogo.open();
  // drivePD(-400,-400,0,0);
  // turnPD(176);
  // drivePD(665,665,1,130);
  // drivePD(-335,-335,0,0);
  // turnPD(217);
  // drivePD(-1030,-1030,0,0);
  // drivePD(-480,-480,0,0);
  // backMogoClamp.close();
  // backMogoLift(70, 1);
  // drivePD(-500,-500,0,0);
  // backMogoLift(750, 0);
  // turnPD(253);
  // ringPickUp(1);
  // wait(500,msec);
  // drivePD(-1400,-1400,0,0);
  // backMogoLift(500, 1);
  // wait(1500,msec);
  // backMogoClamp.open();
  // backMogo.rotateTo(750,deg,25,velocityUnits::pct,false);
  // drivePD(350,350,0,0);
  // ringPickUp(0);
  // turnPD(180);
  // backMogoLift(0, 0);
  // drivePD(-650,-650,0,0);
  // frontMogo.open();
  // drivePD(-380,-380,0,0);
  // turnPD(2.64);
  // drivePD(685,685,1,130);
  // wait(300,msec);
  // drivePD(-875,-875,0,0);
  // turnPD(99);
  // drivePD(-400,-400,0,0);
  // drivePD(-340,-340,0,0);
  // backMogoClamp.close();
  // backMogoLift(660, 0);
  // turnPD(69);
  // ringPickUp(1);
  // drivePD(-2080,-2080,0,0);
  // backMogoClamp.open();
  // drivePD(330,330,0,0);
  // turnPD(0);
  // ringPickUp(0);
  // backMogoLift(0, 1);
  // drivePD(1500,1500,0,0);
  // turnPD(50.5);
  // drivePD(-585,-585,0,0);
  // drivePD(-320,-320,0,0);
  // backMogoClamp.close();
  // backMogoLift(40,0);
  // drivePD(600,600,0,0);
  // turnPD(-34);
  // backMogoLift(700, 0);
  // ringPickUp(1);
  // drivePD(-3300,-3300,0,0);
  // wait(750,msec);
  // turnPD(-110);
  // drivePD(-570,-570,0,0);
  // backMogoClamp.open();
  // turnPD(-105);
  // backMogoLift(0, 0);
  // drivePD(2710,2710,0,0);
  // turnPD(-175);
  // drivePD(-600,-600,0,0);
  //  drivePD(-400,-400,0,0);
  // backMogoClamp.close();
  // backMogoLift(800, 1);
  // turnPD(-240);
  // drivePD(-200,-200,0,0);
  // backMogoClamp.open();
  // drivePD(300,300,0,0);
  // Inertial.resetRotation();
  // turnPD(180);
}

// ............................................................................
// ............................................................................
// ............................................................................

//easy way to switch between different autons
//may implement a more sophisticated auton select later
void auton(int auton){
  if (auton==1){
    rightAutonWP();
  }
  else if (auton==3){
    right2N2();
  }
  else if (auton==2){
    rightAutonMA();
  }
  else if (auton==4){
    leftAutonWP();
  }
  else if (auton==5){
    auton5();
  }
  else if (auton==6){
    right2N2();
  }
  else if (auton==7){
    auton7();
  }
  else if (auton==8){
    skillAuton();
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  auton(1);\
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
int joystick3;
int joystick1;
void usercontrol(void) {
  // User control code here, inside the loop
    while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    if(abs(Controller.Axis3.value())<=0){
       joystick3=0;
    }
    else{
      joystick3=Controller.Axis3.value();
    }
    if (abs(Controller.Axis1.value())<=0){
       joystick1=0;
    }
    else{
      joystick1=Controller.Axis1.value();
    }
    if (frontRight.installed()&&middleRight.installed()&&backRight.installed()){
    }
    else{
      Brain.Screen.printAt(200,60,"Motor(s) on Right Side Not Connected!");
    }
    rightSide.spin(fwd,((joystick3*.388)-(joystick1*.388)),volt);
    if (frontLeft.installed()&&middleLeft.installed()&&backLeft.installed()){
    }
    else{
      Brain.Screen.printAt(200,80,"Motor(s) on Left Side Not Connected!");
    }
     leftSide.spin(fwd,((joystick3*.355)+(joystick1*.284)),volt);
    
    //controls front four-bar, uses button sensors to prevent over-rotation
    if (Controller.ButtonR2.pressing()){
      frontTilt1.open();
      frontTilt2.open();
      wait(200,msec);
      frontMogo.open();
    }
    else if (Controller.ButtonR1.pressing()){
      frontMogo.close();
      wait(300,msec);
      frontTilt1.close();
      frontTilt2.close();
    }
    
    //controls movement for back four bar.
    if (Controller.ButtonL2.pressing()&&!backMogoDown.pressing()){
      backMogo.spin(reverse,100,pct);
    }
    else if (Controller.ButtonL1.pressing()&&!backMogoUp.pressing()){
      backMogo.spin(fwd,100,pct);
    }
    else{
      backMogo.stop(brake);
    }
    
    //controls the conveyor belt, uses scuf controller
    if (Controller.ButtonDown.pressing()){
      conveyorDirection=directionType::rev;
      conveyorToggle=1;
      task::sleep(25);
    }
    else if(Controller.ButtonB.pressing()){
      conveyorToggle=false;
      task::sleep(25);
    }
    else if(backMogo.rotation(deg)>=150){
      conveyorDirection=directionType::fwd;
      conveyorToggle=1;
      wait(25,msec);
    }
    else if (backMogo.rotation(deg)<=150){
        conveyorToggle=false;
        wait(25,msec);
    }

    //controls back pneumatic clamp, uses scuf controller
    if (Controller.ButtonY.pressing()){
      backMogoClamp.open();
    }
    else if (Controller.ButtonRight.pressing()){
      backMogoClamp.close();
    }
    if (Controller.ButtonX.pressing()){
      spike.close();
    }
    else if (Controller.ButtonA.pressing()){
      spike.open();
    }

    task ConveyorIntake(conveyorIntake);
    Brain.Screen.printAt(20, 20, "%f", frontLeft.velocity(rpm));
    Brain.Screen.printAt(20, 40, "%f", backLeft.velocity(rpm));
    Brain.Screen.printAt(20, 60, "%f", frontRight.velocity(rpm));
    Brain.Screen.printAt(20, 80, "%f", backRight.velocity(rpm));
    Brain.Screen.printAt(200, 20, "%f", middleLeft.velocity(rpm));
    Brain.Screen.printAt(200, 40, "%f",middleRight.velocity(rpm));
    Brain.Screen.printAt(200, 60, "%f", currentX);
    Brain.Screen.printAt(200, 80, "%f",currentY);
  
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

// Main will set up the competition functions and callbacks.
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}