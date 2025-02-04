#include "main.h"


// class DriveControllerOutput : public okapi::ControllerOutput<double> {  //controller output class for control over chassis
// public:
//     std::shared_ptr<okapi::ChassisModel> ChassisModel;  //shared pointer to a chassis model

//     DriveControllerOutput(const std::shared_ptr<okapi::ChassisModel> ChassisModel); //constructor taking a shared pointer to a chassis model

//     void controllerSet(double ivalue) override; //function PID controller calls to move the chassis

//     void arcade(double iforawardSpeed, double iyaw);
// };

// class DriveControllerInput  :   public okapi::ControllerInput<double> {
// public:
//     std::shared_ptr<pros::Imu> inputImu; //shared pointer to an inertial sensor

//     DriveControllerInput(const std::shared_ptr<pros::Imu> inertialSensor);  //constructor taking a shared pointer to an interial sensor

//     double controllerGet(void) override; //function PID controller calls to read from the inertial sensor
// };

// class DriveController { //class to handle gyro rotate PID controller
// private:
//     //PID constants for gyro rotate
//     double kP = 10;           //0.02
//     double kI = 0;       //0.5/1000
//     double kD = 0;  //0.42/1000

//     double zeroPosition = 0;    //offset fro zero

//     //controller input
//     std::shared_ptr<DriveControllerInput> controllerInput;  //shared pointer to a controller input

//     //controller output
//     std::shared_ptr<DriveControllerOutput> controllerOutput; //shared pointer to a controller output

//     //gyro turning pid controller
//     okapi::IterativePosPIDController rotateController;

//     //utility to determine if the controller has settled
//     okapi::SettledUtil rotateSettledUtility;

// public:
//     //class constructor
//     DriveController(std::shared_ptr<DriveControllerInput> input, std::shared_ptr<DriveControllerOutput> output);

//     //function to set pid constants
//     void setRotateConstants(double kP, double kI, double kD);

//     //function to set zero position
//     void tareHeading(void);

//     //function to set heading offset
//     void setHeadingOffset(double offset);

//     //function to reutrn heading with offset
//     double getHeading();

//     //roates to an absolute target
//     void rotateAbsolute(double idegTarget);
//     void setTarget(double idegTarget, int timeOut);

//     //rotates to a target relative to the robot's current position
//     void rotateRelative(double idegTarget);

//     void fullSend(int targetHeading, int speed, MotorGroup leftDrive, int target);    
// };
// #include "main.h"

#ifndef PROFILES
#define PROFILES

                                //Everything is relative. use chassis.tarePosition at the beginning of the auton
//functions
extern void driveIt(int targHeading, double rpm);
extern void chassisProfiling(double target, double error, long double aggr, double targHeading, int timeOut=60000);     //target in motor ticks, error in motor ticks (at least 6), aggression (stop, 0.004), targHeading-- gyro (0-360)
extern void driveItCurve(double targHeading, double rpm, double delay, double initHead, int multiplier);
extern void curveProfiling(double target, double error, long double aggr, double targHeading, double delay, double initHead, bool forward, int timeOut=60000);  // delay= drive before start curve, initHeading= heading before start curve, forward= true or false
extern void profile(double target, double error, long double aggr, double targHeading); 

//Misc.
extern double headg;
extern int multiplier;

#endif
