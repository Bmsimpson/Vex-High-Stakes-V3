#pragma once
#include "main.h"

class RotateControllerOutput : public okapi::ControllerOutput<double> { // controller output class for controll over chassis
public:
  std::shared_ptr<okapi::ChassisModel> chassisModel; // shared pointer to a chassis model

  RotateControllerOutput(const std::shared_ptr<okapi::ChassisModel> chassisModel); // constructor taking a shared
                                                                            // pointer to a chassis model

  void controllerSet(double ivalue) override; // function PID controller calls to move the chassis
};

class RotateControllerInput : public okapi::ControllerInput<double> { // controller input for reading from the inertial sensor
public:
  std::shared_ptr<pros::Imu> inputImu; // shared pointer to an inertial sensor

  RotateControllerInput(const std::shared_ptr<pros::Imu> inertialSensor); // constructor taking a shared
                                                                          // pointer to an inertial sensor

  double controllerGet(void) override; // function PID controller calles to read from the inertial sensor
};



class GyroRotateController { // class to hande gyro rotate PID controller
private:
  // PID constants for gyro rotate
 double kP = 0.0082;     // 0.01567;   // increase p until overshoots and then just undershoots (power)   0.015
 double kI = 0.01/1000; //0.012/1000;      // increase i until under a degree off either way
 double kD = 0.0023/100; //0.0568/100;       // increase d until only undershoots (dampening) too high of d = overshoots (sine curve) 0.05/100

  double zeroPosition = 0; // offset from zero

  // controller input
  std::shared_ptr<RotateControllerInput> controllerInput; // shared pointer to a controller input

  // controller output
  std::shared_ptr<RotateControllerOutput> controllerOutput; // shared pointer to a controller output

  // gyro turning pid controller
  // okapi::IterativePosPIDController rotateController;

  // utility to determine if the controller has settled
  okapi::SettledUtil rotateSettledUtility;

public:
  okapi::IterativePosPIDController rotateController;

  // class constructor
  GyroRotateController(std::shared_ptr<RotateControllerInput> input, std::shared_ptr<RotateControllerOutput> output);

  // function to set zero position
  void tareHeading(void);

  // function to set heading offset
  void setHeadingOffset(double offset);

  // function to return heading with offset
  double getHeading();

  // rotates to an absolute target
  void rotateAbsolute(double idegTarget);

  void setTarget(double idegTarget, int timeOut);

  // rotates to a target realative to the robots current position
  void rotateRelative(double idegTarget);
};