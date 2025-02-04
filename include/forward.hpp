#pragma once
#include "main.h"

class ForwardControllerOutput : public okapi::ControllerOutput<double> { // controller output class for controll over chassis
public:
  std::shared_ptr<okapi::ChassisModel> chassisModel; // shared pointer to a chassis model

  ForwardControllerOutput(const std::shared_ptr<okapi::ChassisModel> chassisModel); // constructor taking a shared
                                                                            // pointer to a chassis model

  void controllerSet(double ivalue) override; // function PID controller calls to move the chassis
};


class ForwardControllerInput : public okapi::ControllerInput<double> { // controller input for reading from the inertial sensor
public:
   std::shared_ptr<okapi::ChassisModel> chassisModel;
  std::shared_ptr<pros::Imu> imu;

  ForwardControllerInput(const std::shared_ptr<okapi::ChassisModel> chassisModel); // constructor taking a shared
                                                                          // pointer to an inertial sensor

  double controllerGet(void) override; // function PID controller calles to read from the inertial sensor
};


class ForwardController { // class to hande gyro Forward PID controller
  private:
    std::shared_ptr<okapi::ChassisModel> chassisModel;
    std::shared_ptr<pros::Imu> imu;
    // PID constants for gyro Forward
    
    // controller input and output
    // std::shared_ptr<ForwardControllerInput> controllerInput; // shared pointer to a controller input
    // std::shared_ptr<ForwardControllerOutput> controllerOutput; // shared pointer to a controller output

    // utility to determine if the controller has settled
    okapi::SettledUtil forwardSettledUtility;
    okapi::SettledUtil headingSettledUtility;

  public:
    okapi::IterativePosPIDController forwardController;
    okapi::IterativePosPIDController headingController;

    // class constructor
    ForwardController( 
      std::shared_ptr<ChassisModel> chassisModel,
      std::shared_ptr<pros::Imu> imu,
      double forwardKp,
      double forwardKi,
      double forwardKd,
      double headingKp,
      double headingKi,
      double headingKd
    );
    double getPosition(void);
    void setTarget(double target, int heading, int timeOut);
};

