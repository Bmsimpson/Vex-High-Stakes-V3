#include "main.h"
#include "forward.hpp"
#include "okapi/api.hpp"
#include "subsystems.hpp"
#include <cmath>

// ForwardControllerOutput::ForwardControllerOutput(const std::shared_ptr<okapi::ChassisModel> chassisModel)
//     : chassisModel(chassisModel) {} // constructor taking a shared pointer to a chassis model

// void ForwardControllerOutput::controllerSet(double ivalue) { // function PID controller calls to move the chassis
//   //chassisModel->Forward(ivalue);
//   chassisModel->driveVectorVoltage(ivalue, 0);
// }

// // Controller Input
// ForwardControllerInput::ForwardControllerInput(const std::shared_ptr<okapi::ChassisModel> chassisModel)
//     : chassisModel(chassisModel) {} // constructor taking a shared pointer to an inertial sensor

// double ForwardControllerInput::controllerGet(void) { // function PID controller calles to read from the inertial sensor
//   //return inputImu->get_rotation();
//   return (chassisModel->getSensorVals()[0] + chassisModel->getSensorVals()[1]) / 2;
// }


// Forward Controller Class
 ForwardController::ForwardController(
    std::shared_ptr<ChassisModel> chassisModel,
    std::shared_ptr<pros::Imu> imu,
    double forwardKp,
    double forwardKi,
    double forwardKd,
    double headingKp,
    double headingKi,
    double headingKd
    ) : chassisModel(chassisModel),
        imu(std::shared_ptr<pros::Imu>(imu)),
        // controllerInput(std::make_shared<ForwardControllerInput>(chassisModel)),
        // controllerOutput(std::make_shared<ForwardControllerOutput>(chassisModel)),
        forwardController(okapi::IterativeControllerFactory::posPID(forwardKp, forwardKi, forwardKd)),
        headingController(okapi::IterativeControllerFactory::posPID(headingKp, headingKi, headingKd)),
        forwardSettledUtility(okapi::TimeUtilFactory::createDefault().getTimer(), 2, 3, 200_ms),
        headingSettledUtility(okapi::TimeUtilFactory::createDefault().getTimer(), 2, 3, 200_ms) {}

// get the average encoder value from left and right side
double ForwardController::getPosition(void) {
    return (chassisModel->getSensorVals()[0] + chassisModel->getSensorVals()[1]) / 2;
}

void ForwardController::setTarget(double target, int heading, int timeOut) {
  chassisModel->resetSensors();
  forwardController.setTarget(target); // set the target to Forward to
  headingController.setTarget(heading);

  forwardSettledUtility.reset();           // reset the settled utility
  long startTime = pros::millis();
  double maxPosition = getPosition();
  while (true) {                          // loop infinitely
    double chassisPosition = getPosition();
    // controllerOutput->controllerSet(forwardController.step(chassisPosition)); // set output to the value
    chassisModel->driveVectorVoltage(forwardController.step(chassisPosition), headingController.step(imu->get_rotation()));
    maxPosition = std::max(chassisPosition, maxPosition);                                                       // calculated by the PID controller
    //forwardController.setIntegratorReset(true); // reset the integral value when target is passed


    // temporary debugging
    pros::lcd::set_text(0, "Forward Debugging");
    pros::lcd::print(2, "Position %1.2f \n", chassisPosition);                     // print yaw value to LCD
    pros::lcd::print(3, "Target: %1.0f\n", target);                            // print target value to LCD
    pros::lcd::print(4, "Error: %1.2f \n", abs(chassisPosition - target));
    pros::lcd::print(5, "Max Position: %1.2f \n", maxPosition); // print error value to LCD
    printf("Yaw: %1.2f degrees Power: %1.2f\n", chassisPosition, forwardController.step(chassisPosition));

    if (
        (forwardSettledUtility.isSettled(forwardController.getError()) &&
        headingSettledUtility.isSettled(headingController.getError())) ||
        pros::millis() >= startTime + timeOut) { // if the controller is settled
      // printf("Settled\n");
      // controllerOutput->controllerSet(0); // set controller output to zero
      chassisModel->driveVectorVoltage(0, 0);
      break;                              // break out of loop
    }

    pros::delay(20); // delay 20 milliseconds to not overload cpu
    }
 }