#include "main.h"
#include "okapi/api/control/async/asyncMotionProfileController.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
// #include "okapi/impl/device/opticalSensor.hpp"
#include "pros/misc.h"
#include "rotate.hpp"
#include "solenoid.hpp"

extern Controller controller1;
extern MotorGroup lDrive;
extern MotorGroup rDrive;
extern MotorGroup chassis;
extern Solenoid fire;
extern Solenoid unfire;


// extern void rotate(int target);

extern std::shared_ptr<ChassisController> drive;
extern std::shared_ptr<AsyncMotionProfileController> profileController;

 extern GyroRotateController gyroRotate;
 extern std::shared_ptr<pros::Imu> imu;

