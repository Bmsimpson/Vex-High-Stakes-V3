#include "main.h"
#include "okapi/api.hpp"
#include "autoFiles.hpp"
#include "drive.hpp"
#include "rotate.hpp"
#include "forward.hpp"
#include "selection.h"
#include "solenoid.hpp"
#include "subsystems.hpp"
#include <cmath>

using namespace okapi;

Controller controller1;
ControllerButton r1(ControllerDigital::R1);
ControllerButton r2(ControllerDigital::R2);
ControllerButton l1(ControllerDigital::L1);
ControllerButton l2(ControllerDigital::L2);
ControllerButton x(ControllerDigital::X);
ControllerButton y(ControllerDigital::Y);
ControllerButton a(ControllerDigital::A);
ControllerButton b(ControllerDigital::B);
ControllerButton up(ControllerDigital::up);
ControllerButton down(ControllerDigital::down);
ControllerButton left(ControllerDigital::left);
ControllerButton right(ControllerDigital::right);

MotorGroup rDrive ({10, 8, 20});
MotorGroup lDrive ({-1, -2, -6});


Motor intake (12, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor conveyor (11, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor ladyBrown (13,true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

Solenoid clamp('A');
Solenoid doinker('E');
Solenoid intakeLift('D');

okapi::ADIButton ringLimit('F');

pros::Optical opticalSensor(18);
pros::Rotation rotationSensor(19);

bool elims = true;
bool colorSensing = true;
bool colorSensorEnabled = true;

enum ConveyorMode: int {
	User,	
	Auto,
	Eject,
};

ConveyorMode conveyorMode = User;


enum LadyBrownMode: int {
	Manual,
	Set,
};

LadyBrownMode ladyBrownMode = Manual;

int brownRotation;

std::shared_ptr<ChassisController> drive =
ChassisControllerBuilder()
.withMotors(lDrive, rDrive)
// .withDimensions(AbstractMotor::gearset::green, {{
// 	2.75_in, // wheel size
// 	12.5_in // distance from center of wheel to center of wheel
// }, 
// 	imev5GreenTPR/2.0 // gear ratio 	
// })
.withDimensions({AbstractMotor::gearset::blue, 0.2}, {{2.75_in, 12.5_in}, imev5GreenTPR})
.build();

std::shared_ptr<pros::Imu> imu = std::make_shared<pros::Imu>(15);	//Imu>(9) is the port number it's in

// takes the gyro and inputs it into the Drive controller
std::shared_ptr<RotateControllerInput> gyroInput = std::make_shared<RotateControllerInput>(imu);

// method for the gyro rotate controller writes values to the chassis motors
std::shared_ptr<RotateControllerOutput> rotateOutput = std::make_shared<RotateControllerOutput>(drive->getModel());

// class containing everything needed to do turns with the new inertial sensor
GyroRotateController gyroRotate(gyroInput, rotateOutput);

ForwardController forwardController(
	drive->getModel(),
	imu,

	// drive PID
	0.001, // kP 0.001
	0, // kI 0.0
	0, // kD 0.0

	// heading PID
	0.0082,     // 0.01567;   // increase p until overshoots and then just undershoots (power)   0.015
 	0.01/1000, //0.012/1000;      // increase i until under a degree off either way
 	0.0023/100 //0.0568/100;       // increase d until only undershoots (dampening) too high of d = overshoots (sine curve) 0.05/100
);

std::shared_ptr<AsyncMotionProfileController> profileController =
AsyncMotionProfileControllerBuilder()
.withLimits({
	5.0,					//maximum velocity
	2.0,					//maximum acceleration
	10.0					//maximum jerk
})
.withOutput(drive)
.buildMotionProfileController();

/*
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	doinker.closeSole();
	intakeLift.closeSole();

}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void wait(double time) {
	pros::delay(time*1000);
}

void red_5_ring(void) {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	// tares position and heading
	gyroRotate.tareHeading();
	// drives toward mogo
	chassis.tarePosition();
	chassisProfiling(-100, 6, 0.003, 0, 430);
	wait(0.1);
	gyroRotate.setTarget(37, 700);
	wait(0.1);
	chassis.tarePosition();
	chassisProfiling(-100, 6, 0.0009, 37, 200);
	chassisProfiling(-240, 6, 0.0001, 37, 700);
	// grabs mogo
	clamp.openSole();
	wait(0.3);
	// scores first ring
	gyroRotate.setTarget(105, 730);
	conveyor.moveVoltage(12000);
	wait(0.4);
	// goes and picks up second
	intake.moveVoltage(12000);
	conveyor.moveVoltage(0);
	chassis.tarePosition();
	chassisProfiling(305, 6, 0.003, 105, 640);
	intake.moveVoltage(0);
	wait(0.2);
	// turns to third
	gyroRotate.setTarget(178, 900);
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	wait(0.1);
	// drives to third
	chassis.tarePosition();
	chassisProfiling(135, 6, 0.003, 178, 455);
	wait(0.4);
	// backs off
	chassis.tarePosition();
	chassisProfiling(-110, 6, 0.003, 178, 400);
	wait(0.1);
	// turns to fourth
	gyroRotate.setTarget(209, 900);
	conveyor.moveVoltage(12000);
	wait(0.3);
	// picks up fourth
	chassis.tarePosition();
	chassisProfiling(130, 6, 0.003, 209, 450);
	conveyor.moveVoltage(0);
	chassis.tarePosition();
	chassisProfiling(-170, 6, 0.003, 209, 270);
	wait(0.3);
		// wait(0.2);
		// // backs off
		// chassis.tarePosition();
		// chassisProfiling(-120, 6, 0.003, 209, 450);
		// wait(0.4);
		// // turns to tower
		// gyroRotate.setTarget(70, 900);
		// ladyBrown.setBrakeMode(AbstractMotor::brakeMode::brake);
		// ladyBrown.moveVoltage(2000);
		// // drives to tower
		// chassis.tarePosition();
		// chassisProfiling(-390, 6, 0.003, 76, 700);
		// ladyBrown.moveVoltage(0);
		// conveyor.moveVoltage(0);
		// intake.moveVoltage(0);
		// chassisProfiling(-500, 6, 0.00006, 66, 410);
		// ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	gyroRotate.setTarget(0, 700);
	wait(0.15);
	conveyor.moveVoltage(12000);
	// drives toward 5th ring and lines up 45 degrees from corner
	chassis.tarePosition();
	chassisProfiling(570, 6, 0.001, 0, 800);
	gyroRotate.setTarget(45, 700);
	wait(0.1);
	conveyor.moveVoltage(0);
	// gets 5th ring
	chassis.tarePosition();
	chassisProfiling(580, 6, 0.0002, 45, 680);
	conveyor.moveVoltage(12000);
	chassisProfiling(800, 6, 0.0002, 45, 1400);
	chassis.tarePosition();
	chassisProfiling(-800, 6, 0.0003, 45, 1000);
}

void blue_5_ring() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	// tares position and headingSS
	gyroRotate.tareHeading();
	// drives toward mogo
	chassis.tarePosition();
	chassisProfiling(-100, 6, 0.003, 0, 430);
	wait(0.1);
	gyroRotate.setTarget(-37, 700);
	wait(0.1);
	chassis.tarePosition();
	chassisProfiling(-100, 6, 0.0009, -37, 200);
	chassisProfiling(-240, 6, 0.0001, -37, 700);
	// grabs mogo
	clamp.openSole();
	wait(0.3);
	// scores first ring
	gyroRotate.setTarget(-105, 730);
	conveyor.moveVoltage(12000);
	wait(0.4);
	// goes and picks up second
	intake.moveVoltage(12000);
	conveyor.moveVoltage(0);
	chassis.tarePosition();
	chassisProfiling(305, 6, 0.003, -105, 640);
	intake.moveVoltage(0);
	wait(0.2);
	// turns to third
	gyroRotate.setTarget(-178, 900);
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	wait(0.1);
	// drives to third
	chassis.tarePosition();
	chassisProfiling(135, 6, 0.003, -178, 455);
	wait(0.4);
	// backs off
	chassis.tarePosition();
	chassisProfiling(-110, 6, 0.003, -178, 400);
	wait(0.1);
	// turns to fourth
	gyroRotate.setTarget(-209, 900);
	conveyor.moveVoltage(12000);
	wait(0.3);
	// picks up fourth
	chassis.tarePosition();
	chassisProfiling(130, 6, 0.003, -209, 450);
	conveyor.moveVoltage(0);
	chassis.tarePosition();
	chassisProfiling(-170, 6, 0.003, -209, 270);
	wait(0.3);
	if (!elims) {
		wait(0.2);
		// backs off
		chassis.tarePosition();
		chassisProfiling(-120, 6, 0.003, -200, 450);
		wait(0.4);
		// turns to tower
		gyroRotate.setTarget(-70, 900);
		ladyBrown.setBrakeMode(AbstractMotor::brakeMode::brake);
		ladyBrown.moveVoltage(2000);
		// drives to tower
		chassis.tarePosition();
		chassisProfiling(-390, 6, 0.003, -76, 700);
		ladyBrown.moveVoltage(0);
		conveyor.moveVoltage(0);
		intake.moveVoltage(0);
		chassisProfiling(-500, 6, 0.00006, -66, 410);
		ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	} else {
		gyroRotate.setTarget(0, 700);
		wait(0.15);
		conveyor.moveVoltage(12000);
		// drives toward 5th ring and lines up 45 degrees from corner
		chassis.tarePosition();
		chassisProfiling(580, 6, 0.001, 0, 800);
		gyroRotate.setTarget(-45, 700);
		wait(0.1);
		conveyor.moveVoltage(0);
		// gets 5th ring
		chassis.tarePosition();
		chassisProfiling(580, 6, 0.0003, -45, 680);
		conveyor.moveVoltage(12000);
		chassisProfiling(800, 6, 0.00035, -45, 1400);
		chassis.tarePosition();
		chassisProfiling(-800, 6, 0.0003, -45, 1000);
		
		
	}
}

void red_3_ring() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(700, 0, 250);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveAbsolute(750, 12000);
	wait(0.6);
	// backs off
	forwardController.setTarget(-1000, 0, 600);
	// turns to mogo
	gyroRotate.setTarget(-46, 1000);
	// put ladybrown down
	ladyBrown.moveAbsolute(-200, 12000);
	// backs into mogo and clamps it 
	forwardController.setTarget(-1000, -46, 600);
	forwardController.setTarget(-250, -46, 500);
	clamp.openSole();
	wait(0.1);
	// turns to 2nd ring
	gyroRotate.setTarget(-146, 1100);
	// picks up 2nd ring
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	forwardController.setTarget(1200, -146, 1300);
	// turns and drives to line up with corner
	gyroRotate.setTarget(-45, 1200);
	conveyor.moveVoltage(12000);
	forwardController.setTarget(1120, -45, 1300);
	// turns to corner
	gyroRotate.setTarget(-93, 1400);
	// drives into corner
	conveyor.moveVoltage(12000);
	forwardController.setTarget(1150, -93, 1200);
	ladyBrown.moveAbsolute(280, 12000);
	wait(0.6);
	// backs out
	forwardController.setTarget(-2750, -87, 1500);
	conveyor.moveVoltage(0);
	forwardController.setTarget(-400, -80, 500);
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	ladyBrown.moveVoltage(-2000);
}

void blue_3_ring() { //Rename to 2 ring and wall
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(700, 0, 250);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveAbsolute(750, 12000);
	wait(0.5);
	// backs off
	forwardController.setTarget(-1000, 0, 600);
	// turns to mogo
	gyroRotate.setTarget(46, 900);
	// put ladybrown down
	ladyBrown.moveAbsolute(-200, 12000);
	// backs into mogo and clamps it
	forwardController.setTarget(-1200, 46, 800);
	forwardController.setTarget(-250, 46, 450);
	clamp.openSole();
	wait(0.1);
	// turns to 2nd ring
	gyroRotate.setTarget(150, 1100);
	// picks up 2nd ring
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	forwardController.setTarget(1200, 150, 1300);
	// turns and drives to line up with corner
	gyroRotate.setTarget(45, 1200);
	conveyor.moveVoltage(12000);
	forwardController.setTarget(1120, 45, 1300);
	// turns to corner
	gyroRotate.setTarget(90, 1000);
	// drives into corner
	conveyor.moveVoltage(12000);
	forwardController.setTarget(1150, 90, 1150);
	ladyBrown.moveAbsolute(280, 12000);
	wait(0.6);
	// backs out
	forwardController.setTarget(-2750, 90, 1500);
	conveyor.moveVoltage(0);
	forwardController.setTarget(-400, 94, 500);
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	ladyBrown.moveVoltage(-2000);
}

void red_2_ring_alliance() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	// tares position and heading
	gyroRotate.tareHeading();
	// lines up with alliance
	chassis.tarePosition();
	chassisProfiling(-95, 6, 0.0005, 0, 600);
	wait(0.1);
	// turns to alliance
	gyroRotate.setTarget(90, 1000);
	wait(0.1);
	// runs into wall
	chassis.tarePosition();
	chassisProfiling(-120, 6, 0.0003, 90, 530);
	// backs up a little to score on alliance
	chassis.tarePosition();
	chassisProfiling(10, 6, 0.0004, 90, 1000);
	wait(0.5);
	conveyor.moveVoltage(12000);
	wait(0.5);
	// backs off wall to turn to mogo
	intake.moveVoltage(-12000);
	conveyor.moveVoltage(0);
	chassis.tarePosition();
	chassisProfiling(50, 6, 0.0005, 90, 600);
	wait(0.1);
	// turns to mogo
	gyroRotate.setTarget(232, 1300);
	wait(0.1);
	// backs into mogo
	chassis.tarePosition();
	chassisProfiling(-250, 6, 0.0009, 232, 620);
	chassisProfiling(-260, 6, 0.0001, 232, 700);
	// clamps mogo
	clamp.openSole();
	wait(0.3);
	// turns to second ring
	gyroRotate.setTarget(380, 1300);
	wait(0.1);
	// grabs 2nd ring
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	chassis.tarePosition();
	chassisProfiling(400, 6, 0.0009, 380, 700);
	wait(0.6);
	// turns to 3rd ring
	gyroRotate.setTarget(280, 900);
	// gets 3rd ring
	wait(0.1);
	chassis.tarePosition();
	chassisProfiling(200, 6, 0.0008, 280, 400);
	// turns to ladder
	gyroRotate.setTarget(360, 900);
	wait(0.4);
	conveyor.moveVoltage(0);
	// ladyBrown.moveVoltage(7000);
	chassis.tarePosition();
	profileController->generatePath({{0_ft, 0_ft, 360_deg}, {3.167_ft, 1.167_ft, 315_deg}}, "Go_To_Tower");
	profileController->setTarget("Go_To_Tower", true);
	profileController->waitUntilSettled();
	// chassis.tarePosition();
	// chassisProfiling(250, 6, 0.0008, 315, 450);

}

void red_goal_rush() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::hold);
	// tares position and heading
	gyroRotate.tareHeading();
	chassis.tarePosition();
	// curves toward second ring and pcks it up
	intake.moveVoltage(12000);
	// (target, error, aggr, targetheading, delay, initialheading, forward?, timeout);
	curveProfiling(540, 6, 0.0004, 0, 300, 23, true, 900);
	wait(0.5);
	intake.moveVoltage(0);
	// turns to mogo
	gyroRotate.setTarget(18, 600);
	// drives to mogo
	chassis.tarePosition();
	chassisProfiling(180, 6, 0.001, 15, 400);
	// grabs mogo with doinker
	doinker.openSole();
	wait(0.5);
	// backs up
	chassis.tarePosition();
	chassisProfiling(-300, 6, 0.001, 15, 500);
	wait(0.2);
	//  lets go of goal with doinker
	doinker.closeSole();
	// backs off more
	chassis.tarePosition();
	chassisProfiling(-60, 6, 0.001, 15, 200);
	wait(0.1);
	// turns around
	gyroRotate.setTarget(184, 1200);
	conveyor.moveVoltage(-2000);
	wait(0.2);
	conveyor.moveVoltage(0);
	// backs into goal
	chassis.tarePosition();
	chassisProfiling(-150, 6, 0.001, 184, 350);
	chassisProfiling(-240, 6, 0.00011, 184, 700);
	// grabs mogo with clamp
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	clamp.openSole();
	intake.moveVoltage(-3000);
	wait(0.3);
	intake.moveVoltage(0);
	// scores 1st ring
	conveyor.moveVoltage(12000);
	chassis.tarePosition();
	chassisProfiling(260, 6, 0.0001, 184, 470);
	wait(0.5);
	// lets go of mogo
	clamp.closeSole();
	intake.moveVoltage(12000);
	conveyor.moveVoltage(0);
	// backs off to line up with 2nd mogo
	chassisProfiling(220, 6, 0.0002, 184, 420);
	wait(0.1);
	// turns to 2nd mogo
	gyroRotate.setTarget(90, 1400);
	conveyor.moveVoltage(0);
	chassis.tarePosition();
	chassisProfiling(-240, 6, 0.001, 90, 400);
	chassisProfiling(-320, 6, 0.0001, 90, 750);
	// Clamps second mogo
	clamp.openSole();
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::brake);
	ladyBrown.moveVoltage(6000);
	wait(0.4);
	ladyBrown.moveVoltage(0);
	// Scores second ring
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	// turns to touch ladder
	conveyor.moveVoltage(12000);
	gyroRotate.setTarget(128, 1200);
	// backs up into ladder
	wait(0.1);
	conveyor.moveVoltage(0);
	wait(0.4);
	chassisProfiling(-180, 6, 0.0005, 128, 500);
	intake.moveVoltage(0);
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	ladyBrown.moveVoltage(-3000);
	wait(0.5);
	ladyBrown.moveVoltage(0);
}

void blue_goal_rush() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::hold);
	// tares position and heading
	gyroRotate.tareHeading();
	// curves toward second ring and pcks it up
	intake.moveVoltage(12000);
	chassis.tarePosition();
	// (target, error, aggr, targetheading, delay, initialheading, forward?, timeout);
	curveProfiling(540, 6, 0.0004, 0, 300, -23, true, 900);
	wait(0.1);
	intake.moveVoltage(0);
	// turns to mogo
	gyroRotate.setTarget(8, 550);
	wait(0.1);
	// drives to mogo
	chassis.tarePosition();
	chassisProfiling(107, 6, 0.001, 8, 360);
	// grabs mogo with doinker
	doinker.openSole();
	wait(0.5);
	// backs up
	chassis.tarePosition();
	chassisProfiling(-300, 6, 0.001, 8, 500);
	wait(0.2);
	//  lets go of goal with doinker
	doinker.closeSole();
	// backs off more
	chassis.tarePosition();
	chassisProfiling(-60, 6, 0.001, 8, 200);
	wait(0.1);
	// turns around
	gyroRotate.setTarget(180, 1300);
	wait(0.2);
	// backs into goal
	chassis.tarePosition();
	chassisProfiling(-150, 6, 0.001, 180, 350);
	chassisProfiling(-240, 6, 0.00011, 180, 700);
	// grabs mogo with clamp
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	clamp.openSole();
	intake.moveVoltage(-3000);
	wait(0.6);
	intake.moveVoltage(0);
	// scores 1st ring
	conveyor.moveVoltage(12000);
	chassis.tarePosition();
	chassisProfiling(260, 6, 0.0001, 180, 470);
	wait(0.5);
	// lets go of mogo
	clamp.closeSole();
	intake.moveVoltage(5000);
	conveyor.moveVoltage(0);
	chassis.tarePosition();
	chassisProfiling(190, 6, 0.0001, 180, 400);
	wait(0.1);
	// turns to second mogo
	gyroRotate.setTarget(270, 1400);
	chassis.tarePosition();
	chassisProfiling(-240, 6, 0.001, 270, 400);
	chassisProfiling(-320, 6, 0.0001, 270, 750);
	// Clamps second mogo
	clamp.openSole();
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::brake);
	ladyBrown.moveVoltage(6000);
	wait(0.4);
	ladyBrown.moveVoltage(0);
	// Scores second ring
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	// turns to touch ladder
	gyroRotate.setTarget(233, 1200);
	// backs up into ladder
	wait(0.6);
	conveyor.moveVoltage(0);
	chassisProfiling(-220, 6, 0.0008, 233, 600);
	intake.moveVoltage(0);
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	ladyBrown.moveVoltage(-3000);
	wait(0.5);
	ladyBrown.moveVoltage(0);
}

void blue_awp() {
	
}

void sig_awp() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(700, 0, 280);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.5);
	// backs off
	forwardController.setTarget(-800, 0, 350);
	wait(0.25);
	ladyBrown.moveAbsolute(-200, 12000);
	wait(0.1);
	// turns to 2nd ring
	gyroRotate.setTarget(-77, 700);
	// puts intake up
	intakeLift.openSole();
	intake.moveVoltage(12000);
	wait(0.1);
	// gets 2nd ring
	forwardController.setTarget(450, -77, 600);
	intakeLift.closeSole();
	// put ladybrown down
	ladyBrown.moveAbsolute(-200, 12000);
	// turns to 1st mogo
	gyroRotate.setTarget(17, 950);
	// backs into mogo and clamps it
	forwardController.setTarget(-1700, 17, 1100);
	clamp.openSole();
	// turns to 3rd ring
	gyroRotate.setTarget(140, 800);
	conveyor.moveVoltage(12000);
	intake.moveVoltage(12000);
	// gets 3rd ring
	forwardController.setTarget(850, 140, 850);
	gyroRotate.setTarget(0, 700);
	forwardController.setTarget(970, 0, 800);
	// turns and drives to other side of field
	clamp.closeSole();
	gyroRotate.setTarget(-39, 660);
	conveyor.moveVoltage(0);
	intake.moveVoltage(-12000);
	forwardController.setTarget(2800, -39, 1500);
	// turns to 4th ring
	intake.moveVoltage(12000);
	gyroRotate.setTarget(-90, 800);
	// drops goal
	clamp.closeSole();
	// gets 4th ring
	forwardController.setTarget(1050, -90, 900);
	ladyBrown.moveAbsolute(240, 12000);
	// turns to 2nd goal and drives
	conveyor.moveVoltage(6600);
	gyroRotate.setTarget(-31, 700);
	conveyor.moveVoltage(0);
	forwardController.setTarget(-900, -30, 900);
	// clamps goal
	clamp.openSole();
	conveyor.moveVoltage(12000);
	wait(0.2);
	forwardController.setTarget(-1000, 0, 900);
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	ladyBrown.moveVoltage(-3000);
}

void left_safe_awp() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(500, 0, 280);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.5);
	// backs off
	forwardController.setTarget(-800, 0, 350);
	ladyBrown.moveVoltage(0);
	wait(0.25);
	ladyBrown.moveAbsolute(-200, 12000);
	wait(0.1);
	// turns to 2nd ring
	gyroRotate.setTarget(-68, 750);
	// puts intake up
	intakeLift.openSole();
	intake.moveVoltage(12000);
	// gets 2nd ring
	forwardController.setTarget(500, -68, 650);
	// puts intake down
	intakeLift.closeSole();
	// put ladybrown down
	ladyBrown.moveAbsolute(-200, 12000);
	// turns to mogo
	gyroRotate.setTarget(19, 1100);
	intake.moveVoltage(0);
	// backs into mogo and clamps it
	forwardController.setTarget(-1200, 19, 800);
	forwardController.setTarget(-400, 19, 300);
	clamp.openSole();
	// turns to 3rd ring
	gyroRotate.setTarget(140, 1000);
	conveyor.moveVoltage(12000);
	intake.moveVoltage(12000);
	// gets 3rd ring
	forwardController.setTarget(1200, 140, 1200);
	// turns and drives to line up with corner
	gyroRotate.setTarget(45, 1200);
	// put ladybrown up
	ladyBrown.moveAbsolute(240, 12000);
	forwardController.setTarget(1150, 45, 1100);
	// turns to corner
	gyroRotate.setTarget(98, 1200);
	// drives into corner
	conveyor.moveVoltage(12000);
	forwardController.setTarget(900, 98, 1000);
	forwardController.setTarget(500, 98, 500);
	// backs out
	forwardController.setTarget(-2850, 97, 1600);
	wait(0.3);
	conveyor.moveVoltage(0);
	intake.moveVoltage(0);
	forwardController.setTarget(-400, 97, 500);
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	ladyBrown.moveVoltage(-12000);
}

void right_safe_awp() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(505, 0, 290);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.5);
	// backs off
	forwardController.setTarget(-800, 0, 350);
	ladyBrown.moveVoltage(0);
	wait(0.25);
	ladyBrown.moveAbsolute(-200, 12000);
	wait(0.1);
	// turns to 2nd ring
	gyroRotate.setTarget(68, 750);
	// puts intake up
	intakeLift.openSole();
	intake.moveVoltage(12000);
	// gets 2nd ring
	forwardController.setTarget(500, 68, 650);
	// puts intake down
	intakeLift.closeSole();
	// put ladybrown down
	ladyBrown.moveAbsolute(-200, 12000);
	// turns to mogo
	gyroRotate.setTarget(-19, 1100);
	intake.moveVoltage(0);
	// backs into mogo and clamps it
	forwardController.setTarget(-1300, -19, 800);
	intake.moveVoltage(12000);
	forwardController.setTarget(-400, -19, 300);
	clamp.openSole();
	// turns to 3rd ring
	gyroRotate.setTarget(-140, 1000);
	conveyor.moveVoltage(12000);
	intake.moveVoltage(12000);
	// gets 3rd ring
	forwardController.setTarget(1200, -140, 1100);
	conveyor.moveVoltage(12000);
	// turns and drives to line up with corner
	gyroRotate.setTarget(-45, 1200);
	// put ladybrown up
	ladyBrown.moveAbsolute(240, 12000);
	forwardController.setTarget(1100, -45, 1100);
	// turns to corner
	gyroRotate.setTarget(-97, 1200);
	// drives into corner
	conveyor.moveVoltage(12000);
	forwardController.setTarget(900, -97, 1000);
	forwardController.setTarget(500, -97, 500);
	// backs out
	forwardController.setTarget(-2850, -93, 1600);
	wait(0.3);
	conveyor.moveVoltage(0);
	intake.moveVoltage(0);
	forwardController.setTarget(-400, -93, 500);
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	ladyBrown.moveVoltage(-12000);
}

void right_elims() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(505, 0, 290);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.5);
	// backs off
	forwardController.setTarget(-800, 0, 350);
	ladyBrown.moveVoltage(0);
	wait(0.25);
	ladyBrown.moveAbsolute(-200, 12000);
	wait(0.1);
	// turns to 2nd ring
	gyroRotate.setTarget(68, 750);
	// puts intake up
	intakeLift.openSole();
	intake.moveVoltage(12000);
	// gets 2nd ring
	forwardController.setTarget(500, 68, 650);
	// puts intake down
	intakeLift.closeSole();
	// put ladybrown down
	ladyBrown.moveAbsolute(-200, 12000);
	// turns to mogo
	gyroRotate.setTarget(-19, 1100);
	intake.moveVoltage(0);
	// backs into mogo and clamps it
	forwardController.setTarget(-1300, -19, 800);
	intake.moveVoltage(12000);
	forwardController.setTarget(-400, -19, 300);
	clamp.openSole();
	// turns to 3rd ring
	gyroRotate.setTarget(-140, 1000);
	conveyor.moveVoltage(12000);
	intake.moveVoltage(12000);
	// gets 3rd ring
	forwardController.setTarget(1200, -140, 1100);
	conveyor.moveVoltage(12000);
	// turns and drives to line up with corner
	gyroRotate.setTarget(-45, 1200);
	forwardController.setTarget(1100, -45, 1100);
	// turns to corner
	gyroRotate.setTarget(-98, 1200);
	// drives into corner
	conveyor.moveVoltage(12000);
	forwardController.setTarget(900, -98, 1100);
	forwardController.setTarget(500, -98, 500);
	// backs out
	forwardController.setTarget(-500, -98, 500);
	wait(0.2);
	intakeLift.openSole();
	wait(0.2);
	forwardController.setTarget(600, -98, 500);
	intakeLift.closeSole();
	wait(0.4);
	forwardController.setTarget(-200, -98, 200);
	
	
}

void red_6_ring() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(720, 0, 270);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.5);
	forwardController.setTarget(-800, 0, 350);
	ladyBrown.moveAbsolute(-200, 12000);
	wait(0.1);
	// turns to 2nd ring
	gyroRotate.setTarget(-76, 700);
	// puts intake up
	intakeLift.openSole();
	intake.moveVoltage(12000);
	wait(0.1);
	// gets 2nd ring
	forwardController.setTarget(450, -76, 600);
	intakeLift.closeSole();
	// put ladybrown down
	ladyBrown.moveAbsolute(-200, 12000);
	// turns to mogo
	gyroRotate.setTarget(19, 1000);
	// backs into mogo and clamps it
	forwardController.setTarget(-1300, 19, 800);
	forwardController.setTarget(-300, 19, 350);
	clamp.openSole();
	// turns to line up with ring stack
	gyroRotate.setTarget(185, 800);
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	wait(0.5);
	// drives to get parallel to middle
	forwardController.setTarget(945, 185, 695);
	wait(0.4);
	// turns parallel to middle
	forwardController.setTarget(450, 130, 400);
	gyroRotate.setTarget(129, 550);
	// gets 3rd and 4th ring
	forwardController.setTarget(725, 130, 750);
	// turns to 5th
	rDrive.moveVoltage(12000);
	wait(0.15);
	gyroRotate.setTarget(20, 700);
	// grabs 5th
	forwardController.setTarget(800, 20, 800);
	// turns to corner
	gyroRotate.setTarget(70, 700);
	// gets 6th ring
	forwardController.setTarget(1000, 70, 800);
	forwardController.setTarget(1000, 97, 1000);
	forwardController.setTarget(-1000, 97, 1000);
}

void blue_6_ring() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(720, 0, 270);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.5);
	forwardController.setTarget(-800, 0, 350);
	ladyBrown.moveAbsolute(-200, 12000);
	wait(0.1);
	// turns to 2nd ring
	gyroRotate.setTarget(76, 700);
	// puts intake up
	intakeLift.openSole();
	intake.moveVoltage(12000);
	wait(0.1);
	// gets 2nd ring
	forwardController.setTarget(450, 76, 600);
	intakeLift.closeSole();
	// put ladybrown down
	ladyBrown.moveAbsolute(-180, 12000);
	// turns to mogo
	gyroRotate.setTarget(-17, 1000);
	// backs into mogo and clamps it
	forwardController.setTarget(-1300, -17, 800);
	forwardController.setTarget(-300, -17, 350);
	clamp.openSole();
	// turns to line up with ring stack
	gyroRotate.setTarget(-185, 800);
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	wait(0.5);
	// drives to get parallel to middle
	forwardController.setTarget(945, -185, 695);
	wait(0.4);
	// turns parallel to middle
	forwardController.setTarget(450, -130, 400);
	gyroRotate.setTarget(-130, 550);
	// gets 3rd and 4th ring
	forwardController.setTarget(690, -130, 750);
	// turns to 5th
	lDrive.moveVoltage(12000);
	wait(0.15);
	gyroRotate.setTarget(-20, 600);
	// grabs 5th
	forwardController.setTarget(800, -20, 800);
	// turns to corner
	gyroRotate.setTarget(-70, 700);
	// gets 6th ring
	forwardController.setTarget(1000, -70, 750);
	forwardController.setTarget(1200, -97, 1000);
	forwardController.setTarget(-1000, -97, 1000);
}

void skillsAuto() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance with ladybrown
	ladyBrown.moveVoltage(12000);
	wait(0.7);
	ladyBrown.moveVoltage(0);
	// backs off alliance into mogo
	forwardController.setTarget(-900, 0, 900);
	// puts ladybrown down
	ladyBrown.moveAbsolute(-200, 12000);

	// mogo #1
	// clamps mogo
	clamp.openSole();
	// turns to 1st ring
	gyroRotate.setTarget(135, 900);
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	// drives and picks up 1st ring
	forwardController.setTarget(900, 135, 900);
	

}

// tasks
// conveyor
void ejectRing() {
	// pros::lcd::print(5, "Eject   ");
	// stores original mode
	ConveyorMode originalMode = conveyorMode;
	int originalVoltage = conveyor.getVoltage();
	// sets mode to eject
	conveyorMode = Eject;
	conveyor.moveVoltage(12000);

	while (!ringLimit.changedToReleased()) {
		// failsafe
		if (l1.isPressed() || conveyor.getVoltage() == 0) {
			conveyorMode = User;
			return;
		}
		pros::delay(20);
	}

	// reverses conveyor to throw ring
	conveyor.moveVoltage(-12000);
	pros::delay(200);
	// sets conveyorMode back to the mode it was before it was switched to eject mode
	conveyorMode = originalMode;
	if (conveyorMode == Auto) {
		conveyor.moveVoltage(originalVoltage);
	}
}

bool leftJustPressed = true;
// conveyor and color sorter thread
void conveyorTask(void *parems) {
	while (true) {
		if (conveyorMode == User) {
			// pros::lcd::print(5, "User   ");
			if (r1.isPressed()) {
				// conveyor forward
				conveyor.moveVoltage(12000);
				colorSensorEnabled = true;
			} else if (l1.isPressed()) {
				// conveyor backward
				conveyor.moveVoltage(-12000);
				colorSensorEnabled = false;
			} else {
				conveyor.moveVoltage(0);
			}
		}
		if (conveyorMode == Auto) {
			// pros::lcd::print(5, "Auto   ");
		}

		if (left.isPressed()) {
			if (leftJustPressed) {
				colorSensing = !colorSensing;
				leftJustPressed = false;
				controller1.rumble(".");
			}
		} else {
			leftJustPressed = true;
		}

		if (colorSensing && colorSensorEnabled && conveyorMode != Eject) {
			// determine if the robot should eject
			// optical sensor color
			double opticalSensorHue = opticalSensor.get_hue();
			// red: 0-20
			// blue: 180-220
			if (opticalSensorHue >= 0 && opticalSensorHue <= 18 && teamColor == "blue") {
				// ring is red
				ejectRing();
			} else if (opticalSensorHue >= 210 && opticalSensorHue <= 232 && teamColor == "red") {
				// ring is blue
				ejectRing();
			}
		}
		pros::delay(20);
	}
}

// ladyBrown task and functions
int targetAngle;
std::string ladyBrownTarget;
void setLadyBrown(std::string position) {
	ladyBrownMode = Set;
	ladyBrownTarget = position;

	if ("load") {
		targetAngle = 11890;
	} else if ("down") {
		targetAngle = 14800;
	} else if ("alliance") {
		targetAngle = 32200;
	} else {
		// failsafe
		ladyBrownMode = Manual;
	}
}

// ladybrown task
void ladyBrownTask(void *parems) {
	ladyBrownMode = Manual;
	float speedMultiplier = 2.5;
	int ladyBrownSpeed;

	while (true) {
		// gets ladybrown rotation
		brownRotation = rotationSensor.get_angle();

		// manual mode
		if (ladyBrownMode == Manual) {
			
		}

		// set mode
		if (ladyBrownMode == Set) {
			// determines speed based off how far the current angle is from the target angle
			// checks to make find out if the rotation is past the point of position reset
			if (brownRotation > 16000) {
				ladyBrownSpeed = (36000 - brownRotation + targetAngle) * -speedMultiplier;
			} else {
				// if before 16000 and after 36000
				ladyBrownSpeed = (brownRotation - targetAngle) * speedMultiplier;
			}
			// makes sure voltage is less than or equal to 12 volts
			ladyBrownSpeed = std::max(std::min(ladyBrownSpeed, 12000), -12000);
			// error for ladybrown's position vs target position 
			int error = 5;
			// if between max and min angles, then move the lady brown. If within the two angles, stop the lady brown
			if (brownRotation < targetAngle - error || brownRotation > targetAngle + error) {
				ladyBrown.moveVoltage(ladyBrownSpeed);
			} else if (ladyBrownTarget == "down") {
				ladyBrown.moveVoltage(0);
			}
		}

		// any mode 
		// finds the right joystick's y axis value 
		float controllerYAxis = controller1.getAnalog(ControllerAnalog::rightY);
		// deadzone for joystick to avoid accidentally raising ladybrown while driving
		float controllerDeadZone = 0.85;
		if (controllerYAxis > controllerDeadZone) {
			// if joystick is pointed up move ladybrown forward
			ladyBrown.moveVoltage(12000);
			ladyBrownMode = Manual;
		} else if (controllerYAxis < -controllerDeadZone) {
			// if joystick is pointed down move ladybrown backward
			ladyBrown.moveVoltage(-12000);
			ladyBrownMode = Manual;
		} else if (ladyBrownMode == Manual) {
			ladyBrown.moveVoltage(0);
		}

		if (y.isPressed()) {
			// loading position for ladybrown
			setLadyBrown("load");
			ladyBrownMode = Set;
		} else if (right.isPressed()) {
			// puts ladybrown down
			setLadyBrown("down");
			ladyBrownMode = Set;
		}
		pros::delay(20);
	}
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	// displayInit();
	// teamColor = "red";
	gyroRotate.tareHeading();

	controller1.rumble("--..");
	// initializes conveyor/color sensing task
	// turns on optical light
	if (colorSensing) {
		opticalSensor.set_led_pwm(100);
	}
	pros::Task ConveyorTask(conveyorTask, NULL);
	pros::Task LadyBrownTask(ladyBrownTask, NULL);
}

bool grabGoal = false;
void autonomous() {
	drive->getModel()->setBrakeMode(AbstractMotor::brakeMode::brake);
	conveyorMode = Auto;
	runSelectedAuto();
	// right_safe_awp();
	// left_safe_awp();
	// skillsAuto();
	// red_goal_rush();
	// red_2_ring_alliance();
	// sig_awp();
	// red_6_ring();
	// blue_5_ring();
	// blue_6_ring();
	// blue_3_ring();

	// drive PID tuning
	// forwardController.setTarget(2000, 90, 30000);s

	// rotation PID tuning
	// gyroRotate.tareHeading();
	// gyroRotate.rotateAbsolute(90);

	grabGoal = true;
}

void solenoidCheck(Solenoid& sol, ControllerButton button, bool& buttonJustPressed, bool& solBool) {
	if (button.isPressed()) {
		if (buttonJustPressed) {
			solBool = !solBool;
			buttonJustPressed = false;
		}
	} else {
		buttonJustPressed = true;
	}

	if (solBool) {
		sol.openSole();
	} else {
		sol.closeSole();
	}

}

void opcontrol() {
	colorSensing = true;
	conveyorMode = User;
	drive->getModel()->setBrakeMode(AbstractMotor::brakeMode::coast);
	drive->getModel()->setMaxVoltage(12000);
	
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::brake);

	pros::screen::set_pen(COLOR_BLUE);

	bool doinkerDown = false;
	bool mogoClose = false;
	bool intakeDown = false;

	// lady brown toggle conveyorMode to set in grab ring position
	bool setBrown = false;
	bool downBrown = false;
	
	bool aJustPressed = true;
	bool xJustPressed = true;
	bool bJustPressed = true;

   	while (true) {
		pros::lcd::print(1, "color: %f   ", opticalSensor.get_hue());
		pros::lcd::print(2, "rotation: %i   ", brownRotation);
		pros::lcd::print(3, "Controller right Y: %f   ", controller1.getAnalog(ControllerAnalog::rightY));
 		// chassis
		drive->getModel()->arcade(controller1.getAnalog(ControllerAnalog::leftY), controller1.getAnalog(ControllerAnalog::rightX));
		// // distance sensor
		// float distanceValue = distanceSensor.get();
		// // pros::screen::print(TEXT_SMALL, 5, "Distance: %lf", distanceValue);
		// if (distanceValue <= 21) {
		// 	if (mogoClose == false) {
		// 		grabGoal = true;
		// 	}
		// 	mogoClose = true;
		// } else {
		// 	mogoClose = false;
		// }


		// clamp 
		solenoidCheck(clamp, a, aJustPressed, grabGoal);
		// doinker
		solenoidCheck(doinker, b, bJustPressed, doinkerDown);
		// intake lift
		solenoidCheck(intakeLift, x, xJustPressed, intakeDown);

		// Intake
		if (r2.isPressed()) {
			intake.moveVoltage(12000);
		} else if (l2.isPressed()) {
			intake.moveVoltage(-12000);
		} else {
			intake.moveVoltage(0);
		}

		// // ladyBrown
		// if (up.isPressed()) {
		// 	ladyBrown.moveVoltage(12000);
		// 	setBrown = false;
		// 	downBrown = false;
		// } else if (down.isPressed()) {
		// 	ladyBrown.moveVoltage(-12000);
		// 	setBrown = false;
		// 	downBrown = false;
		// } else if (!setBrown && !downBrown) {
		// 	// ladyBrown.moveVoltage(0);
		// }

		float controllerYAxis = controller1.getAnalog(ControllerAnalog::rightY);
		float controllerDeadZone = 0.85;
		if (controllerYAxis > controllerDeadZone) {
			ladyBrown.moveVoltage(12000);
			setBrown = false;
			downBrown = false;
		} else if (controllerYAxis < -controllerDeadZone) {
			ladyBrown.moveVoltage(-12000);
			setBrown = false;
			downBrown = false;
		} else if (!setBrown && !downBrown) {
			ladyBrown.moveVoltage(0);
		}
		
		brownRotation = rotationSensor.get_angle();
		if (y.isPressed()) {
			setBrown = true;
			downBrown = false;
		} else if (right.isPressed()) {
			downBrown = true;
			setBrown = false;
		}

		// when toggled into gab ring conveyorMode, it'll try to rotate the lady brown into a range within 2650-2700
		if (setBrown) {
			int targetAngle = 11890;
			float speedMultiplier = 2.5;
			int ladyBrownSpeed;
			// determines speed based off how far the current angle is from the target angle
			if (brownRotation > 16000) {
				ladyBrownSpeed = (36000 - brownRotation + targetAngle) * -speedMultiplier;
			} else {
				ladyBrownSpeed = (brownRotation - targetAngle) * speedMultiplier;
			}
			// pros::screen::print(TEXT_SMALL, 6, "rotation speed: %i", ladyBrownSpeed);
			// makes sure voltage is less than or equal to 12 volts
			ladyBrownSpeed = std::max(std::min(ladyBrownSpeed, 12000), -12000);
			// gravity control
			int error = 5;
			// if between max and min angles, then move the lady brown. If within the two angles, stop the lady brown
			if (brownRotation < targetAngle - error || brownRotation > targetAngle + error) {
				ladyBrown.moveVoltage(ladyBrownSpeed);
			}
		} else if (downBrown) {
			int targetAngle = 14800;
			float speedMultiplier = 2.5;
			int ladyBrownSpeed;
			// determines speed based off how far the current angle is from the target angle
			if (brownRotation > 16000) {
				ladyBrownSpeed = (36000 - brownRotation + targetAngle) * -speedMultiplier;
			} else {
				ladyBrownSpeed = (brownRotation - targetAngle) * speedMultiplier;
			}
			// pros::screen::print(TEXT_SMALL, 6, "rotation speed: %i", ladyBrownSpeed);
			// makes sure voltage is less than or equal to 12 volts
			ladyBrownSpeed = std::max(std::min(ladyBrownSpeed, 12000), -12000);
			// if between max and min angles, then move the lady brown. If within the two angles, stop the lady brown
			if (brownRotation < targetAngle || brownRotation > 16000) {
				ladyBrown.moveVoltage(ladyBrownSpeed);
			} else {
				downBrown = false;
				ladyBrown.moveVoltage(0);
			}
		}
    	pros::delay(20);
 	}
}