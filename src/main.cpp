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

MotorGroup rDrive ({10, 9, 20});
MotorGroup lDrive ({-1, -2, -6});

Motor intake (12, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor conveyor (11, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor ladyBrown (13,true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

Solenoid clamp('A');
Solenoid rightDoinker('C');
Solenoid intakeLift('D');
Solenoid leftDoinker('E');
Solenoid pusher('G');
Solenoid doinkerClamp('H');

okapi::ADIButton ringLimit('F');

pros::Optical opticalSensor(18);
pros::Rotation rotationSensor(19);

bool colorSensing = true;
bool colorSensorEnabled = true;
bool jamDetection = true;
bool grabGoal = false;

enum ConveyorMode: int {
	User,
	Auto,
	Eject,
};

ConveyorMode conveyorMode = Auto;

enum LadyBrownMode: int {
	Manual,
	Auton,
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
	0.0082,     // 0.0082  // increase p until overshoots and then just undershoots (power)   0.015
 	0.01/1000, //0.01/1000;      // increase i until under a degree off either way
 	0.0023/100 //0.0023/100;       // increase d until only undershoots (dampening) too high of d = overshoots (sine curve) 0.05/100
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
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	leftDoinker.closeSole();
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

// tasks
// conveyor
int ejections = 0;
void ejectRing() {
	// stores original mode
	ejections += 1;
	pros::lcd::print(5, "Eject  ");
	pros::lcd::print(6, "Ejections: %i", ejections);
	ConveyorMode originalMode = conveyorMode;
	int originalVoltage = conveyor.getVoltage();
	// sets mode to eject
	conveyorMode = Eject;
	conveyor.moveVoltage(12000);

	while (!ringLimit.changedToReleased()) {
		// failsafe
		if (l1.isPressed()) {
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
	pros::lcd::print(5, "done  ");
}

bool leftJustPressed = true;
// conveyor and color sorter thread
void conveyorTask(void *parems) {
	while (true) {

		if (colorSensing) {
			opticalSensor.set_led_pwm(100);
		} else {
			opticalSensor.set_led_pwm(0);
		}

		if (conveyorMode == User) {
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
			if (opticalSensorHue >= 0 && opticalSensorHue <= 20 && teamColor == "blue" && opticalSensor.get_proximity() >= 254) {
				// ring is red
				ejectRing();
			} else if (opticalSensorHue >= 210 && opticalSensorHue <= 240 && teamColor == "red" && opticalSensor.get_proximity() >= 254) {
				// ring is blue
				ejectRing();
			}
		}
		pros::delay(20);
	}
}

int ladyBrownTimeOut = 0;
bool delayStopConveyor = false;
std::string ladyBrownTarget;
void setLadyBrownDelayed(std::string position, int timeOut, bool stopConveyor = false) {
	ladyBrownMode = Auton;
	ladyBrownTarget = position;
	ladyBrownTimeOut = timeOut;
	if (stopConveyor) {
		delayStopConveyor = true;
	}
}

// ladyBrown task and functions
int targetAngle;
void setLadyBrown(std::string position) {
	ladyBrownMode = Set;
	ladyBrownTarget = position;

	if (position == "load") {
		targetAngle = 11400;
	} else if (position == "down") {
		targetAngle = 7600;
	} else if (position == "alliance") {
		targetAngle = 10000;
	} else if (position == "wallScored") {
		targetAngle = 21000;
	} else if (position == "touchLadder") {
		targetAngle = 15000;
	} else if (position == "descore6th") {
		targetAngle = 23500;
	} else if (position == "up") {
		targetAngle = 23000; 
	}	else {
		// failsafe
		ladyBrownMode = Manual;
	}
}
// ladybrown task
void ladyBrownTask(void *parems) {
	float speedMultiplier = 2.4;
	int ladyBrownSpeed;

	while (true) {
		// gets ladybrown rotation
		brownRotation = rotationSensor.get_angle();

		// manual mode
		if (ladyBrownMode == Manual) {
			
		}
		if (ladyBrownMode == Auton) {
			if (ladyBrownTimeOut > 0) {
				pros::delay(ladyBrownTimeOut);
				ladyBrownTimeOut = 0;
				setLadyBrown(ladyBrownTarget);
				if (delayStopConveyor) {
					conveyor.moveVoltage(0);
					delayStopConveyor = false;
				}
			}
		}

		// set mode
		if (ladyBrownMode == Set) {
			// determines speed based off how far the current angle is from the target angle
			// checks to make find out if the rotation is past the point of position reset
			
			ladyBrownSpeed = (brownRotation - targetAngle) * -speedMultiplier;
			// makes sure voltage is less than or equal to 12 volts
			ladyBrownSpeed = std::max(std::min(ladyBrownSpeed, 12000), -12000);
			// error for ladybrown's position vs target position
			int error = 5;
			// gravity control
			ladyBrownSpeed += 425;
			// if between max and min angles, then move the lady brown. If within the two angles, stop the lady brown
			if (brownRotation < targetAngle - error || brownRotation > targetAngle + error) {
				ladyBrown.moveVoltage(ladyBrownSpeed);
			} else if (ladyBrownTarget == "down") {
				ladyBrown.moveVoltage(0);
				ladyBrownMode = Manual;
			}
		}

		// any mode 
		// finds the right joystick's y axis value 
		float controllerYAxis = controller1.getAnalog(ControllerAnalog::rightY);
		// deadzone for joystick to avoid accidentally raising ladybrown while driving
		float controllerDeadZone = 0.9;
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
		}
		
		if (right.isPressed()) {
			// puts ladybrown down
			setLadyBrown("down");
			ladyBrownMode = Set;
		}
		
		if (down.isPressed()) {
			setLadyBrown("descore6th");
		}

		int pusherMin = 11600;
		int pusherMax = 20000;
		if (brownRotation <= pusherMax && brownRotation >= pusherMin) {
			pusher.openSole();
		} else {
			pusher.closeSole();
		}

		pros::delay(20);
	}
}

void setLadyBrownVoltage(int voltage) {
	ladyBrown.moveVoltage(voltage);
	ladyBrownMode = Auton;
}

void jamDetectionTask(void *parems) {
	while (true) {
		if (jamDetection) {
			// pros::lcd::print(4, "Target Velocity %d", conveyor.getTargetVelocity());
			// pros::lcd::print(5, "Actual Velocity %d", conveyor.getActualVelocity());
			if (conveyor.getActualVelocity() < 1 && conveyor.getTargetVelocity() > 300) {
				wait(0.1);
				// second check
				if (conveyor.getActualVelocity() < 1 && conveyor.getTargetVelocity() > 300 && jamDetection) {
					pros::lcd::print(6, "jammed    ");
					conveyor.moveVoltage(-12000);
					wait(0.2);
					if (conveyor.getTargetVelocity() == -600) {
						conveyor.moveVoltage(12000);
					}
				}
			} else {
				pros::lcd::print(6, "not jammed");
			}
		}
		pros::delay(20);
	}
}

void getTwoCornerRings(int angle) {
	// gets bottom two rings in stack
	intakeLift.openSole();
	jamDetection = false;
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	// drives into corner
	forwardController.setTarget(1300, angle, 1, 700);
	intakeLift.closeSole();
	wait(0.5);
	forwardController.setTarget(1200, angle, 1, 400);
	// gets 2nd blue ring
	forwardController.setTarget(-600, angle, 1, 450);
	forwardController.setTarget(700, angle, 1, 750);
}

void red_3_ring() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(700, 0, 1, 250);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveAbsolute(750, 12000);
	wait(0.6);
	// backs off
	forwardController.setTarget(-1000, 0, 1, 600);
	// turns to mogo
	gyroRotate.setTarget(-46, 1000);
	// put ladybrown down
	ladyBrown.moveAbsolute(-200, 12000);
	// backs into mogo and clamps it 
	forwardController.setTarget(-1000, -46, 1, 600);
	forwardController.setTarget(-250, -46, 1, 500);
	clamp.openSole();
	wait(0.1);
	// turns to 2nd ring
	gyroRotate.setTarget(-146, 1100);
	// picks up 2nd ring
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	forwardController.setTarget(1200, -146, 1, 1300);
	// turns and drives to line up with corner
	gyroRotate.setTarget(-45, 1200);
	conveyor.moveVoltage(12000);
	forwardController.setTarget(1120, -45, 1, 1300);
	// turns to corner
	gyroRotate.setTarget(-93, 1400);
	// drives into corner
	conveyor.moveVoltage(12000);
	forwardController.setTarget(1150, -93, 1, 1200);
	ladyBrown.moveAbsolute(280, 12000);
	wait(0.6);
	// backs out
	forwardController.setTarget(-2750, -87, 1, 1500);
	conveyor.moveVoltage(0);
	forwardController.setTarget(-400, -80, 1, 500);
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
	forwardController.setTarget(700, 0, 1, 250);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveAbsolute(750, 12000);
	wait(0.5);
	// backs off
	forwardController.setTarget(-1000, 0, 1, 600);
	// turns to mogo
	gyroRotate.setTarget(46, 900);
	// put ladybrown down
	ladyBrown.moveAbsolute(-200, 12000);
	// backs into mogo and clamps it
	forwardController.setTarget(-1200, 46, 1, 800);
	forwardController.setTarget(-250, 46, 1, 450);
	clamp.openSole();
	wait(0.1);
	// turns to 2nd ring
	gyroRotate.setTarget(150, 1100);
	// picks up 2nd ring
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	forwardController.setTarget(1200, 150, 1, 1300);
	// turns and drives to line up with corner
	gyroRotate.setTarget(45, 1200);
	conveyor.moveVoltage(12000);
	forwardController.setTarget(1120, 45, 1, 1300);
	// turns to corner
	gyroRotate.setTarget(90, 1000);
	// drives into corner
	conveyor.moveVoltage(12000);
	forwardController.setTarget(1150, 90, 1, 1150);
	ladyBrown.moveAbsolute(280, 12000);
	wait(0.6);
	// backs out
	forwardController.setTarget(-2750, 90, 1, 1500);
	conveyor.moveVoltage(0);
	forwardController.setTarget(-400, 94, 1, 500);
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	ladyBrown.moveVoltage(-2000);
}

void blueSig() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(700, 0, 1, 230);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.5);
	// backs off
	forwardController.setTarget(-800, 0, 1, 600);
	setLadyBrown("down");
	// turns to mogo
	gyroRotate.setTarget(-60, 300);
	gyroRotate.setTarget(-40, 600);
	// backs into mogo and clamps it
	forwardController.setTarget(-1300, -40, 1, 1100);
	clamp.openSole();
	// turns to 2nd ring
	gyroRotate.setTarget(-140, 800);
	conveyor.moveVoltage(12000);
	intake.moveVoltage(12000);
	// gets 2nd ring
	forwardController.setTarget(850, -140, 1, 850);
	// turns and drives to line up with 3rd ring
	gyroRotate.setTarget(-10, 700);
	forwardController.setTarget(1450, -10, 1, 1000);
	// turns and drives to other side of field
	gyroRotate.setTarget(46, 700);
	conveyor.moveVoltage(12000);
	intake.moveVoltage(12000);
	forwardController.setTarget(2700, 46, 0.65, 1500);
	// turns to 4th ring
	intake.moveVoltage(12000);
	gyroRotate.setTarget(100, 800);
	wait(0.1);
	// drops goal
	clamp.closeSole();
	// gets 4th ring
	forwardController.setTarget(900, 100, 1, 700);
	setLadyBrown("touchLadder");
	// turns to 2nd goal and drives
	conveyor.moveVoltage(4500);
	gyroRotate.setTarget(31, 700);
	conveyor.moveVoltage(0);
	forwardController.setTarget(-950, 30, 1, 700);
	// clamps goal
	clamp.openSole();
	conveyor.moveVoltage(12000);
	wait(0.4);
	// touches bar
	forwardController.setTarget(-1000, 0, 0.8, 900);
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	setLadyBrown("down");
}

void redSig() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(700, 0, 1, 230);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.5);
	// backs off
	forwardController.setTarget(-800, 0, 1, 600);
	setLadyBrown("down");
	// turns to mogo
	gyroRotate.setTarget(60, 300);
	gyroRotate.setTarget(40, 600);
	// backs into mogo and clamps it
	forwardController.setTarget(-1200, 40, 1, 1100);
	clamp.openSole();
	// turns to 2nd ring
	gyroRotate.setTarget(140, 800);
	conveyor.moveVoltage(12000);
	intake.moveVoltage(12000);
	// gets 2nd ring
	forwardController.setTarget(850, 140, 1, 850);
	// turns and drives to line up with 3rd ring
	gyroRotate.setTarget(10, 700);
	forwardController.setTarget(1450, 10, 1, 1000);
	// turns and drives to other side of field
	gyroRotate.setTarget(-46, 700);
	conveyor.moveVoltage(12000);
	intake.moveVoltage(12000);
	forwardController.setTarget(2700, -46, 0.65, 1500);
	// turns to 4th ring
	intake.moveVoltage(12000);
	gyroRotate.setTarget(-100, 800);
	wait(0.1);
	// drops goal
	clamp.closeSole();
	// gets 4th ring
	forwardController.setTarget(1050, -100, 1, 900);
	setLadyBrown("touchLadder");
	// turns to 2nd goal and drives
	conveyor.moveVoltage(4500);
	gyroRotate.setTarget(-31, 700);
	conveyor.moveVoltage(0);
	forwardController.setTarget(-950, -31, 0.8, 700);
	// clamps goal
	clamp.openSole();
	conveyor.moveVoltage(12000);
	wait(0.4);
	// touches bar
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	forwardController.setTarget(-1050, 20, 0.8, 750);
	setLadyBrown("down");
}

void left_safe_awp() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(500, 0, 1, 280);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.5);
	// backs off
	forwardController.setTarget(-800, 0, 1, 350);
	ladyBrown.moveVoltage(0);
	wait(0.25);
	ladyBrown.moveAbsolute(-200, 12000);
	wait(0.1);
	// turns to 2nd ring
	gyroRotate.setTarget(-65, 750);
	// puts intake up
	intakeLift.openSole();
	intake.moveVoltage(12000);
	// gets 2nd ring
	forwardController.setTarget(600, -65, 1, 650);
	// puts intake down
	intakeLift.closeSole();
	// put ladybrown down
	ladyBrown.moveAbsolute(-200, 12000);
	// turns to mogo
	gyroRotate.setTarget(20, 1100);
	intake.moveVoltage(0);
	// backs into mogo and clamps it
	forwardController.setTarget(-1200, 20, 1, 800);
	forwardController.setTarget(-400, 20, 1, 300);
	clamp.openSole();
	// turns to 3rd ring
	gyroRotate.setTarget(140, 1000);
	conveyor.moveVoltage(12000);
	intake.moveVoltage(12000);
	// gets 3rd ring
	forwardController.setTarget(1400, 140, 1, 1400);
	conveyor.moveVoltage(12000);
	// turns and drives to line up with corner
	gyroRotate.setTarget(47, 1200);
	colorSensing = true;
	// put ladybrown up
	ladyBrown.moveAbsolute(240, 12000);
	// drive before corner
	forwardController.setTarget(1300, 47, 1, 1200);
	// turns to corner
	gyroRotate.setTarget(100, 1200);
	// drives into corner
	conveyor.moveVoltage(12000);
	forwardController.setTarget(1000, 100, 1, 1000);
	forwardController.setTarget(800, 100, 1, 500);
	conveyor.moveVoltage(12000);
	// backs out
	forwardController.setTarget(-2600, 93, 1, 1600);
	wait(0.3);
	conveyor.moveVoltage(0);
	intake.moveVoltage(0);
	forwardController.setTarget(-600, 93, 1, 400);
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	ladyBrown.moveVoltage(-12000);
	wait(0.5);
	ladyBrown.moveVoltage(0);
}

void right_safe_awp() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(505, 0, 1, 290);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.5);
	// backs off
	forwardController.setTarget(-800, 0, 1, 350);
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
	forwardController.setTarget(600, 68, 1, 600);
	// puts intake down
	intakeLift.closeSole();
	// put ladybrown down
	ladyBrown.moveAbsolute(-200, 12000);
	// turns to mogo
	gyroRotate.setTarget(-19, 1100);
	intake.moveVoltage(0);
	// backs into mogo and clamps it
	forwardController.setTarget(-1300, -19, 1, 800);
	intake.moveVoltage(12000);
	forwardController.setTarget(-400, -19, 1, 300);
	clamp.openSole();
	// turns to 3rd ring
	gyroRotate.setTarget(-135, 1000);
	conveyor.moveVoltage(12000);
	intake.moveVoltage(12000);
	// gets 3rd ring
	forwardController.setTarget(1200, -140, 1, 1200);
	conveyor.moveVoltage(12000);
	// turns and drives to line up with corner
	gyroRotate.setTarget(-45, 1200);
	colorSensing = true;
	// put ladybrown up
	ladyBrown.moveAbsolute(240, 12000);
	forwardController.setTarget(1250, -45, 1, 1100);
	// turns to corner
	gyroRotate.setTarget(-98, 1100);
	// drives into corner
	conveyor.moveVoltage(12000);
	forwardController.setTarget(800, -98, 1, 800);
	forwardController.setTarget(800, -98, 1, 900);
	// backs out
	forwardController.setTarget(-2800, -90, 1, 1600);
	wait(0.3);
	conveyor.moveVoltage(0);
	intake.moveVoltage(0);
	forwardController.setTarget(-400, -89, 1, 350);
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	ladyBrown.moveVoltage(-6000);
	wait(0.5);
	ladyBrown.moveVoltage(0);
}

void right_elims() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(505, 0, 1, 290);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.5);
	// backs off
	forwardController.setTarget(-800, 0, 1, 350);
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
	forwardController.setTarget(500, 68, 1, 650);
	// puts intake down
	intakeLift.closeSole();
	// put ladybrown down
	ladyBrown.moveAbsolute(-200, 12000);
	// turns to mogo
	gyroRotate.setTarget(-19, 1100);
	intake.moveVoltage(0);
	// backs into mogo and clamps it
	forwardController.setTarget(-1300, -19, 1, 800);
	intake.moveVoltage(12000);
	forwardController.setTarget(-400, -19, 1, 300);
	clamp.openSole();
	// turns to 3rd ring
	gyroRotate.setTarget(-140, 1000);
	conveyor.moveVoltage(12000);
	intake.moveVoltage(12000);
	// gets 3rd ring
	forwardController.setTarget(1200, -140, 1, 1100);
	conveyor.moveVoltage(12000);
	// turns and drives to line up with corner
	gyroRotate.setTarget(-45, 1200);
	forwardController.setTarget(1100, -45, 1, 1100);
	// turns to corner
	gyroRotate.setTarget(-98, 1200);
	// drives into corner
	conveyor.moveVoltage(12000);
	forwardController.setTarget(900, -98, 1, 1100);
	forwardController.setTarget(500, -98, 1, 500);
	// backs out
	forwardController.setTarget(-500, -98, 1, 500);
	wait(0.2);
	intakeLift.openSole();
	wait(0.2);
	forwardController.setTarget(600, -98, 1, 500);
	intakeLift.closeSole();
	wait(0.4);
	forwardController.setTarget(-200, -98, 1, 200);
	
	
}

void red_6_ring() {
	colorSensing = false;
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(505, 0, 1, 290);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.6);
	// backs off
	forwardController.setTarget(-800, 0, 1, 350);
	setLadyBrown("down");
	wait(0.1);
	// turns to 2nd ring
	gyroRotate.setTarget(-65, 800);
	// puts intake up
	intakeLift.openSole();
	intake.moveVoltage(12000);
	wait(0.1);
	// gets 2nd ring
	forwardController.setTarget(600, -65, 1, 600);
	intakeLift.closeSole();
	// turns to mogo
	gyroRotate.setTarget(19, 1000);
	// backs into mogo and clamps it
	forwardController.setTarget(-1300, 19, 1, 800);
	forwardController.setTarget(-300, 19, 1, 350);
	clamp.openSole();
	// turns to line up with ring stack
	gyroRotate.setTarget(177, 800);
	colorSensing = true;
	// scores on mogo
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	// wait(0.2);
	// drives to get parallel to middle
	forwardController.setTarget(950, 177, 1, 750);
	conveyor.moveVoltage(12000);
	wait(0.2);
	// turns parallel to middle
	forwardController.setTarget(500, 130, 1, 400);
	// gyroRotate.setTarget(129, 360);
	conveyor.moveVoltage(12000);
	// gets 3rd and 4th ring
	forwardController.setTarget(650, 130, 1, 600);
	// turns to 5th
	rDrive.moveVoltage(12000);
	wait(0.3);
	gyroRotate.setTarget(20, 700);
	// grabs 5th
	forwardController.setTarget(800, 20, 1, 800);
	conveyor.moveVoltage(12000);
	// turns to line up with corner
	gyroRotate.setTarget(66, 700);
	forwardController.setTarget(1300, 66, 1, 1000);
	conveyor.moveVoltage(12000);
	// turns to corner
	gyroRotate.setTarget(95, 650);
	conveyor.moveVoltage(12000);
	// gets 6th ring
	forwardController.setTarget(550, 95, 1, 400);
	forwardController.setTarget(900, 95, 0.65, 800);
	conveyor.moveVoltage(12000);
	forwardController.setTarget(-800, 95, 0.7, 500);
	conveyor.moveVoltage(12000);
	gyroRotate.setTarget(50, 1000);
}

void red6AWP() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(505, 0, 1, 290);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.6);
	// backs off
	forwardController.setTarget(-500, 0, 1, 500);
	setLadyBrown("down");
	// puts intake up
	intakeLift.openSole();
	// turns to 2nd ring
	gyroRotate.setTarget(-65, 650);
	intake.moveVoltage(12000);
	// gets 2nd ring
	forwardController.setTarget(700, -65, 1, 600);
	intakeLift.closeSole();
	// turns to mogo
	gyroRotate.setTarget(13, 700);
	// backs into mogo and clamps it
	forwardController.setTarget(-1300, 13, 1, 800);
	forwardController.setTarget(-300, 13, 1, 350);
	clamp.openSole();
	// turns to line up with ring stack
	gyroRotate.setTarget(182, 800);
	// scores on mogo
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	// wait(0.2);
	// drives to get parallel to middle
	forwardController.setTarget(1100, 182, 1, 800);
	// turns parallel to middle
	forwardController.setTarget(500, 130, 1, 400);
	// gets 3rd and 4th ring
	forwardController.setTarget(650, 130, 1, 550);
	// turns to back up
	gyroRotate.setTarget(160, 600);
	forwardController.setTarget(-600, 160, 1, 500);
	// turns to 5th ring
	gyroRotate.setTarget(85, 600);
	// grabs 5th
	forwardController.setTarget(800, 85, 1, 800);
	// turns to line up with corner
	gyroRotate.setTarget(50, 650);
	forwardController.setTarget(1300, 50, 1, 1000);
	// turns to corner
	gyroRotate.setTarget(90, 650);
	// gets 6th ring
	setLadyBrown("descore6th");
	forwardController.setTarget(2000, 90, 1, 1000);
	forwardController.setTarget(-800, 90, 1, 500);
	// turns around
	gyroRotate.setTarget(270, 650);
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	forwardController.setTarget(2100, 270, 1, 1300);
	setLadyBrownVoltage(12000);
}

void blue6AWP() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(505, 0, 1, 290);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.6);
	// backs off
	forwardController.setTarget(-500, 0, 1, 500);
	setLadyBrown("down");
	// puts intake up
	intakeLift.openSole();
	// turns to 2nd ring
	gyroRotate.setTarget(60, 650);
	intake.moveVoltage(12000);
	// gets 2nd ring
	forwardController.setTarget(700, 60, 1, 600);
	intakeLift.closeSole();
	// turns to mogo
	gyroRotate.setTarget(-11, 700);
	// backs into mogo and clamps it
	forwardController.setTarget(-1300, -11, 1, 800);
	forwardController.setTarget(-300, -11, 1, 350);
	clamp.openSole();
	// turns to line up with ring stack
	gyroRotate.setTarget(-181, 800);
	// scores on mogo
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	// wait(0.2);
	// drives to get parallel to middle
	forwardController.setTarget(1200, -181, 1, 900);
	// turns parallel to middle
	forwardController.setTarget(500, -130, 1, 400);
	// gets 3rd and 4th ring
	forwardController.setTarget(650, -130, 1, 600);
	// turns to back up
	gyroRotate.setTarget(-158, 600);
	forwardController.setTarget(-600, -158, 1, 500);
	// turns to 4th ring
	gyroRotate.setTarget(-85, 600);
	// grabs 5th
	forwardController.setTarget(800, -85, 1, 800);
	// turns to line up with corner
	gyroRotate.setTarget(-50, 650);
	forwardController.setTarget(1300, -50, 1, 1000);
	// turns to corner
	gyroRotate.setTarget(-95, 650);
	// gets 6th ring
	setLadyBrown("descore6th");
	forwardController.setTarget(2000, -95, 1, 1000);
	// backs out
	forwardController.setTarget(-800, -95, 1, 500);
	// turns around
	gyroRotate.setTarget(-270, 650);
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	forwardController.setTarget(2100, -270, 1, 1300);
	setLadyBrownVoltage(12000);
	// conveyor.moveVoltage(0);
	// setLadyBrown("down");
}

void oldBlue6AWP() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(505, 0, 1, 290);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.6);
	// backs off
	forwardController.setTarget(-800, 0, 1, 350);
	setLadyBrown("down");
	wait(0.1);
	// turns to 2nd ring
	gyroRotate.setTarget(65, 800);
	// puts intake up
	intakeLift.openSole();
	intake.moveVoltage(12000);
	wait(0.1);
	// gets 2nd ring
	forwardController.setTarget(600, 65, 1, 600);
	intakeLift.closeSole();
	// turns to mogo
	gyroRotate.setTarget(-19, 1000);
	// backs into mogo and clamps it
	forwardController.setTarget(-1300, -19, 1, 800);
	forwardController.setTarget(-300, -19, 1, 350);
	clamp.openSole();
	// turns to line up with ring stack
	gyroRotate.setTarget(-177, 800);
	// scores on mogo
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	// wait(0.2);
	// drives to get parallel to middle
	forwardController.setTarget(850, -177, 1, 700);
	conveyor.moveVoltage(12000);
	wait(0.2);
	// turns parallel to middle
	// leftDoinker.openSole();
	forwardController.setTarget(500, -128, 1, 400);
	// gyroRotate.setTarget(129, 360);
	conveyor.moveVoltage(12000);
	// gets 3rd and 4th ring
	forwardController.setTarget(500, -131, 0.8, 600);
	// leftDoinker.closeSole();
	// turns to 5th
	lDrive.moveVoltage(12000);
	wait(0.3);
	gyroRotate.setTarget(-20, 700);
	// grabs 5th
	forwardController.setTarget(800, -20, 1, 800);
	conveyor.moveVoltage(12000);
	// turns to line up with corner
	gyroRotate.setTarget(-66, 700);
	forwardController.setTarget(1000, -66, 1, 900);
	conveyor.moveVoltage(12000);
	// turns to corner
	gyroRotate.setTarget(-98, 650);
	conveyor.moveVoltage(12000);
	// gets 6th ring
	setLadyBrown("touchLadder");
	forwardController.setTarget(1200, -98, 1, 1200);
	forwardController.setTarget(-600, -98, 1, 500);
	// backs out
	forwardController.setTarget(-2600, -90, 1, 1700);
	conveyor.moveVoltage(0);
	setLadyBrown("down");
}

void oldRed6AWP() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(505, 0, 1, 290);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.6);
	// backs off
	forwardController.setTarget(-800, 0, 1, 350);
	setLadyBrown("down");
	wait(0.1);
	// turns to 2nd ring
	gyroRotate.setTarget(-65, 800);
	// puts intake up
	intakeLift.openSole();
	intake.moveVoltage(12000);
	wait(0.1);
	// gets 2nd ring
	forwardController.setTarget(600, -65, 1, 600);
	intakeLift.closeSole();
	// turns to mogo
	gyroRotate.setTarget(19, 1000);
	// backs into mogo and clamps it
	forwardController.setTarget(-1300, 19, 1, 800);
	forwardController.setTarget(-300, 19, 1, 350);
	clamp.openSole();
	// turns to line up with ring stack
	gyroRotate.setTarget(177, 800);
	// scores on mogo
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	// wait(0.2);
	// drives to get parallel to middle
	forwardController.setTarget(950, 177, 1, 750);
	conveyor.moveVoltage(12000);
	wait(0.2);
	// turns parallel to middle
	rightDoinker.openSole();
	forwardController.setTarget(500, 130, 1, 400);
	// gyroRotate.setTarget(129, 360);
	conveyor.moveVoltage(12000);
	// gets 3rd and 4th ring
	forwardController.setTarget(650, 130, 1, 600);
	rightDoinker.closeSole();
	// turns to 5th
	rDrive.moveVoltage(12000);
	wait(0.3);
	gyroRotate.setTarget(20, 700);
	// grabs 5th
	forwardController.setTarget(800, 20, 1, 800);
	conveyor.moveVoltage(12000);
	// turns to line up with corner
	gyroRotate.setTarget(66, 700);
	forwardController.setTarget(1300, 66, 1, 1000);
	conveyor.moveVoltage(12000);
	// turns to corner
	gyroRotate.setTarget(95, 650);
	conveyor.moveVoltage(12000);
	// gets 6th ring
	forwardController.setTarget(550, 95, 1, 400);
	setLadyBrown("touchLadder");
	forwardController.setTarget(900, 95, 1, 800);
	conveyor.moveVoltage(12000);
	forwardController.setTarget(-800, 95, 1, 500);
	conveyor.moveVoltage(12000);
	// backs out
	forwardController.setTarget(-2600, 95, 0.9, 1700);
	conveyor.moveVoltage(0);
	setLadyBrown("down");
}

void red7() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(505, 0, 1, 290);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.6);
	// backs off
	forwardController.setTarget(-500, 0, 1, 500);
	setLadyBrown("down");
	// puts intake up
	intakeLift.openSole();
	// turns to 2nd ring
	gyroRotate.setTarget(-65, 650);
	intake.moveVoltage(12000);
	// gets 2nd ring
	forwardController.setTarget(700, -65, 1, 600);
	intakeLift.closeSole();
	// turns to mogo
	gyroRotate.setTarget(15, 700);
	// backs into mogo and clamps it
	forwardController.setTarget(-1300, 15, 1, 800);
	forwardController.setTarget(-300, 15, 1, 350);
	clamp.openSole();
	// turns to line up with ring stack
	gyroRotate.setTarget(182, 800);
	// scores on mogo
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	// wait(0.2);
	// drives to get parallel to middle
	forwardController.setTarget(980, 182, 1, 800);
	// turns parallel to middle
	forwardController.setTarget(500, 130, 1, 400);
	// gets 3rd and 4th ring
	forwardController.setTarget(650, 130, 1, 550);
	// turns to back up
	gyroRotate.setTarget(160, 600);
	forwardController.setTarget(-600, 160, 1, 500);
	// turns to 5th ring
	gyroRotate.setTarget(85, 600);
	// grabs 5th
	forwardController.setTarget(800, 85, 1, 800);
	// turns and drives to line up with corner
	gyroRotate.setTarget(43, 650);
	forwardController.setTarget(1300, 43, 1, 1100);
	// turns to corner
	gyroRotate.setTarget(95, 650);
	getTwoCornerRings(95);
	// backs out
	jamDetection = true;
	forwardController.setTarget(-3500, 145, 1, 3000);
}

void blue7() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(505, 0, 1, 290);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.6);
	// backs off
	forwardController.setTarget(-500, 0, 1, 500);
	setLadyBrown("down");
	// puts intake up
	intakeLift.openSole();
	// turns to 2nd ring
	gyroRotate.setTarget(60, 650);
	intake.moveVoltage(12000);
	// gets 2nd ring
	forwardController.setTarget(700, 60, 1, 600);
	intakeLift.closeSole();
	// turns to mogo
	gyroRotate.setTarget(-11, 700);
	// backs into mogo and clamps it
	forwardController.setTarget(-1300, -11, 1, 800);
	forwardController.setTarget(-300, -11, 1, 350);
	clamp.openSole();
	// turns to line up with ring stack
	gyroRotate.setTarget(-181, 800);
	// scores on mogo
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	// wait(0.2);
	// drives to get parallel to middle
	forwardController.setTarget(1050, -181, 1, 900);
	// turns parallel to middle
	forwardController.setTarget(500, -130, 1, 400);
	// gets 3rd and 4th ring
	forwardController.setTarget(650, -130, 1, 600);
	// turns to back up
	gyroRotate.setTarget(-158, 600);
	forwardController.setTarget(-600, -158, 1, 500);
	// turns to 4th ring
	gyroRotate.setTarget(-85, 600);
	// grabs 5th
	forwardController.setTarget(800, -85, 1, 800);
	// turns and drives to line up with corner
	gyroRotate.setTarget(-43, 650);
	forwardController.setTarget(1150, -43, 1, 1000);
	// turns to corner
	gyroRotate.setTarget(-95, 650);
	getTwoCornerRings(-95);
	// backs out
	jamDetection = true;
	forwardController.setTarget(-3500, -145, 1, 3000);
}

void blue_6_ring() {
	colorSensing = false;
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(505, 0, 1, 290);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.6);
	// backs off
	forwardController.setTarget(-800, 0, 1, 350);
	setLadyBrown("down");
	wait(0.1);
	// turns to 2nd ring
	gyroRotate.setTarget(65, 800);
	// puts intake up
	intakeLift.openSole();
	intake.moveVoltage(12000);
	wait(0.1);
	// gets 2nd ring
	forwardController.setTarget(600, 65, 1, 600);
	intakeLift.closeSole();
	// turns to mogo
	gyroRotate.setTarget(-19, 1000);
	// backs into mogo and clamps it
	forwardController.setTarget(-1300, -19, 1, 800);
	forwardController.setTarget(-300, -19, 1, 350);
	clamp.openSole();
	// turns to line up with ring stack
	gyroRotate.setTarget(-177, 800);
	colorSensing = true;
	// scores on mogo
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	// wait(0.2);
	// drives to get parallel to middle
	forwardController.setTarget(880, -177, 1, 700);
	conveyor.moveVoltage(12000);
	wait(0.2);
	// turns parallel to middle
	forwardController.setTarget(500, -130, 1, 400);
	// gyroRotate.setTarget(129, 360);
	conveyor.moveVoltage(12000);
	// gets 3rd and 4th ring
	forwardController.setTarget(480, -130, 0.8, 600);
	// turns to 5th
	lDrive.moveVoltage(12000);
	wait(0.3);
	gyroRotate.setTarget(-20, 700);
	// grabs 5th
	forwardController.setTarget(800, -20, 1, 800);
	conveyor.moveVoltage(12000);
	// turns to line up with corner
	gyroRotate.setTarget(-66, 700);
	forwardController.setTarget(1150, -66, 1, 900);
	conveyor.moveVoltage(12000);
	// turns to corner
	gyroRotate.setTarget(-94, 650);
	conveyor.moveVoltage(12000);
	// gets 6th ring
	forwardController.setTarget(1200, -94, 1, 1200);
	forwardController.setTarget(-600, -94, 1, 500);
}

void ringRush() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// puts preload on alliance
	leftDoinker.openSole();
	intake.moveVoltage(12000);
	// rushes and grabs ring stack rings
	forwardController.setCurveTarget(2000, -38, 1, 2600);
	// backs off
	forwardController.setCurveTarget(-900, 10, 1, 800);
	forwardController.setTarget(-600, 10, 1, 700);
	leftDoinker.closeSole();
	wait(0.2);
	conveyor.moveVoltage(5000);
	gyroRotate.setTarget(90, 1000);
	conveyor.moveVoltage(0);
	forwardController.setTarget(-700, 90, 1, 700);
	gyroRotate.setTarget(135, 1000);
	// backs into mogo and clamps
	forwardController.setTarget(-1200, 135, 1, 1000);
	clamp.openSole();
	// turns to rings
	gyroRotate.setTarget(88, 1100);
	conveyor.moveVoltage(12000);
	forwardController.setTarget(1200, 90, 0.7, 1500);
	gyroRotate.setTarget(185, 1100);
	forwardController.setCurveTarget(800, 135, 0.7, 1500);
}

void redRingRush() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// puts preload on alliance
	leftDoinker.openSole();
	intake.moveVoltage(12000);
	// rushes and grabs ring stack rings
	forwardController.setCurveTarget(2000, -30, 1, 1200);
	// backs off and latches onto ring in ring stack
	conveyor.moveVoltage(3500);
	// turns to mogo
	gyroRotate.setTarget(-65, 650);
	forwardController.setTarget(-900, -65, 1, 700);
	// forwardController.setCurveTarget(-800, 80, 1, 900);
	conveyor.moveVoltage(0);
	// forwardController.setTarget(-300, 80, 1, 300);
	// grabs mogo
	clamp.openSole();
	leftDoinker.closeSole();
	// turns to 2nd and 3rd ring
	gyroRotate.setTarget(-105, 600);
	conveyor.moveVoltage(12000);
	// picks up rings
	forwardController.setTarget(1100, -105, 0.7, 1000);
	// turns, and goes to corner
	gyroRotate.setTarget(-180, 1000);
	jamDetection = false;
	setLadyBrownDelayed("load", 800);
	forwardController.setTarget(1100, -180, 1, 900);
	// turns to corner
	gyroRotate.setTarget(-128, 800);
	// drives into corner
	forwardController.setTarget(1400, -128, 1, 900);
	conveyor.moveVoltage(0);
	setLadyBrown("up");
	// backs out of corner
	forwardController.setTarget(-950, -130, 1, 800);
	jamDetection = true;
	conveyor.moveVoltage(12000);
	// turns to middle of field
	gyroRotate.setTarget(-270, 900);
	// drives to get last ring
	intakeLift.openSole();
	rightDoinker.openSole();
	forwardController.setTarget(2100, -270, 1, 1300);
	// turns to alliance
	gyroRotate.setTarget(-180, 1100);
	intakeLift.closeSole();
	rightDoinker.closeSole();
	// aligns with alliance
	forwardController.setTarget(1300, -180, 0.8, 1100);
	// backs off
	forwardController.setTarget(-450, -180, 1, 450);
	// scores on alliance
	setLadyBrown("alliance");
	wait(0.4);
	forwardController.setTarget(-650, -180, 1, 400);
	// setLadyBrown("down");
	gyroRotate.setTarget(0, 700);
	forwardController.setTarget(800, 0, 1, 800);
}

void blueRingRush() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// puts preload on alliance
	rightDoinker.openSole();
	intake.moveVoltage(12000);
	// rushes and grabs ring stack rings
	forwardController.setCurveTarget(2000, 30, 1, 1300);
	// backs off and latches onto ring in ring stack
	conveyor.moveVoltage(3500);
	// turns to mogo
	gyroRotate.setTarget(65, 650);
	forwardController.setTarget(-900, 65, 1, 700);
	// forwardController.setCurveTarget(-800, 80, 1, 900);
	conveyor.moveVoltage(0);
	// forwardController.setTarget(-300, 80, 1, 300);
	// grabs mogo
	clamp.openSole();
	rightDoinker.closeSole();
	// turns to 2nd and 3rd ring
	gyroRotate.setTarget(105, 650);
	conveyor.moveVoltage(12000);
	// picks up rings
	forwardController.setTarget(1100, 105, 0.7, 1000);
	// turns, and goes to corner
	gyroRotate.setTarget(180, 1000);
	jamDetection = false;
	setLadyBrownDelayed("load", 800);
	forwardController.setTarget(1100, 180, 1, 900);
	// turns to corner
	gyroRotate.setTarget(132, 900);
	// drives into corner
	forwardController.setTarget(1400, 132, 0.8, 900);
	conveyor.moveVoltage(0);
	setLadyBrown("up");
	// backs out of corner
	forwardController.setTarget(-900, 135, 1, 800);
	jamDetection = true;
	conveyor.moveVoltage(12000);
	// turns to middle of field
	gyroRotate.setTarget(270, 900);
	// drives to get last ring
	intakeLift.openSole();
	leftDoinker.openSole();
	forwardController.setTarget(2250, 270, 1, 1300);
	intakeLift.closeSole();
	wait(0.1);
	// turns to alliance
	gyroRotate.setTarget(180, 1300);
	leftDoinker.closeSole();
	// aligns with alliance
	forwardController.setTarget(1300, 180, 0.8, 1200);
	// backs off
	forwardController.setTarget(-450, 180, 1, 450);
	// scores on alliance
	setLadyBrown("alliance");
	wait(0.5);
	forwardController.setTarget(-650, 180, 0.5, 600);
	setLadyBrown("down");
	// gyroRotate.setTarget(0, 900);
	// forwardController.setTarget(800, 0, 1, 800);
	
}

void redRingRushElims() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// puts preload on alliance
	leftDoinker.openSole();
	intake.moveVoltage(12000);
	// rushes and grabs ring stack rings
	forwardController.setCurveTarget(2000, -30, 1, 1200);
	// backs off and latches onto ring in ring stack
	conveyor.moveVoltage(3500);
	// turns to mogo
	gyroRotate.setTarget(-65, 650);
	forwardController.setTarget(-950, -65, 1, 950);
	conveyor.moveVoltage(0);
	// grabs mogo
	clamp.openSole();
	leftDoinker.closeSole();
	// turns to 2nd and 3rd ring
	gyroRotate.setTarget(-105, 600);
	conveyor.moveVoltage(12000);
	// picks up rings
	forwardController.setTarget(1100, -105, 0.7, 1000);
	// turns, and goes to corner
	gyroRotate.setTarget(-180, 1000);
	jamDetection = false;
	forwardController.setTarget(1100, -180, 1, 900);
	// turns to corner
	intakeLift.openSole();
	gyroRotate.setTarget(-130, 800);
	// drives into corner
	jamDetection = false;
	forwardController.setTarget(1400, -130, 1, 900);
	intakeLift.closeSole();
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	wait(0.5);
	forwardController.setTarget(-450, -130, 1, 450);
	forwardController.setTarget(600, -130, 1, 500);
	// backs out of corner
	jamDetection = true;
	forwardController.setTarget(-900, -130, 1, 800);
	conveyor.moveVoltage(12000);
	// turns to middle of field
	gyroRotate.setTarget(-270, 850);
	// drives to get last 2 rings
	jamDetection = false;conveyor.moveVoltage(12000);
	setLadyBrown("load");
	forwardController.setTarget(1100, -270, 1, 900);
	intakeLift.openSole();
	rightDoinker.openSole();
	forwardController.setTarget(1000, -270, 1, 800);
	// turns to alliance
	setLadyBrownDelayed("up", 500, true);
	intakeLift.closeSole();
	gyroRotate.setTarget(-180, 900);
	conveyor.moveVoltage(12000);
	jamDetection = true;
	rightDoinker.closeSole();
	// aligns with alliance
	forwardController.setTarget(1300, -180, 0.8, 900);
	// backs off and scores on alliance
	setLadyBrownDelayed("alliance", 200);
	forwardController.setTarget(-450, -180, 1, 450);
	
	
}

void blueRingRushElims() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// puts preload on alliance
	rightDoinker.openSole();
	intake.moveVoltage(12000);
	// rushes and grabs ring stack rings
	forwardController.setCurveTarget(2000, 28, 1, 1200);
	// backs off and latches onto ring in ring stack
	conveyor.moveVoltage(3500);
	// turns to mogo
	gyroRotate.setTarget(65, 650);
	forwardController.setTarget(-950, 65, 1, 950);
	conveyor.moveVoltage(0);
	// grabs mogo
	clamp.openSole();
	rightDoinker.closeSole();
	// turns to 2nd and 3rd ring
	gyroRotate.setTarget(105, 600);
	conveyor.moveVoltage(12000);
	// picks up rings
	forwardController.setTarget(1100, 105, 0.7, 1000);
	// turns, and goes to corner
	gyroRotate.setTarget(180, 1000);
	jamDetection = false;
	forwardController.setTarget(1100, 180, 1, 900);
	// turns to corner
	intakeLift.openSole();
	gyroRotate.setTarget(130, 800);
	// drives into corner
	jamDetection = false;
	forwardController.setTarget(1400, 130, 1, 900);
	intakeLift.closeSole();
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	wait(0.5);
	forwardController.setTarget(-450, 130, 1, 450);
	forwardController.setTarget(600, 130, 1, 500);
	// backs out of corner
	jamDetection = true;
	forwardController.setTarget(-900, 130, 1, 800);
	conveyor.moveVoltage(12000);
	// turns to middle of field
	gyroRotate.setTarget(270, 850);
	// drives to get last 2 rings
	jamDetection = false;
	conveyor.moveVoltage(12000);
	setLadyBrown("load");
	forwardController.setTarget(1100, 270, 1, 900);
	intakeLift.openSole();
	leftDoinker.openSole();
	forwardController.setTarget(960, 270, 1, 750);
	// turns to alliance
	setLadyBrownDelayed("up", 500, true);
	intakeLift.closeSole();
	gyroRotate.setTarget(180, 900);
	conveyor.moveVoltage(12000);
	jamDetection = true;
	leftDoinker.closeSole();
	// aligns with alliance
	forwardController.setTarget(1300, 180, 0.8, 900);
	// backs off and scores on alliance
	setLadyBrownDelayed("alliance", 200);
	forwardController.setTarget(-450, 180, 1, 450);
	
	
}

void blueWallStake() {
	forwardController.setTarget(-1400, 0, 1, 1000);
	clamp.openSole();
	setLadyBrown("up");
	gyroRotate.setTarget(90, 800);
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	forwardController.setTarget(900, 90, 1, 700);
	wait(1);
	clamp.closeSole();
	gyroRotate.setTarget(137, 800);
	forwardController.setTarget(1300, 137, 1, 1200);
	setLadyBrownVoltage(5000);
	// gyroRotate.setTarget(30, 800);
	// forwardController.setTarget(1200, 30, 1, 700);
	// leftDoinker.openSole();
	// forwardController.setTarget(500, 30, 0.7, 500);
	// gyroRotate.setTarget(, 800);


}

void redWallStake() {
	forwardController.setTarget(-1400, 0, 1, 1000);
	clamp.openSole();
	setLadyBrown("up");
	gyroRotate.setTarget(-90, 800);
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	forwardController.setTarget(900, -90, 1, 700);
	wait(1);
	clamp.closeSole();
	gyroRotate.setTarget(-137, 800);
	forwardController.setTarget(1100, -137, 1, 1200);
	setLadyBrownVoltage(5000);
}

void redMogo() {
	forwardController.setTarget(-1400, 0, 1, 1000);
	clamp.openSole();
	wait(0.4);
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	gyroRotate.setTarget(-90, 800);
	conveyor.moveVoltage(0);
	forwardController.setTarget(600, -90, 1, 800);
	clamp.closeSole();
	forwardController.setTarget(500, -90, 1, 600);
	gyroRotate.setTarget(2, 1000);
	forwardController.setTarget(-830, 0, 0.4, 1600);
	clamp.openSole();
	wait(0.2);
	conveyor.moveVoltage(12000);
	forwardController.setTarget(500, 0, 1, 600);
}

void blueWallStakeExtended() {
	forwardController.setTarget(-1400, 0, 1, 1000);
	clamp.openSole();
	setLadyBrown("up");
	gyroRotate.setTarget(90, 800);
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	forwardController.setTarget(1000, 90, 1, 700);
	wait(1);
	clamp.closeSole();
	gyroRotate.setTarget(132, 800);
	forwardController.setTarget(1400, 132, 1, 1200);
	setLadyBrownVoltage(5000);
	// gyroRotate.setTarget(30, 800);
	// forwardController.setTarget(1200, 30, 1, 700);
	// leftDoinker.openSole();
	// forwardController.setTarget(500, 30, 0.7, 500);
	// gyroRotate.setTarget(, 800);
}

void oldRed6Baker() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// drives and picks up goal
	forwardController.setTarget(-1100, 0, 1, 700);
	forwardController.setTarget(-700, 0, 0.9, 500);
	clamp.openSole();
	conveyor.moveVoltage(12000);
	// turns to middle rings
	gyroRotate.setTarget(130, 800); // 135 too much
	conveyor.moveVoltage(3000);
	// drives to middle rings
	forwardController.setTarget(950, 130, 1, 700); 
	// grabs 1st ring with right doinker
	rightDoinker.openSole();
	wait(0.15);
	conveyor.moveVoltage(0);
	// turns to 2nd ring
	gyroRotate.setTarget(168, 700);
	// gets 2nd ring with left doinker
	leftDoinker.openSole();
	wait(0.15);
	// turns to back out from under ladder
	gyroRotate.setTarget(120, 400);
	gyroRotate.setTarget(140, 400);
	// outakes opposite color ring
	intake.moveVoltage(-12000);
	// backs off
	forwardController.setTarget(-700, 140, 1, 500);
	forwardController.setTarget(-1400, 175, 1, 1000);
	// drops rings
	rightDoinker.closeSole();
	leftDoinker.closeSole();
	// turns to ring
	gyroRotate.setTarget(130, 700);
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	// picks up ring
	forwardController.setTarget(700, 130, 1, 600);
	// turns to other 2 rings and picks them up
	gyroRotate.setTarget(253, 900);
	forwardController.setTarget(1475, 253, 1, 1000);
	// turns and drives towards corner
	gyroRotate.setTarget(360, 900);
	forwardController.setTarget(1280, 360, 1, 980);
	// turns to corner
	gyroRotate.setTarget(310, 700);
	intakeLift.openSole();
	// drives into corner
	jamDetection = false;
	forwardController.setTarget(1400, 310, 1, 900);
	intakeLift.closeSole();
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	forwardController.setTarget(400, 310, 1, 400);
	wait(0.3);
	forwardController.setTarget(-450, 310, 1, 450);
	forwardController.setTarget(500, 310, 1, 500);
	wait(0.2);
	forwardController.setTarget(-500, 310, 1, 450);
	clamp.closeSole();
	grabGoal = false;
	forwardController.setTarget(500, 310, 1, 450);
	gyroRotate.setTarget(360, 700);
	forwardController.setTarget(-1200, 360, 1, 1000);


	// gets blue ring out
	// forwardController.setTarget(-450, 310, 1, 450);
	// forwardController.setTarget(400, 310, 1, 500);


	// backs out of corner
	// jamDetection = true;
	// forwardController.setTarget(-450, 310, 1, 450);
	// gyroRotate.setTarget(180, 800);
	
	
}

void red6BakerBase() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	conveyor.moveVoltage(0);
	// drives and picks up goal
	forwardController.setTarget(-1750, 0, 0.9, 1200);
	clamp.openSole();
	// turns to middle rings
	gyroRotate.setTarget(125, 700); // 135 too much
	conveyor.moveVoltage(12000);
	// drives to middle rings
	intake.moveVoltage(-1000);
	forwardController.setTarget(1000, 113, 1, 700);
	conveyor.moveVoltage(0);
	// grabs 1st and 2nd ring with left doinker
	rightDoinker.openSole();
	wait(0.2);
	gyroRotate.setTarget(148, 600);
	// backs out from under ladder
	forwardController.setTarget(-1000, 148, 1, 800);
	intake.moveVoltage(0);
	gyroRotate.setTarget(262, 700);
	rightDoinker.closeSole();
	wait(0.2);
	gyroRotate.setTarget(315, 700);
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	forwardController.setTarget(1500, 315, 1, 200);
	rightDoinker.openSole();
	forwardController.setTarget(1500, 240, 0.4, 1200);
	rightDoinker.closeSole();

	// turns and drives towards corner
	gyroRotate.setTarget(360, 750);
	forwardController.setTarget(1200, 360, 1, 860); // 1100 too much
	// turns to corner
	gyroRotate.setTarget(312, 650);
	getTwoCornerRings(312);
}

void red6BakerBar() {
	red6BakerBase();
	// backs out and touches bar
	forwardController.setTarget(-800, 312, 1, 600);
	// turns around
	gyroRotate.setTarget(485, 650);
	setLadyBrown("descore6th");
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	forwardController.setTarget(2100, 485, 0.8, 1300);
	setLadyBrownVoltage(12000);
}

void red6Baker() {
	red6BakerBase();
	// gets red ring
	forwardController.setTarget(-400, 313, 1, 450);
	forwardController.setTarget(1000, 313, 1, 300);
	// backs out of corner
	forwardController.setTarget(-1500, 313, 1, 600);
	// turns to drop mogo
	gyroRotate.setTarget(448, 650);
	conveyor.moveVoltage(0);
	clamp.closeSole();
	grabGoal = false;
	intake.moveVoltage(0);
	forwardController.setTarget(1500, 448, 1, 200);
	gyroRotate.setTarget(372, 600);
	forwardController.setTarget(-1200, 372, 1, 900);
}

void blue6BakerBase() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	conveyor.moveVoltage(0);
	// drives and picks up goal
	forwardController.setTarget(-1750, 0, 0.9, 1200);
	clamp.openSole();
	// turns to middle rings
	gyroRotate.setTarget(-125, 700); // 135 too much
	conveyor.moveVoltage(12000);
	// drives to middle rings
	intake.moveVoltage(-1000);
	forwardController.setTarget(1000, -113, 1, 700);
	conveyor.moveVoltage(0);
	// grabs 1st and 2nd ring with left doinker
	leftDoinker.openSole();
	wait(0.2);
	gyroRotate.setTarget(-148, 600);
	// backs out from under ladder
	forwardController.setTarget(-1000, -148, 1, 800);
	intake.moveVoltage(0);
	gyroRotate.setTarget(-262, 700);
	leftDoinker.closeSole();
	wait(0.2);
	gyroRotate.setTarget(-325, 700);
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	forwardController.setTarget(1500, -325, 1, 200);
	leftDoinker.openSole();
	forwardController.setTarget(1500, -240, 0.4, 1200);
	leftDoinker.closeSole();

	// turns and drives towards corner
	gyroRotate.setTarget(-360, 750);
	forwardController.setTarget(1200, -360, 1, 860); // 1100 too much
	// turns to corner
	gyroRotate.setTarget(-312, 650);
	getTwoCornerRings(-312);
} 

void blue6BakerBar() {
	blue6BakerBase();

	// backs out and touches bar
	forwardController.setTarget(-800, -313, 1, 600);
	// turns around
	gyroRotate.setTarget(-485, 650);
	setLadyBrown("descore6th");
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	forwardController.setTarget(2100, -485, 1, 1300);
	setLadyBrownVoltage(12000);
	
	
}

void blue6Baker() {
	blue6BakerBase();
	// gets red ring
	forwardController.setTarget(-400, -313, 1, 450);
	forwardController.setTarget(1000, -313, 1, 400);
	// backs out of corner
	forwardController.setTarget(-1500, -358, 1, 600);
	// turns to drop mogo
	gyroRotate.setTarget(-448, 650);
	conveyor.moveVoltage(0);
	clamp.closeSole();
	grabGoal = false;
	intake.moveVoltage(0);
	forwardController.setTarget(1500, -448, 1, 250);
	gyroRotate.setTarget(-372, 600);
	forwardController.setTarget(-1200, -372, 1, 900);
}

void red5Baker() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// drives and picks up goal
	forwardController.setTarget(-1100, 0, 1, 700);
	forwardController.setTarget(-700, 0, 0.9, 500);
	clamp.openSole();
	conveyor.moveVoltage(12000);
	// turns to middle rings
	gyroRotate.setTarget(130, 800); // 135 too much
	conveyor.moveVoltage(3000);
	// drives to middle rings
	forwardController.setTarget(900, 130, 1, 700); 
	// grabs 1st ring with right doinker
	rightDoinker.openSole();
	wait(0.15);
	conveyor.moveVoltage(0);
	// outakes opposite color ring
	intake.moveVoltage(-12000);
	// backs off
	forwardController.setTarget(-1125, 140, 1, 900);
	// turns to rings
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	gyroRotate.setTarget(220, 500);
	rightDoinker.closeSole();
	// drops ring off
	gyroRotate.setTarget(263, 900);
	// grabs ring
	forwardController.setTarget(1250, 263, 1, 1000);
	// turns and drives towards corner
	gyroRotate.setTarget(360, 900);
	forwardController.setTarget(1125, 360, 1, 980); // 1100 too much
	// turns to corner
	gyroRotate.setTarget(313, 700);
	getTwoCornerRings(313);
	// gets red ring
	forwardController.setTarget(-400, 313, 1, 450);
	forwardController.setTarget(550, 313, 1, 500);
	// backs out of corner
	forwardController.setTarget(-1500, 313, 1, 600);
	gyroRotate.setTarget(448, 650);
	conveyor.moveVoltage(0);
	clamp.closeSole();
	grabGoal = false;
	intake.moveVoltage(0);
	forwardController.setTarget(500, 448, 1, 450);
	gyroRotate.setTarget(372, 650);
	forwardController.setTarget(-1200, 372, 1, 1200);
	
}

void blue5Baker() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// drives and picks up goal
	forwardController.setTarget(-1100, 0, 1, 700);
	forwardController.setTarget(-700, 0, 0.9, 500);
	clamp.openSole();
	conveyor.moveVoltage(12000);
	// turns to middle rings
	gyroRotate.setTarget(-130, 800); // 135 too much
	conveyor.moveVoltage(3000);
	// drives to middle rings
	forwardController.setTarget(900, -130, 1, 700); 
	// grabs 1st ring with left doinker
	leftDoinker.openSole();
	rightDoinker.openSole();
	wait(0.15);
	conveyor.moveVoltage(0);
	// outakes opposite color ring
	intake.moveVoltage(-12000);
	// backs off
	forwardController.setTarget(-1125, -140, 1, 900);
	rightDoinker.closeSole();
	// turns to rings
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	gyroRotate.setTarget(-220, 500);
	leftDoinker.closeSole();
	// drops ring off
	gyroRotate.setTarget(-263, 900);
	// grabs ring
	forwardController.setTarget(1250, -263, 1, 1000);
	// turns and drives towards corner
	gyroRotate.setTarget(-360, 900);
	forwardController.setTarget(1125, -360, 1, 980); // 1100 too much
	// turns to corner
	gyroRotate.setTarget(-313, 700);
	getTwoCornerRings(-313);
	// gets red ring
	forwardController.setTarget(-400, -313, 1, 450);
	forwardController.setTarget(550, -313, 1, 500);
	// backs out of corner
	forwardController.setTarget(-1500, -313, 1, 600);
	gyroRotate.setTarget(-448, 650);
	conveyor.moveVoltage(0);
	clamp.closeSole();
	grabGoal = false;
	intake.moveVoltage(0);
	forwardController.setTarget(500, -448, 1, 450);
	gyroRotate.setTarget(-372, 650);
	forwardController.setTarget(-1200, -372, 1, 1200);

	// gets blue ring out
	// forwardController.setTarget(-450, 310, 1, 450);
	// forwardController.setTarget(400, 310, 1, 500);


	// backs out of corner
	// jamDetection = true;
	// forwardController.setTarget(-450, 310, 1, 450);
	// gyroRotate.setTarget(180, 800);
	
	
}

void red5BakerRush() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	forwardController.setCurveTarget(1950, 14, 1, 1600);
	gyroRotate.setTarget(-25, 900);
	forwardController.setCurveTarget(200, -25, 1, 500);
	leftDoinker.openSole();
	wait(0.4);
	gyroRotate.setTarget(-70, 1000);
	forwardController.setCurveTarget(-800, -70, 1, 600);
}

void redMogoRush() {
	teamColor = "red";
	colorSensing = false;
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// rushes and grabs middle mogo
	rightDoinker.openSole();
	forwardController.setCurveTarget(1800, 10, 1, 1100);
	// grabs mogo with doinker
	doinkerClamp.openSole();
	// backs off and drops goal off
	forwardController.setCurveTarget(-1300, 5, 1, 1000);
	doinkerClamp.closeSole();
	// puts doinker up
	rightDoinker.closeSole();
	// turns to 2nd mogo
	gyroRotate.setTarget(135, 1000);
	// drives and picks up 2nd mogo
	intake.moveVoltage(12000);
	colorSensing = true;
	forwardController.setTarget(600, 135, 1, 500);
	forwardController.setTarget(-1400, 135, 1, 1000);
	clamp.openSole();
	conveyor.moveVoltage(12000);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
	intake.moveVoltage(12000);
	// turns to corner to get rings
	// gyroRotate.setTarget(155, 300);
	// gyroRotate.setTarget(133, 450);
	// drives into corner
	forwardController.setTarget(1600, 133, 1, 1000);
	getTwoCornerRings(133);
	// gets red ring
	forwardController.setTarget(-400, 135, 1, 450);
	forwardController.setTarget(550, 135, 1, 500);
	// backs out of corner and drops mogo off
	forwardController.setTarget(-1800, 135, 1, 1300);
	clamp.closeSole();
	jamDetection = true;
	forwardController.setTarget(900, 135, 1, 700);
	conveyor.moveVoltage(0);
	intake.moveVoltage(0);
	// turns to 1st mogo
	gyroRotate.setTarget(180, 600);
	// backs into 1st mogo and clamps
	forwardController.setTarget(-1200, 180, 1, 900);
	clamp.openSole();
	rightDoinker.openSole();
	gyroRotate.setTarget(80, 700);
	conveyor.moveVoltage(12000);
	intake.moveVoltage(12000);
	rightDoinker.closeSole();
	forwardController.setTarget(800, 80, 1, 600);

}

void blueMogoRush() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// rushes and grabs middle mogo
	rightDoinker.openSole();
	forwardController.setCurveTarget(1800, 8, 1, 1100);
	// grabs mogo with doinker
	doinkerClamp.openSole();
	// backs off and drops goal off
	forwardController.setCurveTarget(-1300, 0, 1, 800);
	doinkerClamp.closeSole();
	// puts doinker up
	rightDoinker.closeSole();
	// turns to 2nd mogo
	gyroRotate.setTarget(-115, 1000);
	// drives and picks up 2nd mogo
	forwardController.setTarget(-1000, -115, 1, 700);
	forwardController.setTarget(-800, -115, 0.8, 600);
	clamp.openSole();
	conveyor.moveVoltage(12000);
	intake.moveVoltage(12000);
	// turns to corner to get rings
	gyroRotate.setTarget(-155, 300);
	gyroRotate.setTarget(-133, 450);
	// drives into corner
	forwardController.setTarget(1600, -133, 1, 1200);
	intakeLift.openSole();
	jamDetection = false;
	conveyor.moveVoltage(12000);
	intake.moveVoltage(12000);
	forwardController.setTarget(1300, -133, 1, 900);
	intakeLift.closeSole();
	// puts continuous pressure on rings
	// forwardController.setTarget(2500, -133, 1, 1000);
	wait(0.6);
	// gets 2nd blue ring
	forwardController.setTarget(-600, -133, 1, 450);
	forwardController.setTarget(700, -133, 1, 600);
	// gets red ring
	forwardController.setTarget(-400, -133, 1, 450);
	forwardController.setTarget(550, -133, 1, 500);
	// backs out of corner and drops mogo off
	clamp.closeSole();
	forwardController.setTarget(-1100, -133, 1, 900);
	jamDetection = true;
	conveyor.moveVoltage(0);
	intake.moveVoltage(0);
	forwardController.setTarget(500, -133, 1, 500);
	// turns to 1st mogo
	gyroRotate.setTarget(-180, 600);
	// backs into 1st mogo and clamps
	forwardController.setTarget(-1600, -180, 1, 1000);
	clamp.openSole();
	rightDoinker.openSole();
	gyroRotate.setTarget(-290, 700);
	conveyor.moveVoltage(12000);
	intake.moveVoltage(12000);
	rightDoinker.closeSole();
	forwardController.setTarget(800, -290, 1, 600);

}

void redTop3Ring() {
	// Top3RingSetup();
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// rushes for 3rd goal and gets ring on the way
	rightDoinker.openSole();
	intake.moveVoltage(12000);
	forwardController.setTarget(1750, 0, 1, 1400);
	doinkerClamp.openSole();
	// backs off of center line
	forwardController.setTarget(-700, 0, 1, 700);
	// lets go of goal and turns to other goal
	rightDoinker.closeSole();
	doinkerClamp.closeSole();
	gyroRotate.setTarget(118, 900);
	// grabs other goal
	setLadyBrown("touchLadder");
	forwardController.setTarget(-1400, 118, 1, 1100);
	clamp.openSole();
	// turns to corner
	conveyor.moveVoltage(4000);
	gyroRotate.setTarget(155, 1000);
	conveyor.moveVoltage(12000);
	rightDoinker.openSole();
	// drives to corner
	forwardController.setTarget(2200, 153, 1, 1800);
	// sweeps corner
	gyroRotate.setTarget(18, 1000);
	conveyor.moveVoltage(0);
	// grabs bottom ring
	forwardController.setTarget(900, 18, 1, 600);
	clamp.closeSole();
	// turns around and gets other goal
	gyroRotate.setTarget(-155, 1300);
	rightDoinker.closeSole();
	forwardController.setTarget(-1000, -155, 1, 800);
	clamp.openSole();
	gyroRotate.setTarget(-290, 1000);
	forwardController.setTarget(800, -290, 1, 600);
}

void redPosAWP() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(505, 0, 1, 290);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.6);
	// backs off
	forwardController.setTarget(-500, 0, 1, 500);
	setLadyBrown("down");
	// puts intake up
	intakeLift.openSole();
	// turns to 2nd ring
	gyroRotate.setTarget(60, 650);
	intake.moveVoltage(12000);
	// gets 2nd ring
	forwardController.setTarget(700, 60, 1, 600);
	intakeLift.closeSole();
	// turns to mogo
	gyroRotate.setTarget(-11, 700);
	// backs into mogo and clamps it
	forwardController.setTarget(-1400, -11, 1, 1200);
	clamp.openSole();
	// scores 2nd ring
	conveyor.moveVoltage(12000);
	intake.moveVoltage(12000);
	// turns around and drops mogo
	gyroRotate.setTarget(169, 800);
	clamp.closeSole();
	// turns and grabs 3rd ring 
	gyroRotate.setTarget(220, 800);
	conveyor.moveVoltage(0);
	forwardController.setTarget(1400, 220, 1, 1000);
	// turns to 2nd mogo
	gyroRotate.setTarget(307, 800);
	// backs up slowly
	forwardController.setTarget(-750, 307, 0.5, 1000);
	// grabs 2nd mogo
	clamp.openSole();
	// scores 3rd ring
	conveyor.moveVoltage(12000);
	// drives toward corner
	forwardController.setTarget(1600, 307, 1, 1300);
	// turns to corner
	gyroRotate.setTarget(262, 800);
	// gets 4th ring
	forwardController.setTarget(2000, 262, 1, 1600);

	// backs out, turns around and touches bar
	forwardController.setTarget(-800, 262, 1, 600);
	// turns around
	gyroRotate.setTarget(100, 650);
	setLadyBrown("descore6th");
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	forwardController.setTarget(2250, 100, 1, 1300);
	// setLadyBrownVoltage(12000);

}

void bluePosAWP() {
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();
	// puts preload on alliance
	forwardController.setTarget(505, 0, 1, 290);
	// scores on alliance with ladybrown
	ladyBrown.tarePosition();
	ladyBrown.moveVoltage(12000);
	wait(0.6);
	// backs off
	forwardController.setTarget(-500, 0, 1, 500);
	setLadyBrown("down");
	// puts intake up
	intakeLift.openSole();
	// turns to 2nd ring
	gyroRotate.setTarget(-70, 650);
	intake.moveVoltage(12000);
	// gets 2nd ring
	forwardController.setTarget(700, -70, 1, 600);
	intakeLift.closeSole();
	// turns to mogo
	gyroRotate.setTarget(11, 800);
	// backs into mogo and clamps it
	forwardController.setTarget(-1400, 11, 1, 1200);
	clamp.openSole();
	// scores 2nd ring
	conveyor.moveVoltage(12000);
	intake.moveVoltage(12000);
	// turns around and drops mogo
	gyroRotate.setTarget(-169, 800);
	clamp.closeSole();
	// turns and grabs 3rd ring 
	gyroRotate.setTarget(-220, 800);
	conveyor.moveVoltage(0);
	forwardController.setTarget(1250, -220, 1, 1000);
	// turns to 2nd mogo
	gyroRotate.setTarget(-307, 800);
	// backs up slowly
	forwardController.setTarget(-750, -307, 0.5, 1000);
	// grabs 2nd mogo
	clamp.openSole();
	// scores 3rd ring
	conveyor.moveVoltage(12000);
	// drives toward corner
	forwardController.setTarget(1800, -307, 1, 1500);
	// turns to corner
	gyroRotate.setTarget(-262, 800);
	// gets 4th ring
	forwardController.setTarget(2000, -262, 1, 1600);

	// // backs out, turns around and touches bar
	forwardController.setTarget(-800, -262, 1, 600);
	// // turns around
	gyroRotate.setTarget(-100, 650);
	setLadyBrown("descore6th");
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::coast);
	forwardController.setTarget(2250, -100, 1, 1300);
	// setLadyBrownVoltage(12000);

}

void skillsAuto() {
	teamColor = "blue";
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::hold);
	conveyor.setBrakeMode(AbstractMotor::brakeMode::coast);
	// tares heading
	imu->tare_rotation();
	gyroRotate.tareHeading();

	// 1st half
	// puts preload on alliance with ladybrown
	setLadyBrownVoltage(12000);
	wait(0.6);
	ladyBrown.moveVoltage(0);
	// backs off alliance into mogo
	forwardController.setTarget(-900, 0, 1, 900);
	// puts ladybrown down
	setLadyBrown("down");

	// 1st mogo
	// clamps mogo
	clamp.openSole();
	// turns to 1st ring
	gyroRotate.setTarget(125, 900);
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	// drives and picks up 1st ring
	forwardController.setTarget(1000, 125, 1, 800);
	// turns to 1st wallstake ring
	gyroRotate.setTarget(165, 300);
	gyroRotate.setTarget(153, 450);
	// picks up 1st wallstake ring and loads it into ladybrown
	setLadyBrownDelayed("load", 900);
	forwardController.setTarget(2700, 153, 1, 1500);
	jamDetection = false;
	// turns and drives to line up with 2nd ring and 1st wall stake
	gyroRotate.setTarget(290, 1200);
	conveyor.moveVoltage(0);
	forwardController.setTarget(1130, 290, 1, 900);
	conveyor.moveVoltage(12000);
	// turns to 1st wall stake and 2nd ring
	gyroRotate.setTarget(215, 900);
	// stops conveyor so ladybrown can move
	conveyor.moveVoltage(0);
	// puts ladybrown up
	setLadyBrown("up");
	// picks up 2nd ring and presses against 1st wallstake
	forwardController.setTarget(500, 217, 0.7, 500); // 218 turned to far right
	jamDetection = true;
	// scores 3rd ring while scoring on 1st wallstake
	conveyor.moveVoltage(12000);
	setLadyBrown("wallScored");
	forwardController.setTarget(500, 217, 0.1, 500);
	// backs off 1st wallstake
	forwardController.setTarget(-750, 217, 1, 700);
	// turns to 3rd 4th and 5th
	gyroRotate.setTarget(308, 900);
	// picks up 3rd 4th and 5th
	forwardController.setTarget(2700, 308, 0.65, 2000);
	// turns to 6th ring
	gyroRotate.setTarget(190, 1000);
	// turns and picks up 6th ring
	forwardController.setTarget(750, 190, 1, 850);
	// turns to corner
	gyroRotate.setTarget(117, 900);  // 117 too much
	// backs into corner
	clamp.closeSole();
	conveyor.moveVoltage(-12000);
	forwardController.setTarget(-700, 117, 1, 600);
	// drops 1st mogo in corner

	// transition 1
	setLadyBrown("down");
	// gets out of corner
	conveyor.moveVoltage(0);
	intake.moveVoltage(0);
	forwardController.setTarget(475, 117, 1, 500);
	// turns, drives, and picks up 2nd mogo
	gyroRotate.setTarget(219, 1400);
	forwardController.setTarget(-3300, 219, 1, 1700); // dont change 217
	forwardController.setTarget(-700, 219, 0.7, 700);

	// 2nd mogo
	clamp.openSole();
	// turns to 1st ring
	gyroRotate.setTarget(126, 1200);
	intake.moveVoltage(12000);
	conveyor.moveVoltage(12000);
	// grabs 1st ring
	forwardController.setTarget(920, 126, 1, 1000); // 850 too little
	// turns to 2nd wallstake ring
	gyroRotate.setTarget(90, 300);
	gyroRotate.setTarget(105, 800);
	// picks up 2nd wallstake ring and loads it into ladybrown
	setLadyBrownDelayed("load", 1000);
	forwardController.setTarget(2600, 105, 1, 1500);
	jamDetection = false;
	// turns and drives to line up with 2nd ring and 2nd wall stake
	gyroRotate.setTarget(-28, 1000);  // -49 too little
	conveyor.moveVoltage(0);
	forwardController.setTarget(1200, -28, 1, 950); // 1100 too far
	conveyor.moveVoltage(12000);
	// turns to wall stake and 2nd ring
	gyroRotate.setTarget(41, 1000); // 41 too much 33 too little
	// puts ladybrown up
	setLadyBrown("wallScored");
	conveyor.moveVoltage(0);
	// picks up 2nd ring and presses against 2nd wallstake
	forwardController.setTarget(695, 41, 1, 700);
	forwardController.setTarget(500, 41, 0.6, 500);
	jamDetection = true;
	// scores on 2nd wall stake
	conveyor.moveVoltage(12000);
	wait(0.1);
	setLadyBrown("wallScored");
	// forwardController.setTarget(500, 37, 0.1, 500);
	wait(0.35);
	// backs off wallstake
	forwardController.setTarget(-780, 41, 1, 700); // 710 too little
	// turns to 3rd 4th and 5th
	gyroRotate.setTarget(-50, 900);
	// picks up 3rd 4th and 5th
	forwardController.setTarget(2750, -50, 0.6, 2350);
	// turns to 6th ring
	gyroRotate.setTarget(74, 900);
	// turns and picks up 6th ring
	forwardController.setTarget(750, 74, 1, 750);
	// turns to corner
	gyroRotate.setTarget(145, 1000); // 135 too far left
	// backs into corner
	conveyor.moveVoltage(-12000);
	forwardController.setTarget(-700, 142, 1, 450);
	clamp.closeSole();
	// drop 2nd goal
	setLadyBrown("down");
	
	// transition 2
	intake.moveVoltage(0);
	conveyor.moveVoltage(0);
	wait(0.2);
	// gets out of corner
	forwardController.setTarget(2750, 140, 1, 1500);
	conveyor.moveVoltage(0);
	// turns to 1st ring and 3rd mogo
	gyroRotate.setTarget(168, 750);
	conveyor.moveVoltage(0);
	intake.moveVoltage(12000);
	// gets 1st ring
	forwardController.setTarget(2100, 168, 1, 1400);
	// turns around
	gyroRotate.setTarget(0, 1100);
	// backs into 3rd mogo
	forwardController.setTarget(-1000, 0, 0.7, 1000); // 1000 too little // 1500 too much
	
	// 2nd half
	// 3rd mogo
	clamp.openSole();
	// scores 1st ring
	conveyor.moveVoltage(12000);
	// turns to 2nd and 3rd ring
	gyroRotate.setTarget(39, 1200);
	conveyor.moveVoltage(12000);
	// gets 2nd ring
	forwardController.setTarget(2150, 39, 1, 1700);
	conveyor.moveVoltage(12000);
	// turns to 3rd ring
	gyroRotate.setTarget(128, 1000);
	conveyor.moveVoltage(12000);
	// gets 3rd ring
	forwardController.setTarget(650, 128, 1, 700);
	conveyor.moveVoltage(12000);
	// backs up
	forwardController.setTarget(-500, 128, 1, 550);
	conveyor.moveVoltage(12000);
	// turns and drives to go to opposite side of field
	gyroRotate.setTarget(223, 1200);
	intake.moveVoltage(0);
	conveyor.moveVoltage(0);
	forwardController.setTarget(2400, 223, 1, 1200); // 1600 too little
	// turns to 4th ring
	gyroRotate.setTarget(268, 750);
	// gets 4th ring
	intake.moveVoltage(12000);
	conveyor.moveVoltage(0);
	forwardController.setTarget(1100, 268, 1, 800); // 1800 too far
	// turns and gets 5th and 6th ring
	gyroRotate.setTarget(168, 1200);
	conveyor.moveVoltage(12000);
	leftDoinker.openSole();
	forwardController.setTarget(1950, 168, 1, 1400);
	// turns, sweeps corner with leftDoinker
	gyroRotate.setTarget(352, 1100);
	// back into corner
	leftDoinker.closeSole();
	clamp.closeSole();
	conveyor.moveVoltage(-12000);
	// pushes mogo all the way into the corner
	forwardController.setTarget(-800, 373, 1, 500);
	forwardController.setTarget(500, 373, 1, 500);
	clamp.openSole();
	forwardController.setTarget(-700, 373, 1, 600);
	wait(0.25);
	// gets out of corner
	forwardController.setTarget(900, 360, 1, 600);
	clamp.closeSole();
	conveyor.moveVoltage(12000);
	intake.moveVoltage(12000);
	// lines up with last mogo
	forwardController.setTarget(2300, 418, 1, 1000);
	// pushes last mogo into opposite corner
	forwardController.setTarget(3000, 402, 1, 1700);
	// backs up
	intake.moveVoltage(-12000);
	forwardController.setTarget(-500, 390, 1, 500);
	forwardController.setTarget(2000, 410, 1, 600);
	forwardController.setTarget(-1000, 394, 1, 500);

}

void tuneForwardPID() {
	gyroRotate.rotateAbsolute(90);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	teamColor = "red";
	displayInit();
	
	// turns on optical light
	if (colorSensing) {
		opticalSensor.set_led_pwm(100);
	}
	// initializes all tasks
	pros::Task ConveyorTask(conveyorTask, NULL);
	pros::Task LadyBrownTask(ladyBrownTask, NULL);
	pros::Task JamDetectionTask(jamDetectionTask, NULL);
	// vibrates controller to signal that everything is initialized
	controller1.rumble("--..");
}

void autonomous() {
	drive->getModel()->setBrakeMode(AbstractMotor::brakeMode::brake);
	imu->tare_rotation();
	gyroRotate.tareHeading();
	conveyorMode = Auto;
	ladyBrownMode = Auton;
	grabGoal = true;
	colorSensing = true;
	// redTop3Ring();
	// redMogoRush();
	// allianceStakeMacro();
	// redRingRush();
	// blueRingRushElims();
	// red7();
	red6Baker();
	// red7();
	// skillsAuto();
	// runSelectedAuto();
	// getTwoCornerRings(0);
	// bluePosAWP();
	// red6BakerBar();
	// red5Baker();
	// redMogo();
	// redMogo();
	// blue6AWP();
	// left_safe_awp();
	// right_safe_awp();
	// left_safe_awp();
	// skillsAuto();
	// ringRush();

	// rotation PID tuning
	// gyroRotate.tareHeading();
	// gyroRotate.rotateAbsolute(90);
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
	conveyor.moveVoltage(0);
	colorSensing = true;
	jamDetection = false;
	conveyorMode = User;
	drive->getModel()->setBrakeMode(AbstractMotor::brakeMode::coast);
	drive->getModel()->setMaxVoltage(12000);
	ladyBrown.setBrakeMode(AbstractMotor::brakeMode::brake);

	pros::screen::set_pen(COLOR_BLUE);
	rightDoinker.closeSole();

	bool leftDoinkerDown = false;
	bool rightDoinkerDown = false;
	bool doinkerClampClosed = false;
	// grabGoal = true;
	bool intakeDown = false;

	// lady brown toggle conveyorMode to set in grab ring position
	bool setBrown = false;
	bool downBrown = false;
	
	bool aJustPressed = true;
	bool xJustPressed = true;
	bool bJustPressed = true;
	bool upJustPressed = true;

   	while (true) {
		pros::lcd::print(1, "color: %f   ", opticalSensor.get_hue());
		pros::lcd::print(2, "rotation: %i   ", brownRotation);
		pros::lcd::print(3, "Controller right Y: %f   ", controller1.getAnalog(ControllerAnalog::rightY));
		pros::lcd::print(4, "proximity: %i   ", opticalSensor.get_proximity());
 		// chassis
		drive->getModel()->arcade(controller1.getAnalog(ControllerAnalog::leftY), controller1.getAnalog(ControllerAnalog::rightX));

		// clamp
		solenoidCheck(clamp, a, aJustPressed, grabGoal);
		// leftDoinker
		solenoidCheck(leftDoinker, b, bJustPressed, leftDoinkerDown);
		// rightDoinker
		solenoidCheck(rightDoinker, x, xJustPressed, rightDoinkerDown);
		
		
		if (rightDoinkerDown) {
			// doinker clamp
			solenoidCheck(doinkerClamp, up, upJustPressed, doinkerClampClosed);
			intakeDown = false;
			intakeLift.closeSole();
			
		} else {
			// intake lift
			solenoidCheck(intakeLift, up, upJustPressed, intakeDown);
			doinkerClampClosed = false;
			doinkerClamp.closeSole();
		}

		// Intake
		if (r2.isPressed()) {
			intake.moveVoltage(12000);
		} else if (l2.isPressed()) {
			intake.moveVoltage(-12000);
		} else {
			intake.moveVoltage(0);
		}
				
		// 	} else {
		// 		rDrive.moveVoltage(-7200);
		// 		lDrive.moveVoltage(-7200);
		// 	}
		// }

    	pros::delay(20);
 	}
}