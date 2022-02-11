#include "main.h"
#include "math.h"
#include "okapi/api.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

 pros::Motor right_intake(5); //or 18
 pros::Motor left_intake(18, true); //or 5
 pros::Motor blue(4, pros::E_MOTOR_GEARSET_06);
 pros::Motor red(10, pros::E_MOTOR_GEARSET_06);
 pros::Motor left_back_mtr(11, true);
 pros::Motor right_back_mtr(20);
 pros::Motor left_front_mtr(1);
 pros::Motor right_front_mtr(8, true);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

//1852 1860
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


void intakeBck (int ticks, int speed) {
	int i = 0;
	bool x = true;
	while (x == true) {
		right_intake.move_voltage(speed);
		left_intake.move_voltage(speed);
		pros::delay(10);
		i++;
		if (i >= ticks) {
			x = false;
		}
	}
}

void intakeFwd (int ticks, int speed) {
	int i = 0;
	bool x = true;
	while (x >= true) {
		right_intake.move_voltage(-speed);
		left_intake.move_voltage(-speed);
		pros::delay(10);
		i++;
		if (i >= ticks) {
			x = false;
		}
	}
}

void poopFwd (int ticks, int speed) {
	int i = 0;
	int x = true;
	while (x == true) {
		blue.move_voltage(speed);
		red.move_voltage(speed);
		pros::delay(10);
		i++;
		if (i >= ticks) {
			x = false;
		}
	}
}

void poopBck (int ticks, int speed) {
	int i = 0;
	bool x = true;
	while (x == true) {
		blue.move_voltage(speed);
		red.move_voltage(-speed);
		pros::delay(10);
		i++;
		if (i >= ticks) {
			x = false;
		}
	}
}

void score (int ticks, int speed) {
	int i = 0;
	bool x = true;
	while (x == true) {
		blue.move_voltage(-speed);
		red.move_voltage(-speed);
		pros::delay(10);
		i++;
		if (i >= ticks) {
			x = false;
		}
	}
}

void stopBlueRed() {
	blue.move_voltage(0);
	red.move_voltage(0);
}

void stopIntake() {
	right_intake.move_voltage(0);
	left_intake.move_voltage(0);
}

void storeBall() {
	intakeBck(40, 12000);
	stopIntake();
	score(10, 12000);
	stopBlueRed();
}

void score3() {
	intakeBck(100, 12000);
	score(100, 12000);
	stopIntake();
	stopBlueRed();
}

void score2() {
	intakeBck(75, 12000);
	score(75, 12000);
	stopIntake();
	stopBlueRed();
}

//chassis
using namespace okapi;

std::shared_ptr<ChassisController> baseController = ChassisControllerBuilder()
.withMotors(11, 20)
.withDimensions(AbstractMotor::gearset::green, {{5.25_in, 12.5_in}, imev5GreenTPR})
.build();

void autonomous() {
  //first goal
  baseController->setMaxVelocity(100);
	intakeBck(100, 12000);
  score(15, 12000);
  stopBlueRed();
	baseController->moveDistance(2_ft);
  score(10, 12000);
	stopIntake();
	stopBlueRed();
	baseController->turnAngle(-123_deg);
	baseController->moveDistance(2.1_ft);
  intakeBck(90, 12000);
  score(100, 12000);
	stopIntake();
  stopBlueRed();

  //second goal
	baseController->moveDistance(-3.4_ft);
  poopFwd(100, 12000);
  intakeFwd(100, 12000);
	stopIntake();
  stopBlueRed();
	baseController->turnAngle(130_deg);
	intakeBck(50, 12000);
	baseController->moveDistance(2.5_ft);
  stopIntake();
  baseController->turnAngle(-87_deg);
  intakeBck(50, 12000);
  baseController->moveDistance(2.4_ft);
  stopIntake();
  intakeBck(90, 12000);
  score(100, 12000);
  stopIntake();
  stopBlueRed();

  //third goal
  baseController->moveDistance(-1.4_ft);
  poopFwd(100, 12000);
  intakeFwd(100, 12000);
	stopIntake();
  stopBlueRed();
  baseController->turnAngle(85_deg);
  intakeBck(100, 12000);
  baseController->moveDistance(4_ft);
  stopIntake();
  baseController->turnAngle(-60_deg);
  baseController->moveDistance(2.2_ft);
  intakeBck(100, 12000);
  score(110,12000);
  stopIntake();
  stopBlueRed();

  //fourth goal
  baseController->moveDistance(-2.8_ft);
  poopFwd(100, 12000);
  intakeFwd(100, 12000);
	stopIntake();
  stopBlueRed();
  baseController->turnAngle(135_deg);
  intakeBck(100, 12000);
  baseController->moveDistance(2.8_ft);
  stopIntake();
  baseController->turnAngle(90_deg);
  baseController->moveDistance(2_ft);
  intakeBck(100, 12000);
  score(110,12000);
  stopIntake();
  stopBlueRed();

  //fifth goal
  baseController->moveDistance(-1.4_ft);
  poopFwd(100, 12000);
  intakeFwd(100, 12000);
	stopIntake();
  stopBlueRed();
  baseController->turnAngle(85_deg);
  intakeBck(100, 12000);
  baseController->moveDistance(4_ft);
  stopIntake();
  baseController->turnAngle(-60_deg);
  baseController->moveDistance(2.2_ft);
  intakeBck(100, 12000);
  score(110,12000);
  stopIntake();
  stopBlueRed();

  //sixth goal
  baseController->moveDistance(-2.8_ft);
  poopFwd(100, 12000);
  intakeFwd(100, 12000);
	stopIntake();
  stopBlueRed();
  baseController->turnAngle(135_deg);
  intakeBck(100, 12000);
  baseController->moveDistance(2.8_ft);
  stopIntake();
  baseController->turnAngle(90_deg);
  baseController->moveDistance(2_ft);
  intakeBck(100, 12000);
  score(110,12000);
  stopIntake();
  stopBlueRed();

  //seventh goal
  baseController->moveDistance(-1.4_ft);
  poopFwd(100, 12000);
  intakeFwd(100, 12000);
  stopIntake();
  stopBlueRed();
  baseController->turnAngle(85_deg);
  intakeBck(100, 12000);
  baseController->moveDistance(4_ft);
  stopIntake();
  baseController->turnAngle(-60_deg);
  baseController->moveDistance(2.2_ft);
  intakeBck(100, 12000);
  score(110,12000);
  stopIntake();
  stopBlueRed();

  //eighth goal
  baseController->moveDistance(-2.8_ft);
  poopFwd(100, 12000);
  intakeFwd(100, 12000);
  stopIntake();
  stopBlueRed();
  baseController->turnAngle(135_deg);
  intakeBck(100, 12000);
  baseController->moveDistance(2.8_ft);
  stopIntake();
  baseController->turnAngle(90_deg);
  baseController->moveDistance(2_ft);
  intakeBck(100, 12000);
  score(110,12000);
  stopIntake();
  stopBlueRed();


	//storeBall();
	//pros::delay(5000);
	//storeBall();



	/*intakeFwd(1000, 12000);
	stopIntake();
	score(1000, 12000);
	stopBlueRed();
	poopBck(1000, 12000);
	stopBlueRed();
	poopFwd(1000, 12000);
	stopBlueRed();*/
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	while (true) {
		left_back_mtr.move(master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_RIGHT_X)/2);
		left_front_mtr.move(master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_RIGHT_X)/2);
		right_back_mtr.move(master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_RIGHT_X)/2);
		right_front_mtr.move(master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_RIGHT_X)/2);
		pros::delay(20);

		//non continuous

		if (master.get_digital(DIGITAL_R1)) { //intake in
			right_intake.move_voltage(12000);
			left_intake.move_voltage(12000);
		}

		else if (master.get_digital(DIGITAL_R2)) { //intake out
			right_intake.move_voltage(-12000);
			left_intake.move_voltage(-12000);
		}

		else {
			right_intake.move_voltage(0);
			left_intake.move_voltage(0);
		}

		if (master.get_digital(DIGITAL_L1)) { //poop up
			blue.move_voltage(-12000);
			red.move_voltage(-12000);
		} else if (master.get_digital(DIGITAL_L2)) { //poop bck
			blue.move_voltage(12000);
			red.move_voltage(-12000);
		}	else if (master.get_digital(DIGITAL_X)) { //poop fwd
			blue.move_voltage(12000);
			red.move_voltage(12000);
		} else {
			blue.move_voltage(0);
			red.move_voltage(0);
		}
		pros::delay(20);
	}
}
