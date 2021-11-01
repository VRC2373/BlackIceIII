#include "main.h"

okapi::Controller controller(okapi::ControllerId::master);
auto drivebase = okapi::ChassisControllerBuilder().withMotors({-11, 12, -13}, {18, -19, 20}).withDimensions({okapi::AbstractMotor::gearset::green}, {{4_in, 15_in}, okapi::imev5GreenTPR}).build();
okapi::Motor arm(15, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations);
okapi::Motor ringleLift(1, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::degrees);
pros::ADIDigitalOut claw('A');
pros::ADIDigitalOut forklift('B');
pros::ADIDigitalIn clawSwitch('C');

const auto base = drivebase -> getModel();
bool isRingleLiftOn = false, isClawClosed = false, isForkliftUp = true;
bool wasRinglePrevPressed = false, wasClawPrevPressed = false, wasForkliftPrevPressed = false;
float pctSpeed = 1.0f;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	arm.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

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
void autonomous()
{
	drivebase->moveDistanceAsync(-40_in);
	arm.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	arm.tarePosition();
	arm.moveAbsolute(-640, 100);
	okapi::Timer timer;
	while (!(clawSwitch.get_value() || drivebase->isSettled()))
		;
	isClawClosed = true;
	claw.set_value(isClawClosed);
	arm.moveAbsolute(-620, 70);
	drivebase->moveDistance(35_in);
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
void opcontrol()
{
	while (true)
	{
		pctSpeed = isRingleLiftOn ? 0.625f : 1.0f;
		base->arcade(controller.getAnalog(okapi::ControllerAnalog::leftY) * pctSpeed, controller.getAnalog(okapi::ControllerAnalog::rightX) * .625);

		if (controller.getDigital(okapi::ControllerDigital::A) && !wasRinglePrevPressed)
			isRingleLiftOn = !isRingleLiftOn;
		wasRinglePrevPressed = controller.getDigital(okapi::ControllerDigital::A);

		ringleLift.moveVelocity(
			controller.getDigital(okapi::ControllerDigital::B) ? -100
			: isRingleLiftOn								   ? 100
															   : 0);

		if (controller.getDigital(okapi::ControllerDigital::R1))
			arm.moveVelocity(100);
		else if (controller.getDigital(okapi::ControllerDigital::R2))
			arm.moveVelocity(-100);
		else
			arm.moveVelocity(0);

		if (controller.getDigital(okapi::ControllerDigital::L1) && !wasClawPrevPressed)
		{
			isClawClosed = !isClawClosed;
			claw.set_value(isClawClosed);
		}
		wasClawPrevPressed = controller.getDigital(okapi::ControllerDigital::L1);
		if (controller.getDigital(okapi::ControllerDigital::L2) && !wasForkliftPrevPressed)
		{
			forklift.set_value(isForkliftUp);
			isForkliftUp = !isForkliftUp;
		}
		wasForkliftPrevPressed = controller.getDigital(okapi::ControllerDigital::L2);

		pros::delay(20);
	}
}
