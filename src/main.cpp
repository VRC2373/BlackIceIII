#include "main.h"

okapi::Controller controller(okapi::ControllerId::master);
auto drivebase = okapi::ChassisControllerBuilder().withMotors({-11, 12, -13}, {18, -19, 20}).withDimensions({okapi::AbstractMotor::gearset::green}, {{4_in, 15_in}, okapi::imev5GreenTPR}).withOdometry(okapi::StateMode::CARTESIAN).buildOdometry();
okapi::Motor arm(15, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor ringleLift(1, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::degrees);
pros::Gps gps(2, 3.5 * 0.0254, 0.75 * 0.0254);
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
	// Short neutural goal and alliance goal
	if (pros::ADIDigitalIn('h').get_value())
	{
		drivebase->moveDistanceAsync(-40_in);
		arm.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
		arm.tarePosition();
		arm.moveAbsolute(-640, 100);
		while (!(clawSwitch.get_value() || drivebase->isSettled()))
			;
		isClawClosed = true;
		claw.set_value(isClawClosed);
		arm.moveAbsolute(-620, 70);
		drivebase->moveDistance(25_in);
		pros::delay(300);
		drivebase->turnAngle(-90_deg);
		drivebase->moveDistance(-1_ft);
		ringleLift.moveRelative(90, 100);
		isForkliftUp = true;
		forklift.set_value(isForkliftUp);
		drivebase->moveDistance(2.5_ft);
		isForkliftUp = false;
		forklift.set_value(isForkliftUp);
		drivebase->moveDistance(-2_ft);
		ringleLift.moveVelocity(100);
		pros::delay(3000);
		ringleLift.moveVelocity(0);
	}

	// Short neutural goal only
	else if (pros::ADIDigitalIn('g').get_value())
	{
		drivebase->setMaxVelocity(200);
		drivebase->moveDistanceAsync(-40_in);
		// pros::delay(200);
		// drivebase->setMaxVelocity(200);
		arm.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
		arm.tarePosition();
		arm.moveAbsolute(-640, 100);
		while (!(clawSwitch.get_value() || drivebase->isSettled()))
			;
		isClawClosed = true;
		claw.set_value(isClawClosed);
		arm.moveAbsolute(-620, 70);
		drivebase->moveDistance(30_in);
	}

	// Alliance goal only
	else if (pros::ADIDigitalIn('f').get_value())
	{
		printf("Auton f\n");
		arm.tarePosition();
		arm.moveAbsolute(-640, 100);
		ringleLift.moveRelative(90, 100);
		isForkliftUp = true;
		forklift.set_value(isForkliftUp);
		pros::delay(500);
		drivebase->setMaxVelocity(100);
		drivebase->moveDistance(20_in);
		isForkliftUp = false;
		forklift.set_value(isForkliftUp);
		pros::delay(700);
		drivebase->moveDistance(-1.5_ft);
		ringleLift.moveVelocity(100);
		pros::delay(3000);
		ringleLift.moveVelocity(0);
	}

	// Center neutural goal only
	else if (pros::ADIDigitalIn('e').get_value())
	{
		drivebase->moveDistanceAsync(-60_in);
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

	else if (pros::ADIDigitalIn('d').get_value())
	{
		arm.tarePosition();
		arm.moveAbsolute(-640, 100);
		pros::delay(1000);
		drivebase->moveDistanceAsync(-20_in);
		while (!(clawSwitch.get_value() || drivebase->isSettled()))
			pros::delay(20);
		isClawClosed = true;
		claw.set_value(isClawClosed);
		arm.moveAbsolute(-50, 70);
		arm.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
		pros::delay(1000);
		drivebase->moveDistance(10_in);
		gps.get_status();
		pros::delay(500);
		auto gpsState = gps.get_status();
		drivebase->setState({okapi::QLength(gpsState.x), okapi::QLength(gpsState.y), okapi::QAngle(gpsState.yaw * 2_pi / 360.0) - 90_deg});
		// Push top yellow
		drivebase->driveToPoint({25_in, 36.008_in});
		// Push mid yellow
		drivebase->driveToPoint({-25_in, -36.008_in});
		// Push bottom yellow
		drivebase->driveToPoint({45_in, -36.008_in});
	}
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
