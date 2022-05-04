#include "main.h"

okapi::Controller controller(okapi::ControllerId::master);
auto drivebase = okapi::ChassisControllerBuilder().withMotors({8, 9, -10}, {-1, -2, 3}).withDimensions({okapi::AbstractMotor::gearset::green}, {{4_in, 15_in}, okapi::imev5GreenTPR * 60.0 / 84.0}).withOdometry(okapi::StateMode::FRAME_TRANSFORMATION).buildOdometry();
okapi::Motor arm(5, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::degrees);
okapi::Motor ringleLift(20, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::degrees);
pros::Gps gps(18, 5 * 0.0254, 1 * 0.0254);
okapi::RotationSensor rot(19);
pros::ADIDigitalOut claw('D');
pros::ADIDigitalOut forklift('C');
pros::ADIDigitalIn clawSwitch1('A');
pros::ADIDigitalIn clawSwitch2('B');

const auto base = drivebase->getModel();
bool isRingleLiftOn = false, isClawClosed = false, isForkliftUp = true;
bool wasRinglePrevPressed = false, wasClawPrevPressed = false, wasForkliftPrevPressed = false;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
    const char *autons[] = {"Goal Rush", "Half WP", "Do Nothing"};
    arms::selector::init(360, 1, autons);
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
    if (pros::ADIDigitalIn('h').get_value() == 5)
    {
        printf("Auton h\n");
        printf("%d,%d\n", pros::ADIDigitalIn('h').get_value(), pros::ADIDigitalIn('g').get_value());
        drivebase->moveDistanceAsync(40_in);
        while (!((clawSwitch1.get_value() && clawSwitch2.get_value()) || drivebase->isSettled()))
            ;
        isClawClosed = true;
        claw.set_value(isClawClosed);
        arm.moveAbsolute(-620, 70);
        drivebase->moveDistance(25_in);
        arm.moveAbsolute(-50, 70);
        drivebase->setMaxVelocity(100);
        pros::delay(1000);
        drivebase->turnAngle(-90_deg);
        drivebase->moveDistance(-1_ft);
        ringleLift.moveRelative(90, 100);
        isForkliftUp = true;
        forklift.set_value(isForkliftUp);
        drivebase->moveDistanceAsync(2.5_ft);
        pros::delay(1500);
        isForkliftUp = false;
        forklift.set_value(isForkliftUp);
        pros::delay(400);
        drivebase->moveDistance(-2_ft);
        ringleLift.moveVelocity(100);
        pros::delay(3000);
        ringleLift.moveVelocity(0);
    }

    // Short neutural goal only
    else if (pros::ADIDigitalIn('g').get_value() == 5)
    {
        printf("Auton g");
        drivebase->getModel()->arcade(.5, 0);
        drivebase->setMaxVelocity(200);
        drivebase->moveDistanceAsync(56_in);
        while (!(clawSwitch1.get_value() || clawSwitch2.get_value() || drivebase->isSettled()))
            ;
        isClawClosed = true;
        claw.set_value(isClawClosed);
        arm.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
        arm.moveVelocity(0);
        drivebase->moveDistance(-36_in);
    }

    // Alliance goal only
    else if (pros::ADIDigitalIn('f').get_value() == 5)
    {
        printf("Auton f\n");
        drivebase->setMaxVelocity(50);
        isForkliftUp = true;
        forklift.set_value(isForkliftUp);
        pros::delay(300);
        drivebase->moveDistanceAsync(-20_in);
        pros::delay(600);
        isForkliftUp = false;
        forklift.set_value(isForkliftUp);
        pros::delay(700);
        drivebase->moveDistance(16_in);
        pros::delay(1000);
        ringleLift.moveVelocity(100);
        pros::delay(250);
        ringleLift.moveVelocity(0);
        pros::delay(1000);
        ringleLift.moveVelocity(100);
        pros::delay(2500);
        ringleLift.moveVelocity(0);
    }

    // Center neutural goal only
    else if (pros::ADIDigitalIn('e').get_value() == 5)
    {
        printf("Auton e\n");
        drivebase->getModel()->arcade(.5, 0);
        drivebase->setMaxVelocity(200);
        drivebase->moveDistanceAsync(60_in);
        while (!(clawSwitch1.get_value() || clawSwitch2.get_value() || drivebase->isSettled()))
            ;
        isClawClosed = true;
        claw.set_value(isClawClosed);
        drivebase->moveDistance(-50_in);
    }
    // Skills
    else if (pros::ADIDigitalIn('d').get_value() == 5)
    {
        printf("Auton d\n");
        isForkliftUp = false;
        forklift.set_value(isForkliftUp);
        drivebase->moveDistance(-20_in);
        isForkliftUp = true;
        claw.set_value(isForkliftUp);
        drivebase->moveDistanceAsync(8_in);
        pros::delay(1000);
        ringleLift.moveVelocity(200);
        // drivebase->turnAngle();
        //
        arm.moveAbsolute(-50, 70);
        arm.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
        pros::delay(1000);
        drivebase->moveDistance(10_in);
        gps.get_status();
        pros::delay(1000);
        auto gpsState = gps.get_status();
        drivebase->setState({okapi::QLength(gpsState.x), okapi::QLength(gpsState.y), okapi::QAngle(gpsState.yaw * 2_pi / 360.0) - 90_deg});
        drivebase->setMaxVelocity(100);
        // Push top yellow
        drivebase->driveToPoint({36.008_in, 25_in});
        // Push mid yellow
        drivebase->driveToPoint({-36.008_in, -25_in});
        // Push bottom yellow
        drivebase->driveToPoint({-36.008_in, 45_in});
    }
    else if (abs(arms::selector::auton) == 1)
    {
        printf("Goal Rush\n");
        drivebase->getModel()->arcade(.5, 0);
        drivebase->setMaxVelocity(200);
        drivebase->moveDistanceAsync(56_in);
        while (!(clawSwitch1.get_value() || clawSwitch2.get_value() || drivebase->isSettled()))
            ;
        isClawClosed = true;
        claw.set_value(isClawClosed);
        arm.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
        arm.moveVelocity(0);
        drivebase->moveDistance(-36_in);
    }
    else if (abs(arms::selector::auton) == 2)
    {
        printf("Half WP\n");
        drivebase->setMaxVelocity(50);
        isForkliftUp = true;
        forklift.set_value(isForkliftUp);
        pros::delay(300);
        drivebase->moveDistanceAsync(-20_in);
        pros::delay(600);
        isForkliftUp = false;
        forklift.set_value(isForkliftUp);
        pros::delay(700);
        drivebase->moveDistance(16_in);
        pros::delay(1000);
        ringleLift.moveVelocity(100);
        pros::delay(250);
        ringleLift.moveVelocity(0);
        pros::delay(1000);
        ringleLift.moveVelocity(100);
        pros::delay(2500);
        ringleLift.moveVelocity(0);
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
        base->arcade(controller.getAnalog(okapi::ControllerAnalog::leftY), controller.getAnalog(okapi::ControllerAnalog::rightX));

        if (controller.getDigital(okapi::ControllerDigital::A) && !wasRinglePrevPressed)
            isRingleLiftOn = !isRingleLiftOn;
        wasRinglePrevPressed = controller.getDigital(okapi::ControllerDigital::A);

        ringleLift.moveVelocity(
            controller.getDigital(okapi::ControllerDigital::B) ? -100
            : isRingleLiftOn                                   ? 100
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
