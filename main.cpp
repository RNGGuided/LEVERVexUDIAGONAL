#include "main.h"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"
#include "diagonal_tracking_wheel.hpp"

ASSET(test_txt);

// ---------------- CONTROLLER ----------------
pros::Controller master(pros::E_CONTROLLER_MASTER);

// ---------------- DRIVETRAIN ----------------
pros::MotorGroup left_motors({1, -2, 3, -4, 5}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({-11, 12, -13, 14, -15}, pros::MotorGearset::blue);

// ---------------- MECHANISMS ----------------
pros::Motor intake(10);
pros::Motor lever(8);
pros::ADIDigitalOut Middle('H');
pros::adi::Encoder diagLeft('A', 'B',true); 
pros::adi::Encoder diagRight('C', 'D',true);
// ---------------- LEVER CONSTANTS ----------------
constexpr double LEVER_MIN = 0;
constexpr double LEVER_MAX = 270;

constexpr double LEVER_START_BOOST = 10;   // first 20 degrees
constexpr double LEVER_SNAP_ZONE  = 10;    // last 10 degrees

constexpr int LEVER_SPEED_FAST = 70;      // middle goal ON
constexpr int LEVER_SPEED_SLOW = 30;       // middle goal OFF
constexpr int LEVER_SPEED_BOOST = 127;     // full throttle

// ---------------- LEMLIB ----------------
lemlib::Drivetrain drivetrain(
    &left_motors,
    &right_motors,
    10.7,
    3,
    457.142857143,
    2
);

pros::Imu imu(6);
// Virtual tracking wheels built from diagonal pods
// Virtual tracking wheels built from diagonal pods
DiagonalTrackingWheel virtualVertical(
    &diagLeft,
    &diagRight,
    true,   // forward
    2.0,    // wheel diameter (inches)
    0       // offset (we'll tune later) (x Axis cancels negative and positive)
);

DiagonalTrackingWheel virtualHorizontal(
    &diagLeft,
    &diagRight,
    false,  // lateral
    2.0,
    0  //(WILL end up with) (DISTANCEFROMCENTERY+DINSTANCEFROMCENTERY)/sqrt(2) = offset of wheels theoretically
);



lemlib::OdomSensors sensors(
    &virtualVertical,     // vertical (forward) (Movements are converted in Y values from diagonals)
    nullptr,
    &virtualHorizontal,   // horizontal (lateral) (Movements are converted in x values from diagonals)
    nullptr,
    &imu
);


lemlib::ControllerSettings lateral_controller(
    10, 0, 3,
    0,
    0, 0,
    0, 0,
    0
);

lemlib::ControllerSettings angular_controller(
    2, 0, 10,
    3,
    1, 100,
    3, 500,
    0
);

lemlib::Chassis chassis(
    drivetrain,
    lateral_controller,
    angular_controller,
    sensors
);

// ---------------- INITIALIZE ----------------
void initialize() {
    pros::lcd::initialize();

    lever.set_gearing(pros::E_MOTOR_GEARSET_18);
    lever.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    lever.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    lever.tare_position();

    chassis.calibrate();

    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %.2f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %.2f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %.2f", chassis.getPose().theta);
            pros::lcd::print(3, "Lever: %.1f", lever.get_position());
            pros::delay(50);
        }
    });
}

// ---------------- AUTON ----------------
void autonomous() {
    chassis.setPose(-31.224, -39.436, 0);
    chassis.moveToPoint(-31.42, 39.86, 10000);
    chassis.turnToHeading(180, 1000);
    chassis.moveToPoint(-31.42, -39.436, 10000);
}

// ---------------- TELEOP ----------------
void opcontrol() {
    static bool middleState = false;
    static bool lastB = false;

    static bool leverActive = false;
    static bool leverGoingUp = true;
    static bool lastR1 = false;
    static bool lastR2 = false;

    while (true) {
        // --------- ARCADE DRIVE ---------
        int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn    = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        int leftPower  = forward + turn;
        int rightPower = forward - turn;

        if (abs(leftPower) < 5) leftPower = 0;
        if (abs(rightPower) < 5) rightPower = 0;

        leftPower  = std::clamp(leftPower, -127, 127);
        rightPower = std::clamp(rightPower, -127, 127);

        left_motors.move(leftPower);
        right_motors.move(rightPower);

        // --------- INTAKE ---------
        intake.move(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) ? 127 : 0);

        // --------- LEVER ONE-SHOT (BOOST → CRUISE → SNAP) ---------
        bool r1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool r2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

        bool r1Pressed = r1 && !lastR1;
        bool r2Pressed = r2 && !lastR2;

        if ((r1Pressed || r2Pressed) && !leverActive) {
            leverActive = true;
            leverGoingUp = true;
        }

        if (leverActive) {
            double pos = lever.get_position();
            int speed;

            if (leverGoingUp) {
                if (pos <= LEVER_START_BOOST) {
                    speed = LEVER_SPEED_BOOST;                     // launch
                } 
                else if (pos >= LEVER_MAX - LEVER_SNAP_ZONE) {
                    speed = LEVER_SPEED_BOOST;                     // snap
                } 
                else {
                    speed = middleState ? LEVER_SPEED_FAST
                                        : LEVER_SPEED_SLOW;        // cruise
                }

                lever.move_absolute(LEVER_MAX, speed);

                if (pos >= LEVER_MAX - 2) {
                    leverGoingUp = false;
                }
            } 
            else {
                int downSpeed = middleState ? LEVER_SPEED_FAST
                                            : LEVER_SPEED_SLOW;

                lever.move_absolute(LEVER_MIN, downSpeed);

                if (pos <= LEVER_MIN + 2) {
                    leverActive = false;
                }
            }
        } 
        else {
            lever.move_absolute(lever.get_position(), 50);
        }

        lastR1 = r1;
        lastR2 = r2;

        // --------- PNEUMATIC TOGGLE (B) ---------
        bool currentB = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        if (currentB && !lastB) {
            middleState = !middleState;
            Middle.set_value(middleState);
        }
        lastB = currentB;

        pros::delay(10);
    }
}