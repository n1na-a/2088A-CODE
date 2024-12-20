#include "main.h"


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
    inertial.tare_rotation();
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

double globalTheta = 0;

int current_robot_heading() {
    globalTheta = fmod(inertial.get_rotation(), 360);
    while (globalTheta < 0) {
        globalTheta += 360;
    }
    while (globalTheta > 360) {
        globalTheta -= 360;
    }
    return globalTheta;
}

int sgn(double num) {
    return (num < 0) ? -1 : ((num > 0) ? 1 : 0);
}

int find_min_angle(int targetHeading, double currentRobotHeading) {
    double turnAngle = targetHeading - currentRobotHeading;
    if (turnAngle > 180 || turnAngle < -180) {
        turnAngle = turnAngle - (sgn(turnAngle) * 360);
    }
    return turnAngle;
}

void moveMotors(double left, double right) {
    pros::lcd::print(2, "Motors moving at %f ", left);
    frontleft.move_voltage(left * (12000.0 / 127));
    frontright.move_voltage(right * (12000.0 / 127));
    midleft.move_voltage(left * (12000.0 / 127));
    midright.move_voltage(right * (12000.0 / 127));
    backleft.move_voltage(left * (12000.0 / 127));
    backright.move_voltage(right * (12000.0 / 127));
}


void intakeAuton(double speed, double time) {
    intake.move_voltage(speed * (12000.0 / 127));
    pros::delay(time * 1000);
    intake.move_voltage(0);
}


double r_kp = 5;
double r_ki = 0;
double r_kd = 45;
double r_derivative = 0;
double r_integral = 0;
double r_error = 0;
double r_prev_error = 0;
double r_iterator = 0;
double r_failsafe = 0;
double r_max_speed = 90;
                 
void reset_rotation_values() {
    r_derivative = 0;
    r_integral = 0;
    r_error = 0;
    r_prev_error = 0;
    r_iterator = 0;
    r_failsafe = 0;
}


double get_min_angle_error_pid(float angle1, float angle2, bool radians){
    float max = radians ? 2 * M_PI : 360;
    float half = radians ? M_PI : 180;
    angle1 = fmod(angle1, max);
    angle2 = fmod(angle2, max);
    float error = angle1 - angle2;
    if (error > half) error -= max;
    else if (error < -half) error += max;
    return error;
}


double compute_rotation_pid(double current, double target) {
  r_error = target - current;
  r_derivative = r_error - r_prev_error;
  if (r_ki != 0){ r_integral += r_error; }
  if (r_error == 0 || r_error > target){ r_integral = 0; }


  double output = (r_kp * r_error) + (r_integral * r_ki) + (r_derivative * r_kd);
  if (output * (12000.0 / 127) >= r_max_speed * (12000.0 / 127)) { output = r_max_speed; }
  if (output * (12000.0 / 127) <= -r_max_speed * (12000.0 / 127)) { output = -r_max_speed; }
  r_prev_error = r_error;
  return output;
}


void turn(double target, double speed) {
  frontleft.set_zero_position(0);
  frontright.set_zero_position(0);

  reset_rotation_values();
  r_max_speed = speed;
  double local_timer = 0;
  while (true){
    double currentPos = inertial.get_rotation();
    double vol = compute_rotation_pid(currentPos, target);
    moveMotors(vol, -vol);

    if (fabs(r_error) < 3) { r_iterator++; } else { r_iterator = 0; }
    if (fabs(r_iterator) >= 5){
      moveMotors(0, 0);
      break;
    }
    pros::delay(10);
  }
}


double m_kp = 3.0;
double m_ki = 0.0;
double m_kd = 25.0;
double m_derivative = 0;
double m_integral = 0;
double m_error = 0;
double m_prev_error = 0;
double m_iterator = 0;
double m_failsafe = 0;
double m_max_speed = 110;


double compute_movement_pid(double current, double target) {
    m_error = target - current;
    m_derivative = m_error - m_prev_error;
    if (m_ki != 0){
        m_integral += m_error;
    }

    double output = (m_kp * m_error) + (m_integral * m_ki) + (m_derivative * m_kd);
    if (output * (12000.0 / 127) > m_max_speed * (12000.0 / 127)) output = m_max_speed;
    if (output * (12000.0 / 127) < -m_max_speed * (12000.0 / 127)) output = -m_max_speed;
    m_prev_error = m_error;
    pros::lcd::print(3, "Output Speed: %f ", output * (12000.0 / 127));
    return output;
}


void reset_movement_values() {
    m_derivative = 0;
    m_integral = 0;
    m_error = 0;
    m_prev_error = 0;
    m_iterator = 0;
    m_failsafe = 0;
}


void move(double target, double speed) {
  frontleft.set_zero_position(0);
  reset_movement_values();
  double POSITION_TARGET = target; bool is_backwards = false; int8_t cd = 0;
  m_max_speed = speed;
  double local_timer = 0;
  double targetHeading = inertial.get_rotation();
  target *= 50;
  while (true){
    pros::lcd::print(1, "Target: %f ", target);
    double avgPos = frontleft.get_position();
    pros::lcd::print(4, "Average Position: %f ", avgPos);
    double avg_voltage_req = compute_movement_pid(avgPos, target); // compute proportional integral derivative controller on filtered pos
    double headingAssist = find_min_angle(targetHeading, current_robot_heading());
    cd++; if (cd <= 2){ moveMotors(0, 0); continue;} // initial timer to keep robot from oscillating
    if (target < 0) { is_backwards = true; } else { is_backwards = false; }
    double l_output = 0; double r_output = 0;
    l_output = avg_voltage_req + headingAssist;
    r_output = avg_voltage_req - headingAssist;

    moveMotors(l_output, r_output);

    if (fabs(m_error) < 100){ m_iterator++; } else { m_iterator = 0;}
    if (fabs(m_iterator) > 10){
        moveMotors(0, 0);
        break;
    }
    pros::delay(10);
  }
}


    // pros::delay(500);
    // piston.set_value(true);
    // pros::delay(500);
    // move(24, 90);
    // turn(-90, 60);
    // move(-6, 80);
    // piston.set_value(false);
    // pros::delay(500);
    // turn(100, 90);
    // intake.move_voltage(12000);
    // move(46, 90);
    // pros::delay(1000);
    // intake.move_voltage(0);
    // turn(125, 80);
    // intake.move_voltage(12000);
    // move(36, 80);
    // pros::delay(1000);
    // intake.move_voltage(0);
    // conveyor.move_voltage(0);
    // piston.set_value(false);
    // move(10, 127);

    

//}




void autonomous() {
    piston.set_value(true);
    pros::delay(500);
    move(-20, 90);
    move(-9, 40);
    pros::delay(800);
    piston.set_value(false);
    pros::delay(500);
    conveyor.move_voltage(-12000);
    pros::delay(2000);
    conveyor.move_voltage(0);
    pros::delay(500);
    turn(90, 60);
    pros::delay(500);
    move(18, 100);
    intake.move_voltage(12000);
    pros::delay(2000);
    intake.move_voltage(0);


    // turn(90, 90);
    // turn(-90, 90);
    // turn(90, 90);

  
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


void intakeControl(double speed) {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        intake.move_voltage(speed * (12000.0 / 127));
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        intake.move_voltage(-speed * (12000.0 / 127));
    } else {
        intake.move_voltage(0);
    }
}


void conveyorControl(double speed) {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        conveyor.move_voltage(speed * (12000.0 / 127));
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        conveyor.move_voltage(-speed * (12000.0 / 127));
    } else {
        conveyor.move_voltage(0);
    }
}


bool pistonStatus = false;

void use_piston() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        pistonStatus = !pistonStatus;
        piston.set_value(pistonStatus);
    }
}



void opcontrol() {
    while (true){
        exponential_curve_accelerator();
        use_piston();
        intakeControl(127);
        conveyorControl(120);
        pros::delay(10);
    }
}
