#include "main.h"

const u_int16_t forwardCurve       = 10;
const u_int16_t turnCurve          = 3;
const double euler                 = 2.71828;
static bool toggleRedCurve         = false; // toggle red curve
static bool turningRed             = false; // turning curve
static bool forwardRed             = false; // linear curve

/**
 * @brief Set exponential joystick accelerator curve type. Allows for more control of small movements, while maintaining max speed for large movements
 * 
 * @param red Red curve enabled. If not enabled, will use blue graph
 * @param t Joystick damper. Tune value as desired by driver
 * @return the value of the joystick accelerator
 */

int32_t joystick_accelerator(bool red, int8_t input, const double t){
    int16_t value = 0;
    // exponential formula
    if (red) { value = (std::exp(-t / 10) + std::exp((std::abs(input) - 100) / 10) * (1 - std::exp(-t / 10))) * input; } 
    else { value = std::exp(((std::abs(input) - 100) * t) / 1000) * input; }
    return value;
}

void exponential_curve_accelerator(){
    int32_t rightXjoystick = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)); // Axis 1
    int32_t rightYjoystick = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)); // Axis 2
    int32_t leftYjoystick  = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)); // Axis 3
    int32_t leftXjoystick  = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)); // Axis 4
    if(abs(leftYjoystick) < 10) leftYjoystick = 0;
    if(abs(rightYjoystick) < 10) rightYjoystick = 0;

    double turn_val = joystick_accelerator(turningRed, rightXjoystick, turnCurve);
    double forward_val = joystick_accelerator(forwardRed, leftYjoystick, forwardCurve);
    double turnVoltage = turn_val * (12000.0 / 127); 
    double forwardVoltage = forward_val * (12000.0 / 127);
    std:: cout << turnVoltage << std::endl;
    std:: cout <<"linear: " << forwardVoltage << std::endl;
    double left =  forwardVoltage + turnVoltage;
    double right = forwardVoltage - turnVoltage;
    frontleft.move_voltage(left);
    frontright.move_voltage(right);
    midleft.move_voltage(left);
    midright.move_voltage(right);
    backleft.move_voltage(left);
    backright.move_voltage(right);

}

void dt_Control(){
    int32_t rightXjoystick = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)); // Axis 1
    int32_t rightYjoystick = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)); // Axis 2
    int32_t leftYjoystick  = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)); // Axis 3
    int32_t leftXjoystick  = (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)); // Axis 4
    if(abs(leftYjoystick) < 10) leftYjoystick = 0;
    if(abs(rightYjoystick) < 10) rightYjoystick = 0;


    int32_t left = (leftYjoystick + rightXjoystick) * (12000.0 / 127);
    int32_t right = (leftYjoystick - rightXjoystick) * (12000.0 / 127);
    frontleft.move_voltage(-left);
    frontright.move_voltage(right);
    midleft.move_voltage(-left);
    midright.move_voltage(right);
    backleft.move_voltage(-left);
    backright.move_voltage(right);
}