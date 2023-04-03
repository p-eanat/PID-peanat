/* 
 * Peanat's PID library made for Jeremy the robot, but should work well for all other robot needs too!
 * Notable features: custom initial output, reverse mode, integral windup prevention, deadband limits
 * Date created: Aug. 16, 2022
 * Version: 1.0
 * Date modified: Aug. 18, 2022
 */

#include "Arduino.h"
#include "PID_peanut.h"

PID::PID(double user_kp, double user_ki, double user_kd, double user_init_output, bool user_reverse, unsigned long user_sample_time, double user_min_output, double user_max_output, double user_deadband) {
    kp = user_kp;
    ki = user_ki;
    kd = user_kd;
    init_output = user_init_output;
    reverse = user_reverse;
    sample_time = user_sample_time;
    min_output = user_min_output;
    max_output = user_max_output;
    deadband = user_deadband;
    first_run = true;
    cumulativeError = 0.0;

    // Reverse coefficients if in reverse mode
    if (reverse == true) {
        kp *= -1.0;
        ki *= -1.0;
        kd *= -1.0;
    }
}

double PID::compute(double user_setpoint, double user_input) {
    // If first run, initialize variables
    if (first_run == true) {
        prev_time = millis();

        setpoint = user_setpoint;
        input = user_input;
        prev_error = setpoint - input;

        output = init_output;
        prev_output = init_output;

        first_run = false;
    }

    // Subsequent runs
    else {
        current_time = millis();
        unsigned long time_interval = current_time - prev_time;
        
        if (time_interval >= sample_time) {
            setpoint = user_setpoint;
            input = user_input;
            double error = setpoint - input;

            output = init_output + kp * error;
            output += kd * (error - prev_error) / ((double)time_interval / 1000.0);

            // To prevent integral windup, only compute and add integral term if output is not at bounds
            if (output < max_output  ||  output > min_output) {
                cumulativeError += error * ((double)time_interval / 1000.0);
                output += ki * cumulativeError;
            }

            if (output > max_output) {
                output = max_output;
            }
            else if (output < min_output) {
                output = min_output;
            }

            // Don't change output if within deadband
            if (abs(output - prev_output) < deadband) {
                output = prev_output;
            }

            // For the next run
            prev_time = current_time;
            prev_error = error;
            prev_output = output;
        }
    }

    return output;
}

void PID::changeSettings(double user_kp, double user_ki, double user_kd, bool user_reverse, unsigned long user_sample_time, double user_min_output, double user_max_output, double user_deadband) {
    kp = user_kp;
    ki = user_ki;
    kd = user_kd;
    reverse = user_reverse;
    sample_time = user_sample_time;
    min_output = user_min_output;
    max_output = user_max_output;
    deadband = user_deadband;

    // Reverse coefficients if in reverse mode
    if (reverse == true) {
        kp *= -1.0;
        ki *= -1.0;
        kd *= -1.0;
    }
}

double PID::getSetpoint() {
    return setpoint;
}
double PID::getOutput() {
    return output;
}
double PID::getkp() {
    return kp;
}
double PID::getki() {
    return ki;
}
double PID::getkd() {
    return kd;
}
