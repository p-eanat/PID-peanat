/* 
 * Peanat's PID library made for Jeremy the robot, but should work well for all other robot needs too!
 * Extra features: custom initial output, reverse mode, integral windup prevention, deadband limits
 * Date created: Aug. 16, 2022
 */

#include "Arduino.h"
#include "PID_peanat.h"

const double SECS_IN_MS = 0.001;


PID::PID(double user_kp, double user_ki, double user_kd) {
    /* 
     * Initialize PID object.
     *
     * @param user_kp: proportional coefficient, should be >= 0
     * @param user_ki: integral coefficient, should be >= 0
     * @param user_kd: derivative coefficient, should be >= 0
     * 
     * @returns PID object
     */

    kp = user_kp;
    ki = user_ki;
    kd = user_kd;
    init_output = 0.0;
    reverse = false;
    sample_time = 100;
    min_output = 0.0;
    max_output = 255.0;
    deadband = 0.0;

    // Reverse coefficients if in reverse mode
    if (reverse == true) {
        kp *= -1.0;
        ki *= -1.0;
        kd *= -1.0;
    }

    // Initiate other fields
    first_run = true;
    cumulative_error = 0.0;
}

void PID::updateCoeffs(double user_kp, double user_ki, double user_kd) {
    /*
     * Updates PID coefficients.
     * 
     * @param user_kp: proportional coefficient, should be >= 0
     * @param user_ki: integral coefficient, should be >= 0
     * @param user_kd: derivative coefficient, should be >= 0
     * 
     * @returns none
     */

    kp = user_kp;
    ki = user_ki;
    kd = user_kd;
}

void PID::setInitOutput(double user_init_output) {
    /*
     * Set initial output
     *
     * @param user_init_output: desired initial output, for when we want the output to start at a specific value
     * 
     * @returns none
     */
}

void PID::setReverse(bool user_reverse) {
    /*
     * Toggle reverse mode on or off.
     *
     * @param user_reverse: true to turn reverse mode on, false to turn off
     * 
     * @returns none
     */

   if (reverse != user_reverse) {
        reverse = user_reverse;
   }
}

void PID::setSampleTime(unsigned long user_sample_time) {
    /*
     * Set sample time (recommended < 100).
     * 
     * @param user_sample_time: sample time in ms
     * 
     * @returns none
     */

    sample_time = user_sample_time;
}

void PID::setOutputBounds(double user_min_output, double user_max_output) {
    /*
     * Set output bounds; useful if something like a servo has input limits.
     *
     * @param user_min_output: minimum output value (inclusive)
     * @param user_max_output: maximum output value (inclusive)
     * 
     * @returns none
     */

    min_output = user_min_output;
    max_output = user_max_output;
}

void PID::setDeadband(double user_deadband) {
    /*
     * Set deadband range (difference between previous and current calculated output, in which the 
     * previous output is returned) if we need to prevent mechanical wear.
     * 
     * @param user_deadband: deadband range, in units of output
     * 
     * @returns none
     */

    deadband = user_deadband;
}

double PID::compute(double user_setpoint, double user_input) {
    /*
     * Computes PID output based on standard PID algorithm; implements deadband functionality and
     * measures to prevent integral windup. The first run will output the initial output value.
     * This loop can run for a maximum of 49 days before unsigned long overflows.
     * 
     * @param user_setpoint: desired value of something (e.g. servo position)
     * @param user_input: the acutal value
     * 
     * @returns output to be fed back into the controller
     */
    
    // If first run, initialize variables
    if (first_run == true) {
        prev_time = millis();
        prev_error = user_setpoint - user_input;
        prev_output = init_output;

        output = init_output;
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
            output += kd * (error - prev_error) / ((double)time_interval * SECS_IN_MS);

            // To prevent integral windup, only compute and add integral term if output is not at bounds
            if (output < max_output  &&  output > min_output) {
                cumulative_error += error * ((double)time_interval * SECS_IN_MS);
                output += ki * cumulative_error;
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
