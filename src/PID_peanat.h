#ifndef PID_PEANUT_H
#define PID_PEANUT_H

class PID {

    public:

    // Constructor
    PID(double user_kp, double user_ki, double user_kd, double user_init_output = 0.0, bool user_reverse = false, unsigned long user_sample_time = 100, double user_min_output = 0.0, double user_max_output = 255.0, double user_deadband = 0.0);

    // Calculate PID output
    double compute(double user_setpoint, double user_input);

    // Change PID settings (Note: cannot reset init_output)
    void changeSettings(double user_kp, double user_ki, double user_kd, bool user_reverse = false, unsigned long user_sample_time = 100, double user_min_output = 0.0, double user_max_output = 255.0, double user_deadband = 0.0);

    // Get private variables
    double getSetpoint();
    double getOutput();
    double getkp();
    double getki();
    double getkd();


    private:

    double setpoint;                        // Target value
    
    double input;                           // What the value really is

    double output;                          // Output to whatever takes it e.g. servo
    double init_output;                     // Output value to start from
    double prev_output;

    double kp;                              // PID variables
    double ki;
    double kd;

    double prev_error;
    double cumulativeError;                 // For calculating integral output

    unsigned long current_time;             // Time (self explanatory)
    unsigned long prev_time;
    unsigned long sample_time;              // Time interval to read in inputs

    double min_output;
    double max_output;

    double deadband;                        // The range in which we don't change the output (to prevent mechanical wear)
    
    bool reverse;
    bool first_run;
};

#endif