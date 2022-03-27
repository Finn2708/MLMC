/******************************************************************************
 * Interdisciplinary Research Group on Optronics and Avionics                 *
 ******************************************************************************
 * Application:     ----
 * Component:       PID Controller
 *
 * Authors:         Hagen.Hasberg@Haw-Hamburg.de
 *
 * Created:         2018.11.19
 * Updated:         2019.02.25
 *
 * Version:         0.01
 ******************************************************************************
 * Description:     PID Controller Implementation with Anti-Windup            *
 *                  Anti-Windup is based on Clamping:                         *
 *                  https://www.youtube.com/watch?v=NVLXCwc8HzM               *
 *****************************************************************************/

/******************************************************************************
 *                                 Includes                                   *
 *****************************************************************************/

#include "PIDController.h"

/******************************************************************************
 *                         Construction & Destruction                         *
 *****************************************************************************/

PIDController::PIDController(float Kp, float Ki, float Kd, float outMax, float outMin)
/////////// Kx Gains ///////////
: gain_Kp    (Kp)
, gain_Ki    (Ki)
, gain_Kd    (Kd)
, gain_outMax(outMax)
, gain_outMin(outMin)
/////////// Integrator ///////////
, integrator_sum    (0)
, integrator_doClamp(false)
/////////// Derivative ///////////
, derivative_last(0)
{

}

PIDController::~PIDController()
{

}

/******************************************************************************
 *                                  Tune Gains                                *
 *****************************************************************************/

void PIDController::tuneGains(float Kp, float Ki, float Kd, float outMax, float outMin)
{
    // Set gains
    this->gain_Kp     = Kp;
    this->gain_Ki     = Ki;
    this->gain_Kd     = Kd;
    this->gain_outMax = outMax;
    this->gain_outMin = outMin;
}

/******************************************************************************
 *                                     Run                                    *
 *****************************************************************************/

float PIDController::runController(float processVariable, float setpoint, float dt)
{
    // Calculate error
    float error = setpoint - processVariable;
    
    // Proportional
    float part_p = gain_Kp * error;
    
    // Integral
    if(integrator_doClamp)
        integrator_sum += dt * 0; // Effectively clamp the integrator
    else
        integrator_sum += dt * error;
    float part_i = gain_Ki * integrator_sum;
    
    // Derivative
    float tmp_d = 0;
    if(dt != 0)
        tmp_d = (error - derivative_last) / dt;
    float part_d = gain_Kd * tmp_d;
    derivative_last = error;
    
    // Sum parts
    float output = part_p + part_i + part_d;
	last_i  = part_i;
	last_p = part_p;
	last_error = error;
    
    // Clamp
    float o1 = output;
    if (output > gain_outMax) output = gain_outMax;
    if (output < gain_outMin) output = gain_outMin;
    float o2 = output;
    
    // Check whether output was clamped
    bool awu_outputWasClamped = o1 != o2;
    
    // Check Error and Output signs
    bool awu_equalSigns = (error >= 0 && o1 >= 0) || (error < 0 && o1 < 0);
    
    // Eventually activate anti-windup clamping
    integrator_doClamp = awu_outputWasClamped && awu_equalSigns;
    
    return output;
}

// Without Integrator Anti-Windup
float PIDController::runController_woAntiWindup(float processVariable, float setpoint, float dt)
{
    // Calculate error
    float error = setpoint - processVariable;
    
    // Proportional
    float part_p = gain_Kp * error;
    
    // Integral
    integrator_sum += dt * error;
    float part_i = gain_Ki * integrator_sum;
    
    // Derivative
    float tmp_d = 0;
    if(dt != 0)
        tmp_d = (error - derivative_last) / dt;
    float part_d = gain_Kd * tmp_d;
    derivative_last = error;
    
    // Sum parts
    float output = part_p + part_i + part_d;
    
    // Clamp
    if (output > gain_outMax) output = gain_outMax;
    if (output < gain_outMin) output = gain_outMin;
    
    return output;
}

void PIDController::resetController()
{
    integrator_doClamp = false;
    derivative_last    = 0;
    integrator_sum     = 0;
}
