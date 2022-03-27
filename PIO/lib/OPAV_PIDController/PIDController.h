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

#ifndef __OPAV_PID_CONTROLLER_H__
#define __OPAV_PID_CONTROLLER_H__

/******************************************************************************
 *                                 Includes                                   *
 *****************************************************************************/

#include <stdint.h>

/******************************************************************************
 *                              Class Definition                              *
 *****************************************************************************/

class PIDController
{
    /******************************************************************************
     *                         Construction & Destruction                         *
     *****************************************************************************/
public:
             PIDController(float Kp=0, float Ki=0, float Kd=0, float outMax=0, float outMin=0);

    virtual ~PIDController();

    /******************************************************************************
     *                                  Tune Gains                                *
     *****************************************************************************/
    
    virtual void tuneGains(float Kp, float Ki, float Kd, float outMax, float outMin);
    
    /******************************************************************************
     *                                     Run                                    *
     *****************************************************************************/
    
    virtual float runController             (float processVariable, float setpoint, float dt); // dt in seconds
    virtual float runController_woAntiWindup(float processVariable, float setpoint, float dt);
    
    virtual void  resetController           ();
    
    /******************************************************************************
     *                                   Private                                  *
     *****************************************************************************/
public:
    /////////// Kx Gains ///////////
    float gain_Kp;
    float gain_Ki;
    float gain_Kd;
    float gain_outMax;
    float gain_outMin;
	float last_i;
	float last_p;
	float last_error;
    
    /////////// Integrator ///////////
    float integrator_sum;
    bool  integrator_doClamp;
    
    /////////// Derivative ///////////
    float derivative_last;
    
};

#endif
