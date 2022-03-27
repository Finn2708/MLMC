/******************************************************************************
 * Interdisciplinary Research Group on Optronics and Avionics                 *
 ******************************************************************************
 * Application:     ----
 * Component:       Rate Limiter
 *
 * Authors:         Hagen.Hasberg@Haw-Hamburg.de
 *
 * Created:         2018.02.19
 * Updated:         2019.02.25
 *
 * Version:         0.01
 ******************************************************************************
 * Description:     ----                                                      *
 *****************************************************************************/

/******************************************************************************
 *                                 Includes                                   *
 *****************************************************************************/

#include "RateLimiter.h"

/******************************************************************************
 *                         Construction & Destruction                         *
 *****************************************************************************/

RateLimiter::RateLimiter(float rateUp, float rateDown)
/////////// Rates ///////////
: rate_up  (rateUp)
, rate_down(rateDown)
/////////// State ///////////
, state_sum(0)
{

}

RateLimiter::~RateLimiter()
{

}

/******************************************************************************
 *                                  Tune Rate                                 *
 *****************************************************************************/

void RateLimiter::tuneRates(float rateUp, float rateDown)
{
    // Set rates
    this->rate_up   = rateUp;
    this->rate_down = rateDown;
}

/******************************************************************************
 *                                     Run                                    *
 *****************************************************************************/

float RateLimiter::limitRate (float processVariable, float dt)
{
    // Calculate diff
    float diff = processVariable - state_sum;
    
    // Calculate vals
    float val_up   = rate_up   * dt;
    float val_down = rate_down * dt;
    
    // Limit Rate
    float output = 0;
    if     (diff > val_up)
        output = state_sum + val_up;
    else if(diff < val_down)
        output = state_sum + val_down;
    else
        output = processVariable;
        
    state_sum = output;
    
    return output;
}

void RateLimiter::resetState()
{
    state_sum = 0;
}
