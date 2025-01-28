//==============================================================================

/* 
 * File Description:
 * This module implements the buck cv-cc controller using two PI controllers with anti-windup
 * 
 */

// author: Munadir Aziz Ahmed
// date: 1/24/2025

//==============================================================================

// Module used
#include <stdint.h>

#include "PI_Controller.h"
#include "buck_controller.h"


// Define data types and structures

// Define constants

// Declare static global variables

static piControllerParam_t vLoopCtrllr =  { .b_coeff = {4.1f,-3.9f},
                                            .x={0.0f,0.0f},
                                            .y={0.0f,0.0f},
                                            .y_max = 4.5f,
                                            .y_min = 0.0f
                                          };
                                          
static piControllerParam_t iLoopCtrllr =  { .b_coeff = {1.008f, -0.9921f},
                                            .x={0.0f,0.0f},
                                            .y={0.0f,0.0f},
                                            .y_max = 1.0f,
                                            .y_min = 0.0f
                                          };

// Define local function prototypes

// Declare function

/**
 * Buck converter voltage control loop
 * @param Vo_measure - measured value
 * @param V_ref - reference value
 *
 * @return - reference value for current loop
*/
float buck_voltage_loop (float Vo_measure, float V_ref)
{
     
   float i_ref = piController(&vLoopCtrllr, Vo_measure, V_ref);
	
	return i_ref;    
     
}

/**
 * Buck converter current control loop
 * @param I_L_measure - measured value
 * @param I_ref - reference value
 *
 * @return - duty cycle
*/
float buck_current_loop (float I_L_measure, float I_ref)
{
     
   float d = piController(&iLoopCtrllr, I_L_measure, I_ref);
	
	return d;    
     
}