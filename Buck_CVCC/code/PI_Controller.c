//==============================================================================

/* 
 * File Description:
 * This module implements the PI controller with anti-windup as a first-order transfer function:
 *
 * Discrete Transfer Function:
 * G(z) = (b0 + b1 * z^-1) / (1 - z^-1)
 *
 * Coefficients:
 * b0 = Kp + (Ki * Ts / 2)
 * b1 = -Kp + (Ki * Ts / 2)
 *
 * where:
 * Kp = Proportional gain
 * Ki = Integral gain
 * Ts = Sampling time
 *
 * This transfer function is derived using the Tustin transformation.
 *
 * Final equation translates to:
 * y[n] = y[n-1] + b0 * x[n] + b1 * x[n-1]
 *
 * where:
 * y[n]   = Current output of the controller
 * y[n-1] = Previous output of the controller (from the last computation step)
 * x[n]   = Current input to the controller (error signal at the current step)
 * x[n-1] = Previous input to the controller (error signal from the last computation step)
 */

// author: Munadir Aziz Ahmed
// date: 1/24/2025

//==============================================================================

// Module used
#include <stdint.h>

#include "PI_Controller.h"


// Define data types and structures

// Define constants

// Declare static global variables

// Define local function prototypes

// Declare function

/**
 * PI controller implementation
 * @param ptrPiCntrlParam - pointer to piControlerParam structure that holds the controller coefficients
 * @param measurement - measured value
 * @param reference - reference value
 *
 * @return - output of a PI controller
*/
static float piController(piControllerParam_t *ptrPiCntrlParam, float measurement, float reference)
{
     
   // Calculate error for current controller execution
   ptrPiCntrlParam->x[0] = reference - measurement;
	   
   // Calculate output for the current controller execution
	ptrPiCntrlParam->y[0] = ptrPiCntrlParam->y[1] + ( ( ptrPiCntrlParam->b_coeff[0] ) * ptrPiCntrlParam->x[0] ) + ( ( ptrPiCntrlParam->b_coeff[1] ) * ptrPiCntrlParam->x[1] );
   
   
   //Bound output if it is above/below max/min
   if(ptrPiCntrlParam->y[0] > ptrPiCntrlParam->y_max)
   {
      ptrPiCntrlParam->y[0] = ptrPiCntrlParam->y_max;
   }

   if(ptrPiCntrlParam->y[0] < ptrPiCntrlParam->y_min)
   {
      ptrPiCntrlParam->y[0] = ptrPiCntrlParam->y_min;
   }	
   	
	// Update error and output terms for next controller execution
	ptrPiCntrlParam->x[1] = ptrPiCntrlParam->x[0];
	ptrPiCntrlParam->y[1] = ptrPiCntrlParam->y[0];
	
	return ptrPiCntrlParam->y[0];    
     
}