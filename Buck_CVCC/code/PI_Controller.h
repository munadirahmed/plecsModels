//==============================================================================
/**
 * Filename: PI_Controller.h
 * Description: PI Controller header file that defines the interface and object/structure used by the controller
 * Author: Munadir Ahmed
*/
//==============================================================================

#ifndef PI_CONTROLLER_H_
#define PI_CONTROLLER_H_

typedef struct
{ 
	//float const a_coeff[2];   // holds the denominator coefficients (always set to [1,-1]] by the PI controller implementation
	float const b_coeff[2];   // holds the numerator coefficients
   float x[2];   // holds the present step and previous step input values: x[n, n-1]
   float y[2];   // holds the present step and previous step output values: y[n, n-1]
   float y_max;
   float y_min;
} piControllerParam_t;

static float piController(piControllerParam_t *ptrPiCntrlParam, float measurement, float reference);


#endif /* PI_CONTROLLER_H_ */