//==============================================================================
/**
 * Filename: PI_Controller.h
 * Description: PI Controller header file that defines the interface and object/structure used by the controller
 * Author: Munadir Ahmed
*/
//==============================================================================

#ifndef BUCK_CONTROLLER_H_
#define BUCK_CONTROLLER_H_

#include "PI_Controller.h"

float buck_voltage_loop(float Vo_measure, float V_ref);
float buck_current_loop(float I_L_measure, float I_ref);


#endif /* BUCK_CONTROLLER_H_ */