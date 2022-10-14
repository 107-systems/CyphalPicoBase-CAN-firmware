/*
 * Software for the auxiliary controller for the L3X-Z Hexapod
 */

#ifndef AUX_CTRL_PORT_ID_H_
#define AUX_CTRL_PORT_ID_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <107-Arduino-Cyphal.h>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static CanardPortID const ID_INPUT_VOLTAGE        = 1001U;
static CanardPortID const ID_LED1                 = 1005U;
static CanardPortID const ID_INTERNAL_TEMPERATURE = 1010U;
static CanardPortID const ID_INPUT0               = 2000U;
static CanardPortID const ID_INPUT1               = 2001U;
static CanardPortID const ID_INPUT2               = 2002U;
static CanardPortID const ID_INPUT3               = 2003U;
static CanardPortID const ID_OUTPUT0              = 2004U;
static CanardPortID const ID_OUTPUT1              = 2005U;
static CanardPortID const ID_SERVO0               = 2006U;
static CanardPortID const ID_SERVO1               = 2007U;
static CanardPortID const ID_ANALOG_INPUT0        = 2008U;
static CanardPortID const ID_ANALOG_INPUT1        = 2009U;
static CanardPortID const ID_LIGHT_MODE           = 2010U;

#endif /* AUX_CTRL_PORT_ID_H_ */
