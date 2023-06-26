/*
 * Software for the auxiliary controller for the L3X-Z Hexapod
 */

#ifndef AUX_CTRL_DIGITAL_OUTPUT_CONTROL_H_
#define AUX_CTRL_DIGITAL_OUTPUT_CONTROL_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <107-Arduino-Cyphal.h>

//#include "PortId.h"

/**************************************************************************************
 * CLASS DEFINITION
 **************************************************************************************/

class DigitalOutControl
{
public:

  typedef uavcan::primitive::scalar::Bit_1_0 TOpenCyphalDigitalOut_0;
  typedef uavcan::primitive::scalar::Bit_1_0 TOpenCyphalDigitalOut_1;

  DigitalOutControl(int const out_0_pin, int const out_1_pin, Node & node_hdl);

  void begin();

private:
  int _out_0_pin, _out_1_pin;
  Subscription _out_0_sub, _out_1_sub;
};

#endif /* AUX_CTRL_DIGITAL_OUTPUT_CONTROL_H_ */
