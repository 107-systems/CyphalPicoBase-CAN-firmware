/*
 * Software for the auxiliary controller for the L3X-Z Hexapod
 */

#ifndef AUX_CTRL_SERVO_CONTROL_H_
#define AUX_CTRL_SERVO_CONTROL_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Servo.h>

#include <107-Arduino-Cyphal.h>

#include "PortId.h"

/**************************************************************************************
 * CLASS DEFINITION
 **************************************************************************************/

class ServoControl
{
public:

  typedef uavcan::primitive::scalar::Integer16_1_0<ID_SERVO0> TOpenCyphalServo_0;
  typedef uavcan::primitive::scalar::Integer16_1_0<ID_SERVO1> TOpenCyphalServo_1;

  ServoControl();

  void begin(int const servo0_pin, int const servo1_pin);
  void subscribe(Node & node_hdl);

private:
  Servo _servo0, _servo1;
};

#endif /* AUX_CTRL_SERVO_CONTROL_H_ */
