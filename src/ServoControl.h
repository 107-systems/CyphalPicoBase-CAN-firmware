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

  typedef uavcan::primitive::scalar::Integer16_1_0 TOpenCyphalServo_0;
  typedef uavcan::primitive::scalar::Integer16_1_0 TOpenCyphalServo_1;

  ServoControl(int const servo_0_pin, int const servo_1_pin, Node & node_hdl);

  void begin();

private:
  Servo _servo_0, _servo_1;
  Subscription _servo_0_sub, _servo_1_sub;
};

#endif /* AUX_CTRL_SERVO_CONTROL_H_ */
