/*
 * Software for the auxiliary controller for the L3X-Z Hexapod
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "ServoControl.h"


/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

ServoControl::ServoControl(int const servo_0_pin, int const servo_1_pin, Node & node_hdl)
{
  _servo_0.attach(servo_0_pin, 800, 2200);
  _servo_1.attach(servo_1_pin, 800, 2200);

/*  _servo_0_sub = node_hdl.create_subscription<TOpenCyphalServo_0>(
    ID_SERVO0,
    [this](TOpenCyphalServo_0 const & msg) -> void
    {
      _servo_0.writeMicroseconds(msg.value);
    });*/

/*  _servo_1_sub = node_hdl.create_subscription<TOpenCyphalServo_1>(
    ID_SERVO1,
    [this](TOpenCyphalServo_1 const & msg) -> void
    {
      _servo_1.writeMicroseconds(msg.value);
    });*/
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void ServoControl::begin()
{
  _servo_0.writeMicroseconds(1500);
  _servo_1.writeMicroseconds(1500);
}
