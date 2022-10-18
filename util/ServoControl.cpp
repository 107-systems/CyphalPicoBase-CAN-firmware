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

ServoControl::ServoControl(int const servo_0_pin, int const servo_1_pin)
{
  _servo_0.attach(servo_0_pin, 800, 2200);
  _servo_1.attach(servo_1_pin, 800, 2200);
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void ServoControl::begin()
{
  _servo_0.writeMicroseconds(1500);
  _servo_1.writeMicroseconds(1500);
}

void ServoControl::subscribe(Node & node_hdl)
{
  node_hdl.subscribe<TOpenCyphalServo_0>(
      [this](CanardRxTransfer const & transfer, Node & /* node_hdl */)
      {
        TOpenCyphalServo_0 const uavcan_servo_0 = TOpenCyphalServo_0::deserialize(transfer);
        _servo_0.writeMicroseconds(uavcan_servo_0.data.value);
      });

  node_hdl.subscribe<TOpenCyphalServo_1>(
      [this](CanardRxTransfer const & transfer, Node & /* node_hdl */)
      {
        TOpenCyphalServo_1 const uavcan_servo_1 = TOpenCyphalServo_1::deserialize(transfer);
        _servo_1.writeMicroseconds(uavcan_servo_1.data.value);
      });
}
