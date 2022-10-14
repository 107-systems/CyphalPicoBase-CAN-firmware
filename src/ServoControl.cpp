/*
 * Software for the auxiliary controller for the L3X-Z Hexapod
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "../include/ServoControl.h"


/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

ServoControl::ServoControl()
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void ServoControl::begin(int const servo0_pin, int const servo1_pin)
{
  _servo0.attach(servo0_pin, 800, 2200);
  _servo1.attach(servo1_pin, 800, 2200);
  _servo0.writeMicroseconds(1500);
  _servo1.writeMicroseconds(1500);
}

void ServoControl::subscribe(Node & node_hdl)
{
  node_hdl.subscribe<TOpenCyphalServo_0>(
      [this](CanardRxTransfer const & transfer, Node & /* node_hdl */)
      {
        TOpenCyphalServo_0 const uavcan_servo0 = TOpenCyphalServo_0::deserialize(transfer);
        _servo0.writeMicroseconds(uavcan_servo0.data.value);
      });

  node_hdl.subscribe<TOpenCyphalServo_1>(
      [this](CanardRxTransfer const & transfer, Node & /* node_hdl */)
      {
        TOpenCyphalServo_1 const uavcan_servo1 = TOpenCyphalServo_1::deserialize(transfer);
        _servo1.writeMicroseconds(uavcan_servo1.data.value);
      });
}
