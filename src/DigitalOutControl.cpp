/*
 * Software for the auxiliary controller for the L3X-Z Hexapod
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "DigitalOutControl.h"

#include <Arduino.h>

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

DigitalOutControl::DigitalOutControl(int const out_0_pin, int const out_1_pin)
: _out_0_pin{out_0_pin}
, _out_1_pin{out_1_pin}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void DigitalOutControl::begin()
{
  pinMode(_out_0_pin, OUTPUT);
  pinMode(_out_1_pin, OUTPUT);

  digitalWrite(_out_0_pin, LOW);
  digitalWrite(_out_1_pin, LOW);
}

void DigitalOutControl::subscribe(Node & node_hdl)
{
  node_hdl.subscribe<TOpenCyphalDigitalOut_0>(
      [this](CanardRxTransfer const & transfer, Node & /* node_hdl */)
      {
          TOpenCyphalDigitalOut_0 const uavcan_output_0 = TOpenCyphalDigitalOut_0::deserialize(transfer);

          if(uavcan_output_0.data.value)
            digitalWrite(_out_0_pin, HIGH);
          else
            digitalWrite(_out_0_pin, LOW);
      });

  node_hdl.subscribe<TOpenCyphalDigitalOut_1>(
      [this](CanardRxTransfer const & transfer, Node & /* node_hdl */)
      {
        TOpenCyphalDigitalOut_1 const uavcan_output_1 = TOpenCyphalDigitalOut_1::deserialize(transfer);

        if(uavcan_output_1.data.value)
          digitalWrite(_out_1_pin, HIGH);
        else
          digitalWrite(_out_1_pin, LOW);
      });
}
