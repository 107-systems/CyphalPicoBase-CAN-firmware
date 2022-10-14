/*
 * Software for the auxiliary controller for the L3X-Z Hexapod
 */

#ifndef AUX_CTRL_DIGITAL_OUTPUT_CONTROL_H_
#define AUX_CTRL_DIGITAL_OUTPUT_CONTROL_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <107-Arduino-Cyphal.h>

#include "PortId.h"

/**************************************************************************************
 * CLASS DEFINITION
 **************************************************************************************/

class DigitalOutControl
{
public:

  typedef uavcan::primitive::scalar::Bit_1_0<ID_OUTPUT0> TOpenCyphalDigitalOut_0;
  typedef uavcan::primitive::scalar::Bit_1_0<ID_OUTPUT1> TOpenCyphalDigitalOut_1;

  DigitalOutControl(int const out_0_pin, int const out_1_pin)
  : _out_0_pin{out_0_pin}
  , _out_1_pin{out_1_pin}
  { }

  void begin()
  {
    pinMode(_out_0_pin, OUTPUT);
    pinMode(_out_1_pin, OUTPUT);

    digitalWrite(_out_0_pin, LOW);
    digitalWrite(_out_1_pin, LOW);
  }

  void subscribe(Node & node_hdl)
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

private:
  int _out_0_pin, _out_1_pin;
};

#endif /* AUX_CTRL_DIGITAL_OUTPUT_CONTROL_H_ */
