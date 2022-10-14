/*
 * Software for the auxiliary controller for the L3X-Z Hexapod
 */

#ifndef AUX_CTRL_NEO_PIXEL_CONTROL_H_
#define AUX_CTRL_NEO_PIXEL_CONTROL_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

/**************************************************************************************
 * CLASS DEFINITION
 **************************************************************************************/

class NeoPixelControl
{
public:

  NeoPixelControl();

  void light_off();
  void light_green();
  void light_red();
  void light_blue();
  void light_white();
  void light_amber();
};

#endif /* AUX_CTRL_NEO_PIXEL_CONTROL_H_ */
