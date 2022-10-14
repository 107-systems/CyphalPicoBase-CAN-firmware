/*
 * Software for the auxiliary controller for the L3X-Z Hexapod
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "../include/NeoPixelControl.h"

#include <Adafruit_NeoPixel.h>

/**************************************************************************************
 * EXTERN
 **************************************************************************************/

extern Adafruit_NeoPixel pixels;

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

NeoPixelControl::NeoPixelControl()
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void NeoPixelControl::light_off()
{
  pixels.clear();
  pixels.show();
}

void NeoPixelControl::light_green()
{
  pixels.fill(pixels.Color(0, 55, 0));
  pixels.show();
}

void NeoPixelControl::light_red()
{
  pixels.fill(pixels.Color(55, 0, 0));
  pixels.show();
}

void NeoPixelControl::light_blue()
{
  pixels.fill(pixels.Color(0, 0, 55));
  pixels.show();
}

void NeoPixelControl::light_white()
{
  pixels.fill(pixels.Color(55, 55, 55));
  pixels.show();
}

void NeoPixelControl::light_amber()
{
  pixels.fill(pixels.Color(55, 40, 0));
  pixels.show();
}
