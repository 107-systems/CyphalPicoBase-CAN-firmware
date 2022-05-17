/*
 * Software for the auxiliary controller for the L3X-Z Hexapod
 *
 * Hardware:
 *   - Adafruit Feather M0
 *   - MCP2515
 *
 * Used Subject-IDs
 * 1001 - pub - Real32   - input voltage
 * 1005 - sub - Bit      - LED1
 * 2001 - pub - Bit      - emergency stop
 * 2002 - sub - Integer8 - light mode
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/
#include <SPI.h>
#include <Wire.h>


#include <ArduinoUAVCAN.h>
#include <ArduinoMCP2515.h>
#include <I2C_eeprom.h>
#include <Adafruit_NeoPixel.h>

/**************************************************************************************
 * DEFINES
 **************************************************************************************/

#define LED1_PIN 13
#define EMERGENCY_STOP 6
#define ANALOG_PIN A1

// Which pin on the Arduino is connected to the NeoPixels?
#define NEOPIXELPIN        5 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 4 // Popular NeoPixel ring size

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;
using namespace uavcan::primitive::scalar;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int const MKRCAN_MCP2515_CS_PIN  = A2;
static int const MKRCAN_MCP2515_INT_PIN = A3;
static CanardPortID const ID_INPUT_VOLTAGE  = 1001U;
static CanardPortID const ID_LED1           = 1005U;
static CanardPortID const ID_EMERGENCY_STOP = 2001U;
static CanardPortID const ID_LIGHT_MODE     = 2002U;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void    spi_select        ();
void    spi_deselect       ();
uint8_t spi_transfer       (uint8_t const);
void    onExternalEvent    ();
bool    transmitCanFrame   (CanardFrame const &);
void    onReceiveBufferFull(CanardFrame const &);
void    onLed1_Received (CanardTransfer const &, ArduinoUAVCAN &);
void    onLightMode_Received (CanardTransfer const &, ArduinoUAVCAN &);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ArduinoMCP2515 mcp2515(spi_select,
                       spi_deselect,
                       spi_transfer,
                       micros,
                       onReceiveBufferFull,
                       nullptr);

static ArduinoUAVCAN * uc = nullptr;

Heartbeat_1_0<> hb;
Bit_1_0<ID_EMERGENCY_STOP> uavcan_emergency_stop;
Real32_1_0<ID_INPUT_VOLTAGE> uavcan_input_voltage;

I2C_eeprom ee(0x50, I2C_DEVICESIZE_24LC64);
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXELPIN, NEO_GRB + NEO_KHZ800);

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(9600);
  delay(3000);
//  while(!Serial) { } /* only for debug */

  /* Setup LED pins and initialize */
  pinMode(LED1_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  pinMode(EMERGENCY_STOP, INPUT_PULLUP);

  /* Setup I2C Eeprom */
  ee.begin();
  if (! ee.isConnected())
  {
    Serial.println("ERROR: Can't find eeprom\nstopped...");
    while (1);
  }
  uint8_t const eeNodeID=ee.readByte(0);
  Serial.print("Node-ID from eeprom: ");
//  ee.writeByte(0,30);
  Serial.println(eeNodeID);

  /* create UAVCAN class */
  uc = new ArduinoUAVCAN(eeNodeID, transmitCanFrame);

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)


  /* Setup SPI access */
  SPI.begin();
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), onExternalEvent, FALLING);

  /* Initialize MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);
  mcp2515.setNormalMode();

  /* Configure initial values */
  uavcan_emergency_stop.data.value = false;
  uavcan_input_voltage.data.value = 0.0;
  /* Configure initial heartbeat */
  hb.data.uptime = 0;
  hb = Heartbeat_1_0<>::Health::NOMINAL;
  hb = Heartbeat_1_0<>::Mode::INITIALIZATION;
  hb.data.vendor_specific_status_code = 0;

  /* Subscribe to the reception of Bit message. */
  uc->subscribe<Bit_1_0<ID_LED1>>(onLed1_Received);
  uc->subscribe<Integer8_1_0<ID_LIGHT_MODE>>(onLightMode_Received);
  Serial.println("init finished");

  /* Init Neopixel */
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.show();   // Send the updated pixel colors to the hardware.
  delay(300);
  pixels.setPixelColor(0, pixels.Color(55, 0, 0));
  pixels.show();   // Send the updated pixel colors to the hardware.
  delay(300);
  pixels.setPixelColor(1, pixels.Color(0, 55, 0));
  pixels.show();   // Send the updated pixel colors to the hardware.
  delay(300);
  pixels.setPixelColor(2, pixels.Color(0, 0, 55));
  pixels.show();   // Send the updated pixel colors to the hardware.
  delay(300);
//  pixels.setPixelColor(3, pixels.Color(55, 55, 55));
  pixels.setPixelColor(3, pixels.Color(55, 40, 0));
  pixels.show();   // Send the updated pixel colors to the hardware.

}

void loop()
{
  /* check switch */
  static bool bumper_old=0;
  bool bumper_in;
  bumper_in=digitalRead(EMERGENCY_STOP);
  if(bumper_old!=bumper_in)
  {
    uavcan_emergency_stop.data.value = bumper_in;
    uc->publish(uavcan_emergency_stop);
    Serial.print("send bit: ");
    Serial.println(bumper_in);
  }
  bumper_old=bumper_in;

  /* Update the heartbeat object */
  hb.data.uptime = millis() / 1000;
  hb = Heartbeat_1_0<>::Mode::OPERATIONAL;

  /* Publish the heartbeat once/second */
  static unsigned long prev = 0;
  unsigned long const now = millis();
  if(now - prev > 1000) {
  /* read analog value */
    float analog=analogRead(ANALOG_PIN)*3.3*11.0/1023.0;
    Serial.print("Analog Pin: ");
    Serial.println(analog);
    uavcan_input_voltage.data.value = analog;
    uc->publish(uavcan_input_voltage);

  /* publish heartbeat */
    uc->publish(hb);
    prev = now;
  }

  /* Transmit all enqeued CAN frames */
  while(uc->transmitCanFrame()) { }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void spi_select()
{
  digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW);
}

void spi_deselect()
{
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);
}

uint8_t spi_transfer(uint8_t const data)
{
  return SPI.transfer(data);
}

void onExternalEvent()
{
  mcp2515.onExternalEventHandler();
}

bool transmitCanFrame(CanardFrame const & frame)
{
  return mcp2515.transmit(frame);
}

void onReceiveBufferFull(CanardFrame const & frame)
{
  uc->onCanFrameReceived(frame);
}

void onLed1_Received(CanardTransfer const & transfer, ArduinoUAVCAN & /* uavcan */)
{
  Bit_1_0<ID_LED1> const uavcan_led1 = Bit_1_0<ID_LED1>::deserialize(transfer);

  if(uavcan_led1.data.value)
  {
    digitalWrite(LED1_PIN, HIGH);
    Serial.println("Received Bit1: true");
  }
  else
  {
    digitalWrite(LED1_PIN, LOW);
    Serial.println("Received Bit1: false");
  }
}

void onLightMode_Received(CanardTransfer const & transfer, ArduinoUAVCAN & /* uavcan */)
{
  Integer8_1_0<ID_LIGHT_MODE> const uavcan_lightmode = Integer8_1_0<ID_LIGHT_MODE>::deserialize(transfer);

  if(uavcan_lightmode.data.value==1)
  {
    pixels.fill(pixels.Color(55, 0, 0));    // red
    pixels.show();   // Send the updated pixel colors to the hardware.
  }
  else if(uavcan_lightmode.data.value==2)
  {
    pixels.fill(pixels.Color(0, 55, 0));    // green
    pixels.show();   // Send the updated pixel colors to the hardware.
  }
  else if(uavcan_lightmode.data.value==3)
  {
    pixels.fill(pixels.Color(0, 0, 55));    // blue
    pixels.show();   // Send the updated pixel colors to the hardware.
  }
  else if(uavcan_lightmode.data.value==4)
  {
    pixels.fill(pixels.Color(55, 55, 55));    // white
    pixels.show();   // Send the updated pixel colors to the hardware.
  }
  else if(uavcan_lightmode.data.value==5)
  {
    pixels.fill(pixels.Color(55, 40, 0));    // amber
    pixels.show();   // Send the updated pixel colors to the hardware.
  }
  else
  {
    pixels.clear();    // all off
    pixels.show();   // Send the updated pixel colors to the hardware.
  }
}
