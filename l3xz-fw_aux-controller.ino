/*
 * Software for the auxiliary controller for the L3X-Z Hexapod
 *
 * Hardware:
 *   - Raspberry Pi Pico
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
//#include <Adafruit_SleepyDog.h>
//#include <Adafruit_NeoPixel_ZeroDMA.h>

/**************************************************************************************
 * DEFINES
 **************************************************************************************/

#define EMERGENCY_STOP 10
#define ANALOG_PIN 26

// Which pin on the Arduino is connected to the NeoPixels?
//#define NEOPIXELPIN        12 // Adafruit Feather M0
//#define NEOPIXELPIN        A2 // Arduino Nano 33 IoT
#define NEOPIXELPIN        12 // Raspberry Pi Pico

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

static int const MKRCAN_MCP2515_CS_PIN  = 17;
static int const MKRCAN_MCP2515_INT_PIN = 20;

static CanardPortID const ID_INPUT_VOLTAGE       = 1001U;
static CanardPortID const ID_LED1                = 1005U;
static CanardPortID const ID_EMERGENCY_STOP      = 2001U;
static CanardPortID const ID_LIGHT_MODE          = 2002U;

static SPISettings  const MCP2515x_SPI_SETTING{1000000, MSBFIRST, SPI_MODE0};

static int8_t const LIGHT_MODE_RED   = 1;
static int8_t const LIGHT_MODE_GREEN = 2;
static int8_t const LIGHT_MODE_AMBER = 3;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void onLed1_Received (CanardTransfer const &, ArduinoUAVCAN &);
void onLightMode_Received(CanardTransfer const &, ArduinoUAVCAN &);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

static ArduinoUAVCAN * uc = nullptr;

ArduinoMCP2515 mcp2515([]()
                       {
                         noInterrupts();
                         SPI.beginTransaction(MCP2515x_SPI_SETTING);
                         digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW);
                       },
                       []()
                       {
                         digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);
                         SPI.endTransaction();
                         interrupts();
                       },
                       [](uint8_t const d) { return SPI.transfer(d); },
                       micros,
                       [](CanardFrame const & f) { uc->onCanFrameReceived(f); },
                       nullptr);

Heartbeat_1_0<> hb;
Bit_1_0<ID_EMERGENCY_STOP> uavcan_emergency_stop;
Real32_1_0<ID_INPUT_VOLTAGE> uavcan_input_voltage;
Integer8_1_0<ID_LIGHT_MODE> uavcan_light_mode;

//Adafruit_NeoPixel_ZeroDMA pixels(NUMPIXELS, NEOPIXELPIN, NEO_GRB);

void light_off()
{
//  pixels.clear();
//  pixels.show();
}
void light_green()
{
//  pixels.fill(pixels.Color(0, 55, 0));
//  pixels.show();
}
void light_red()
{
//  pixels.fill(pixels.Color(55, 0, 0));
//  pixels.show();
}
void light_amber()
{
//  pixels.fill(pixels.Color(55, 40, 0));
//  pixels.show();
}

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
//  Watchdog.enable(1000);

  Serial.begin(115200);

  /* Setup LED pins and initialize */
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(EMERGENCY_STOP, INPUT_PULLUP);

  /* create UAVCAN class */
  uc = new ArduinoUAVCAN(99, [](CanardFrame const & frame) -> bool { return mcp2515.transmit(frame); });

  /* Setup SPI access */
  SPI.begin();
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), []() { mcp2515.onExternalEventHandler(); }, FALLING);

  /* Initialize MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);
  mcp2515.setNormalMode();

  /* Configure initial heartbeat */
  uavcan_light_mode.data.value = LIGHT_MODE_GREEN;

  hb.data.uptime = 0;
  hb = Heartbeat_1_0<>::Health::NOMINAL;
  hb = Heartbeat_1_0<>::Mode::INITIALIZATION;
  hb.data.vendor_specific_status_code = 0;

  /* Subscribe to the reception of Bit message. */
  uc->subscribe<Bit_1_0<ID_LED1>>(onLed1_Received);
  uc->subscribe<Integer8_1_0<ID_LIGHT_MODE>>(onLightMode_Received);

  /* Init Neopixel */
//  if(! pixels.begin()) {
//    Serial.println("ERROR: Init NeoPixel...");
//    while(1);
//  }

  light_off();
}

void loop()
{
  /* LED functions */
  static unsigned long prev_led = 0;
  unsigned long const now = millis();

  if((now - prev_led) > 250)
  {
    static bool is_light_on = false;
    is_light_on = !is_light_on;

    if (is_light_on)
    {
      if (uavcan_light_mode.data.value == LIGHT_MODE_GREEN)
        light_green();
      else if (uavcan_light_mode.data.value == LIGHT_MODE_AMBER)
        light_amber();
      else
        light_red();
    }
    else
      light_off();

    prev_led = now;
  }

  /* Update the heartbeat object */
  hb.data.uptime = millis() / 1000;
  hb = Heartbeat_1_0<>::Mode::OPERATIONAL;

  /* Publish the heartbeat once/second */
  static unsigned long prev_hearbeat = 0;
  static unsigned long prev_battery_voltage = 0;

  if(now - prev_hearbeat > 1000) {
    uc->publish(hb);
    prev_hearbeat = now;
  }
  if((now - prev_battery_voltage) > (3*1000))
  {
    float const analog = analogRead(ANALOG_PIN)*3.3*11.0/1023.0;
    Serial.print("Analog Pin: ");
    Serial.println(analog);
    Real32_1_0<ID_INPUT_VOLTAGE> uavcan_input_voltage;
    uavcan_input_voltage.data.value = analog;
    uc->publish(uavcan_input_voltage);
    prev_battery_voltage = now;
  }

  /* Transmit all enqeued CAN frames */
  while(uc->transmitCanFrame()) { }

  /* Feed the watchdog to keep it from biting. */
//  Watchdog.reset();
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onLed1_Received(CanardTransfer const & transfer, ArduinoUAVCAN & /* uavcan */)
{
  Bit_1_0<ID_LED1> const uavcan_led1 = Bit_1_0<ID_LED1>::deserialize(transfer);

  if(uavcan_led1.data.value)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Received Bit1: true");
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Received Bit1: false");
  }
}

void onLightMode_Received(CanardTransfer const & transfer, ArduinoUAVCAN & /* uavcan */)
{
  uavcan_light_mode = Integer8_1_0<ID_LIGHT_MODE>::deserialize(transfer);
}
