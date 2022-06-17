/*
 * Software for the auxiliary controller for the L3X-Z Hexapod
 *
 * Hardware:
 *   - Raspberry Pi Pico
 *   - MCP2515
 *
 * Used Subject-IDs
 * 1001 - pub - Real32    - input voltage
 * 1005 - sub - Bit       - LED1
 * 1010 - pub - Real32    - internal temperature
 * 2000 - pub - Bit       - input0
 * 2001 - pub - Bit       - input1
 * 2002 - pub - Bit       - input2
 * 2003 - pub - Bit       - input3
 * 2004 - sub - Bit       - output0
 * 2005 - sub - Bit       - output1
 * 2006 - sub - Integer16 - servo0
 * 2007 - sub - Integer16 - servo1
 * 2008 - pub - Integer16 - analog_input0
 * 2009 - pub - Integer16 - analog_input1
 * 2010 - sub - Integer8  - light mode
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/
#include <SPI.h>
#include <Wire.h>
#include <Servo.h>

#include <107-Arduino-Cyphal.h>
#include <107-Arduino-MCP2515.h>
#include <I2C_eeprom.h>
//#include <Adafruit_SleepyDog.h>
//#include <Adafruit_NeoPixel_ZeroDMA.h>

/**************************************************************************************
 * DEFINES
 **************************************************************************************/

#define INPUT0_PIN         6
#define INPUT1_PIN         7
#define INPUT2_PIN         8
#define INPUT3_PIN         9
#define OUTPUT0_PIN       10
#define OUTPUT1_PIN       11
#define SERVO0_PIN        14
#define SERVO1_PIN        15
#define LED2_PIN          21
#define LED3_PIN          22
#define ANALOG_PIN        26
#define ANALOG_INPUT0_PIN 27
#define ANALOG_INPUT1_PIN 28

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

static CanardPortID const ID_INPUT_VOLTAGE        = 1001U;
static CanardPortID const ID_LED1                 = 1005U;
static CanardPortID const ID_INTERNAL_TEMPERATURE = 1010U;
static CanardPortID const ID_INPUT0               = 2000U;
static CanardPortID const ID_INPUT1               = 2001U;
static CanardPortID const ID_INPUT2               = 2002U;
static CanardPortID const ID_INPUT3               = 2003U;
static CanardPortID const ID_OUTPUT0              = 2004U;
static CanardPortID const ID_OUTPUT1              = 2005U;
static CanardPortID const ID_SERVO0               = 2006U;
static CanardPortID const ID_SERVO1               = 2007U;
static CanardPortID const ID_ANALOG_INPUT0        = 2008U;
static CanardPortID const ID_ANALOG_INPUT1        = 2009U;
static CanardPortID const ID_LIGHT_MODE           = 2010U;

static SPISettings  const MCP2515x_SPI_SETTING{1000000, MSBFIRST, SPI_MODE0};

static int8_t const LIGHT_MODE_RED   = 1;
static int8_t const LIGHT_MODE_GREEN = 2;
static int8_t const LIGHT_MODE_AMBER = 3;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void onLed1_Received (CanardRxTransfer const &, Node &);
void onOutput0_Received (CanardRxTransfer const &, Node &);
void onOutput1_Received (CanardRxTransfer const &, Node &);
void onServo0_Received (CanardRxTransfer const &, Node &);
void onServo1_Received (CanardRxTransfer const &, Node &);
void onLightMode_Received(CanardRxTransfer const &, Node &);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

static Node * uc = nullptr;

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
                       [](CanardFrame const & f) { uc->onCanFrameReceived(f, micros()); },
                       nullptr);

Heartbeat_1_0<> hb;
Bit_1_0<ID_INPUT0> uavcan_input0;
Bit_1_0<ID_INPUT1> uavcan_input1;
Bit_1_0<ID_INPUT2> uavcan_input2;
Bit_1_0<ID_INPUT3> uavcan_input3;
Integer16_1_0<ID_ANALOG_INPUT0> uavcan_analog_input0;
Integer16_1_0<ID_ANALOG_INPUT1> uavcan_analog_input1;
Real32_1_0<ID_INPUT_VOLTAGE> uavcan_input_voltage;
Real32_1_0<ID_INTERNAL_TEMPERATURE> uavcan_internal_temperature;
Integer8_1_0<ID_LIGHT_MODE> uavcan_light_mode;
Servo servo0;
Servo servo1;

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
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED2_PIN, LOW);
  pinMode(LED3_PIN, OUTPUT);
  digitalWrite(LED3_PIN, LOW);
  pinMode(INPUT0_PIN, INPUT_PULLUP);
  pinMode(INPUT1_PIN, INPUT_PULLUP);
  pinMode(INPUT2_PIN, INPUT_PULLUP);
  pinMode(INPUT3_PIN, INPUT_PULLUP);
  pinMode(OUTPUT0_PIN, OUTPUT);
  digitalWrite(OUTPUT0_PIN, LOW);
  pinMode(OUTPUT1_PIN, OUTPUT);
  digitalWrite(OUTPUT1_PIN, LOW);

  /* Setup servo pins */
  servo0.attach(SERVO0_PIN, 800, 2200);
  servo1.attach(SERVO1_PIN, 800, 2200);
  servo0.writeMicroseconds(1500);
  servo1.writeMicroseconds(1500);

  /* create UAVCAN class */
  uc = new Node(99, [](CanardFrame const & frame) -> bool { return mcp2515.transmit(frame); });

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
  uc->subscribe<Bit_1_0<ID_OUTPUT0>>(onOutput0_Received);
  uc->subscribe<Bit_1_0<ID_OUTPUT1>>(onOutput1_Received);
  uc->subscribe<Integer16_1_0<ID_SERVO0>>(onServo0_Received);
  uc->subscribe<Integer16_1_0<ID_SERVO1>>(onServo1_Received);
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
  /* Publish all the gathered data, although at various
   * different intervals.
   */
  static unsigned long prev_led = 0;
  static unsigned long prev_led_toggle = 0;
  static unsigned long prev_hearbeat = 0;
  static unsigned long prev_battery_voltage = 0;
  static unsigned long prev_internal_temperature = 0;
  static unsigned long prev_input0 = 0;
  static unsigned long prev_input1 = 0;
  static unsigned long prev_input2 = 0;
  static unsigned long prev_input3 = 0;
  static unsigned long prev_analog_input0 = 0;
  static unsigned long prev_analog_input1 = 0;

  unsigned long const now = millis();

  /* toggle status LEDS */
  if((now - prev_led_toggle) > 200)
  {
    if(digitalRead(LED2_PIN)==LOW)
    {
      digitalWrite(LED2_PIN, HIGH);
      digitalWrite(LED3_PIN, LOW);
    }
    else
    {
      digitalWrite(LED2_PIN, LOW);
      digitalWrite(LED3_PIN, HIGH);
    }
    prev_led_toggle = now;
  }

  /* light mode for neopixels */
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
  if((now - prev_internal_temperature) > (10*1000))
  {
    float const temperature = analogReadTemp();
    Serial.print("Temperature: ");
    Serial.println(temperature);
    Real32_1_0<ID_INTERNAL_TEMPERATURE> uavcan_internal_temperature;
    uavcan_internal_temperature.data.value = temperature;
    uc->publish(uavcan_internal_temperature);
    prev_internal_temperature = now;
  }

  /* Handling of inputs */
  if((now - prev_input0) > 500)
  {
    Bit_1_0<ID_INPUT0> uavcan_input0;
    uavcan_input0.data.value = digitalRead(INPUT0_PIN);
    uc->publish(uavcan_input0);
    prev_input0 = now;
  }
  if((now - prev_input1) > 500)
  {
    Bit_1_0<ID_INPUT1> uavcan_input1;
    uavcan_input1.data.value = digitalRead(INPUT1_PIN);
    uc->publish(uavcan_input1);
    prev_input1 = now;
  }
  if((now - prev_input2) > 500)
  {
    Bit_1_0<ID_INPUT2> uavcan_input2;
    uavcan_input2.data.value = digitalRead(INPUT2_PIN);
    uc->publish(uavcan_input2);
    prev_input2 = now;
  }
  if((now - prev_input3) > 500)
  {
    Bit_1_0<ID_INPUT3> uavcan_input3;
    uavcan_input3.data.value = digitalRead(INPUT3_PIN);
    uc->publish(uavcan_input3);
    prev_input3 = now;
  }
  if((now - prev_analog_input0) > 500)
  {
    Integer16_1_0<ID_ANALOG_INPUT0> uavcan_analog_input0;
    uavcan_analog_input0.data.value = analogRead(ANALOG_INPUT0_PIN);
    uc->publish(uavcan_analog_input0);
    prev_analog_input0 = now;
  }
  if((now - prev_analog_input1) > 500)
  {
    Integer16_1_0<ID_ANALOG_INPUT1> uavcan_analog_input1;
    uavcan_analog_input1.data.value = analogRead(ANALOG_INPUT1_PIN);
    uc->publish(uavcan_analog_input1);
    prev_analog_input1 = now;
  }

  /* Transmit all enqeued CAN frames */
  while(uc->transmitCanFrame()) { }

  /* Feed the watchdog to keep it from biting. */
//  Watchdog.reset();
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onLed1_Received(CanardRxTransfer const & transfer, Node & /* node_hdl */)
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

void onOutput0_Received(CanardRxTransfer const & transfer, Node & /* node_hdl */)
{
  Bit_1_0<ID_OUTPUT0> const uavcan_output0 = Bit_1_0<ID_OUTPUT0>::deserialize(transfer);

  if(uavcan_output0.data.value) digitalWrite(OUTPUT0_PIN, HIGH);
  else digitalWrite(OUTPUT0_PIN, LOW);
}

void onOutput1_Received(CanardRxTransfer const & transfer, Node & /* node_hdl */)
{
  Bit_1_0<ID_OUTPUT1> const uavcan_output1 = Bit_1_0<ID_OUTPUT1>::deserialize(transfer);

  if(uavcan_output1.data.value) digitalWrite(OUTPUT1_PIN, HIGH);
  else digitalWrite(OUTPUT1_PIN, LOW);
}

void onServo0_Received(CanardRxTransfer const & transfer, Node & /* node_hdl */)
{
  Integer16_1_0<ID_SERVO0> const uavcan_servo0 = Integer16_1_0<ID_SERVO0>::deserialize(transfer);

  servo0.writeMicroseconds(uavcan_servo0.data.value);
}

void onServo1_Received(CanardRxTransfer const & transfer, Node & /* node_hdl */)
{
  Integer16_1_0<ID_SERVO1> const uavcan_servo1 = Integer16_1_0<ID_SERVO1>::deserialize(transfer);

  servo1.writeMicroseconds(uavcan_servo1.data.value);
}

void onLightMode_Received(CanardRxTransfer const & transfer, Node & /* node_hdl */)
{
  uavcan_light_mode = Integer8_1_0<ID_LIGHT_MODE>::deserialize(transfer);
}
