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
#include <Adafruit_NeoPixel.h>

#undef max
#undef min
#include <algorithm>

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
#define NEOPIXELPIN        13 // Raspberry Pi Pico

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 8 // Popular NeoPixel ring size

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;
using namespace uavcan::_register;
using namespace uavcan::primitive::scalar;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int const MKRCAN_MCP2515_CS_PIN  = 17;
static int const MKRCAN_MCP2515_INT_PIN = 20;

static CanardNodeID const AUX_CONTROLLER_NODE_ID = 99;

static CanardPortID const ID_INPUT_VOLTAGE        = 1001U;
static CanardPortID ID_INPUT_VOLTAGE_X        = 1001U;
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

static int8_t const LIGHT_MODE_RED         =   1;
static int8_t const LIGHT_MODE_GREEN       =   2;
static int8_t const LIGHT_MODE_BLUE        =   3;
static int8_t const LIGHT_MODE_WHITE       =   4;
static int8_t const LIGHT_MODE_AMBER       =   5;
static int8_t const LIGHT_MODE_BLINK_RED   =  11;
static int8_t const LIGHT_MODE_BLINK_GREEN =  12;
static int8_t const LIGHT_MODE_BLINK_BLUE  =  13;
static int8_t const LIGHT_MODE_BLINK_WHITE =  14;
static int8_t const LIGHT_MODE_BLINK_AMBER =  15;
static int8_t const LIGHT_MODE_RUN_RED     = 101;
static int8_t const LIGHT_MODE_RUN_GREEN   = 102;
static int8_t const LIGHT_MODE_RUN_BLUE    = 103;
static int8_t const LIGHT_MODE_RUN_WHITE   = 104;
static int8_t const LIGHT_MODE_RUN_AMBER   = 105;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const & frame);
void onLed1_Received (CanardRxTransfer const &, Node &);
void onOutput0_Received (CanardRxTransfer const &, Node &);
void onOutput1_Received (CanardRxTransfer const &, Node &);
void onServo0_Received (CanardRxTransfer const &, Node &);
void onServo1_Received (CanardRxTransfer const &, Node &);
void onLightMode_Received(CanardRxTransfer const &, Node &);

/* Cyphal Service Requests */
void onList_1_0_Request_Received(CanardRxTransfer const &, Node &);
void onGetInfo_1_0_Request_Received(CanardRxTransfer const &, Node &);
void onAccess_1_0_Request_Received(CanardRxTransfer const &, Node &);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

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
                       onReceiveBufferFull,
                       nullptr);

Node node_hdl([](CanardFrame const & frame) -> bool { return mcp2515.transmit(frame); }, AUX_CONTROLLER_NODE_ID);

static const uavcan_node_GetInfo_Response_1_0 GET_INFO_DATA = {
    /// uavcan.node.Version.1.0 protocol_version
    {1, 0},
    /// uavcan.node.Version.1.0 hardware_version
    {1, 0},
    /// uavcan.node.Version.1.0 software_version
    {0, 1},
    /// saturated uint64 software_vcs_revision_id
    NULL,
    /// saturated uint8[16] unique_id
    {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
     0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff},
    /// saturated uint8[<=50] name
    {
        "107-systems.l3xz-fw_aux-controller",
        strlen("107-systems.l3xz-fw_aux-controller")},
};

static uint16_t updateinterval_inputvoltage=3*1000;
static uint16_t updateinterval_internaltemperature=10*1000;
static uint16_t updateinterval_input0=500;
static uint16_t updateinterval_input1=500;
static uint16_t updateinterval_input2=500;
static uint16_t updateinterval_input3=500;
static uint16_t updateinterval_analoginput0=500;
static uint16_t updateinterval_analoginput1=500;
static uint16_t updateinterval_light=250;

/* REGISTER ***************************************************************************/

static RegisterNatural8  reg_rw_uavcan_node_id                        ("uavcan.node.id",                         Register::Access::ReadWrite, Register::Persistent::No, AUX_CONTROLLER_NODE_ID,                  [&node_hdl](uint8_t const & val) { node_hdl.setNodeId(val); }, nullptr , nullptr);
static RegisterString    reg_ro_uavcan_node_description               ("uavcan.node.description",                Register::Access::ReadWrite, Register::Persistent::No, "L3X-Z AUX_CONTROLLER",                  nullptr, nullptr , nullptr);
static RegisterNatural16 reg_ro_uavcan_pub_inputvoltage_id            ("uavcan.pub.inputvoltage.id",             Register::Access::ReadOnly,  Register::Persistent::No, ID_INPUT_VOLTAGE_X,                      nullptr, nullptr , nullptr);
static RegisterString    reg_ro_uavcan_pub_inputvoltage_type          ("uavcan.pub.inputvoltage.type",           Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Real32.1.0",    nullptr, nullptr , nullptr);
static RegisterNatural16 reg_ro_uavcan_pub_internaltemperature_id     ("uavcan.pub.internaltemperature.id",      Register::Access::ReadOnly,  Register::Persistent::No, ID_INTERNAL_TEMPERATURE,                 nullptr, nullptr , nullptr);
static RegisterString    reg_ro_uavcan_pub_internaltemperature_type   ("uavcan.pub.internaltemperature.type",    Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Real32.1.0",    nullptr, nullptr , nullptr);
static RegisterNatural16 reg_ro_uavcan_pub_input0_id                  ("uavcan.pub.input0.id",                   Register::Access::ReadOnly,  Register::Persistent::No, ID_INPUT0,                               nullptr, nullptr , nullptr);
static RegisterString    reg_ro_uavcan_pub_input0_type                ("uavcan.pub.input0.type",                 Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Bit.1.0",       nullptr, nullptr , nullptr);
static RegisterNatural16 reg_ro_uavcan_pub_input1_id                  ("uavcan.pub.input1.id",                   Register::Access::ReadOnly,  Register::Persistent::No, ID_INPUT1,                               nullptr, nullptr , nullptr);
static RegisterString    reg_ro_uavcan_pub_input1_type                ("uavcan.pub.input1.type",                 Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Bit.1.0",       nullptr, nullptr , nullptr);
static RegisterNatural16 reg_ro_uavcan_pub_input2_id                  ("uavcan.pub.input2.id",                   Register::Access::ReadOnly,  Register::Persistent::No, ID_INPUT2,                               nullptr, nullptr , nullptr);
static RegisterString    reg_ro_uavcan_pub_input2_type                ("uavcan.pub.input2.type",                 Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Bit.1.0",       nullptr, nullptr , nullptr);
static RegisterNatural16 reg_ro_uavcan_pub_input3_id                  ("uavcan.pub.input3.id",                   Register::Access::ReadOnly,  Register::Persistent::No, ID_INPUT3,                               nullptr, nullptr , nullptr);
static RegisterString    reg_ro_uavcan_pub_input3_type                ("uavcan.pub.input3.type",                 Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Bit.1.0",       nullptr, nullptr , nullptr);
static RegisterNatural16 reg_ro_uavcan_pub_analoginput0_id            ("uavcan.pub.analoginput0.id",             Register::Access::ReadOnly,  Register::Persistent::No, ID_ANALOG_INPUT0,                        nullptr, nullptr , nullptr);
static RegisterString    reg_ro_uavcan_pub_analoginput0_type          ("uavcan.pub.analoginput0.type",           Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Integer16.1.0", nullptr, nullptr , nullptr);
static RegisterNatural16 reg_ro_uavcan_pub_analoginput1_id            ("uavcan.pub.analoginput1.id",             Register::Access::ReadOnly,  Register::Persistent::No, ID_ANALOG_INPUT1,                        nullptr, nullptr , nullptr);
static RegisterString    reg_ro_uavcan_pub_analoginput1_type          ("uavcan.pub.analoginput1.type",           Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Integer16.1.0", nullptr, nullptr , nullptr);
static RegisterNatural16 reg_ro_uavcan_sub_led1_id                    ("uavcan.sub.led1.id",                     Register::Access::ReadOnly,  Register::Persistent::No, ID_LED1,                                 nullptr, nullptr , nullptr);
static RegisterString    reg_ro_uavcan_sub_led1_type                  ("uavcan.sub.led1.type",                   Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Bit.1.0",       nullptr, nullptr , nullptr);
static RegisterNatural16 reg_ro_uavcan_sub_output0_id                 ("uavcan.sub.output0.id",                  Register::Access::ReadOnly,  Register::Persistent::No, ID_OUTPUT0,                              nullptr, nullptr , nullptr);
static RegisterString    reg_ro_uavcan_sub_output0_type               ("uavcan.sub.output0.type",                Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Bit.1.0",       nullptr, nullptr , nullptr);
static RegisterNatural16 reg_ro_uavcan_sub_output1_id                 ("uavcan.sub.output1.id",                  Register::Access::ReadOnly,  Register::Persistent::No, ID_OUTPUT1,                              nullptr, nullptr , nullptr);
static RegisterString    reg_ro_uavcan_sub_output1_type               ("uavcan.sub.output1.type",                Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Bit.1.0",       nullptr, nullptr , nullptr);
static RegisterNatural16 reg_ro_uavcan_sub_servo0_id                  ("uavcan.sub.servo0.id",                   Register::Access::ReadOnly,  Register::Persistent::No, ID_SERVO0,                               nullptr, nullptr , nullptr);
static RegisterString    reg_ro_uavcan_sub_servo0_type                ("uavcan.sub.servo0.type",                 Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Integer16.1.0", nullptr, nullptr , nullptr);
static RegisterNatural16 reg_ro_uavcan_sub_servo1_id                  ("uavcan.sub.servo1.id",                   Register::Access::ReadOnly,  Register::Persistent::No, ID_SERVO1,                               nullptr, nullptr , nullptr);
static RegisterString    reg_ro_uavcan_sub_servo1_type                ("uavcan.sub.servo1.type",                 Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Integer16.1.0", nullptr, nullptr , nullptr);
static RegisterNatural16 reg_ro_uavcan_sub_lightmode_id               ("uavcan.sub.lightmode.id",                Register::Access::ReadOnly,  Register::Persistent::No, ID_LIGHT_MODE,                           nullptr, nullptr , nullptr);
static RegisterString    reg_ro_uavcan_sub_lightmode_type             ("uavcan.sub.lightmode.type",              Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Integer8.1.0",  nullptr, nullptr , nullptr);
static RegisterNatural16 reg_rw_aux_updateinterval_inputvoltage       ("aux.updateinterval.inputvoltage",        Register::Access::ReadWrite, Register::Persistent::No, updateinterval_inputvoltage,             nullptr, nullptr , [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(100)); });
static RegisterNatural16 reg_rw_aux_updateinterval_internaltemperature("aux.updateinterval.internaltemperature", Register::Access::ReadWrite, Register::Persistent::No, updateinterval_internaltemperature,      nullptr, nullptr , [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(100)); });
static RegisterNatural16 reg_rw_aux_updateinterval_input0             ("aux.updateinterval.input0",              Register::Access::ReadWrite, Register::Persistent::No, updateinterval_input0,                   nullptr, nullptr , [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(100)); });
static RegisterNatural16 reg_rw_aux_updateinterval_input1             ("aux.updateinterval.input1",              Register::Access::ReadWrite, Register::Persistent::No, updateinterval_input1,                   nullptr, nullptr , [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(100)); });
static RegisterNatural16 reg_rw_aux_updateinterval_input2             ("aux.updateinterval.input2",              Register::Access::ReadWrite, Register::Persistent::No, updateinterval_input2,                   nullptr, nullptr , [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(100)); });
static RegisterNatural16 reg_rw_aux_updateinterval_input3             ("aux.updateinterval.input3",              Register::Access::ReadWrite, Register::Persistent::No, updateinterval_input3,                   nullptr, nullptr , [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(100)); });
static RegisterNatural16 reg_rw_aux_updateinterval_analoginput0       ("aux.updateinterval.analoginput0",        Register::Access::ReadWrite, Register::Persistent::No, updateinterval_analoginput0,             nullptr, nullptr , [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(100)); });
static RegisterNatural16 reg_rw_aux_updateinterval_analoginput1       ("aux.updateinterval.analoginput1",        Register::Access::ReadWrite, Register::Persistent::No, updateinterval_analoginput1,             nullptr, nullptr , [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(100)); });
static RegisterNatural16 reg_rw_aux_updateinterval_light              ("aux.updateinterval.light",               Register::Access::ReadWrite, Register::Persistent::No, updateinterval_light,                    nullptr, nullptr , [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(100)); });
static RegisterList      reg_list;

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

Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXELPIN, NEO_GRB);

void light_off()
{
  pixels.clear();
  pixels.show();
}
void light_green()
{
  pixels.fill(pixels.Color(0, 55, 0));
  pixels.show();
}
void light_red()
{
  pixels.fill(pixels.Color(55, 0, 0));
  pixels.show();
}
void light_blue()
{
  pixels.fill(pixels.Color(0, 0, 55));
  pixels.show();
}
void light_white()
{
  pixels.fill(pixels.Color(55, 55, 55));
  pixels.show();
}
void light_amber()
{
  pixels.fill(pixels.Color(55, 40, 0));
  pixels.show();
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

  /* Setup SPI access */
  SPI.begin();
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), []() { mcp2515.onExternalEventHandler(); }, LOW);

  /* Initialize MCP2515 */
  mcp2515.begin();
//  mcp2515.setBitRate(CanBitRate::BR_1000kBPS_16MHZ);
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);
  mcp2515.setNormalMode();

  /* Configure initial heartbeat */
  uavcan_light_mode.data.value = LIGHT_MODE_RUN_BLUE;

  hb.data.uptime = 0;
  hb = Heartbeat_1_0<>::Health::NOMINAL;
  hb = Heartbeat_1_0<>::Mode::INITIALIZATION;
  hb.data.vendor_specific_status_code = 0;

  /* Subscribe to the GetInfo request */
  node_hdl.subscribe<GetInfo_1_0::Request<>>(onGetInfo_1_0_Request_Received);
  reg_list.subscribe(node_hdl);
  reg_list.add(reg_rw_uavcan_node_id);
  reg_list.add(reg_ro_uavcan_node_description);
  reg_list.add(reg_ro_uavcan_pub_inputvoltage_id);
  reg_list.add(reg_ro_uavcan_pub_internaltemperature_id);
  reg_list.add(reg_ro_uavcan_pub_input0_id);
  reg_list.add(reg_ro_uavcan_pub_input1_id);
  reg_list.add(reg_ro_uavcan_pub_input2_id);
  reg_list.add(reg_ro_uavcan_pub_input3_id);
  reg_list.add(reg_ro_uavcan_pub_analoginput0_id);
  reg_list.add(reg_ro_uavcan_pub_analoginput1_id);
  reg_list.add(reg_ro_uavcan_sub_led1_id);
  reg_list.add(reg_ro_uavcan_sub_output0_id);
  reg_list.add(reg_ro_uavcan_sub_output1_id);
  reg_list.add(reg_ro_uavcan_sub_servo0_id);
  reg_list.add(reg_ro_uavcan_sub_servo1_id);
  reg_list.add(reg_ro_uavcan_sub_lightmode_id);
  reg_list.add(reg_ro_uavcan_pub_inputvoltage_type);
  reg_list.add(reg_ro_uavcan_pub_internaltemperature_type);
  reg_list.add(reg_ro_uavcan_pub_input0_type);
  reg_list.add(reg_ro_uavcan_pub_input1_type);
  reg_list.add(reg_ro_uavcan_pub_input2_type);
  reg_list.add(reg_ro_uavcan_pub_input3_type);
  reg_list.add(reg_ro_uavcan_pub_analoginput0_type);
  reg_list.add(reg_ro_uavcan_pub_analoginput1_type);
  reg_list.add(reg_ro_uavcan_sub_led1_type);
  reg_list.add(reg_ro_uavcan_sub_output0_type);
  reg_list.add(reg_ro_uavcan_sub_output1_type);
  reg_list.add(reg_ro_uavcan_sub_servo0_type);
  reg_list.add(reg_ro_uavcan_sub_servo1_type);
  reg_list.add(reg_ro_uavcan_sub_lightmode_type);
  reg_list.add(reg_rw_aux_updateinterval_inputvoltage);
  reg_list.add(reg_rw_aux_updateinterval_internaltemperature);
  reg_list.add(reg_rw_aux_updateinterval_input0);
  reg_list.add(reg_rw_aux_updateinterval_input1);
  reg_list.add(reg_rw_aux_updateinterval_input2);
  reg_list.add(reg_rw_aux_updateinterval_input3);
  reg_list.add(reg_rw_aux_updateinterval_analoginput0);
  reg_list.add(reg_rw_aux_updateinterval_analoginput1);
  reg_list.add(reg_rw_aux_updateinterval_light);
  /* Subscribe to the reception of Bit message. */
  node_hdl.subscribe<Bit_1_0<ID_LED1>>(onLed1_Received);
  node_hdl.subscribe<Bit_1_0<ID_OUTPUT0>>(onOutput0_Received);
  node_hdl.subscribe<Bit_1_0<ID_OUTPUT1>>(onOutput1_Received);
  node_hdl.subscribe<Integer16_1_0<ID_SERVO0>>(onServo0_Received);
  node_hdl.subscribe<Integer16_1_0<ID_SERVO1>>(onServo1_Received);
  node_hdl.subscribe<Integer8_1_0<ID_LIGHT_MODE>>(onLightMode_Received);

  /* Init Neopixel */
  pixels.begin();

  light_red();
  delay(100);
  light_amber();
  delay(100);
  light_green();
  delay(100);
  light_blue();
  delay(100);
  light_white();
  delay(100);
  light_off();
}

void loop()
{
  /* Process all pending OpenCyphal actions.
   */
  node_hdl.spinSome();

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
  if((now - prev_led) > updateinterval_light)
  {
    static bool is_light_on = false;
    is_light_on = !is_light_on;
    static int running_light_counter = 0;
    running_light_counter ++;
    if(running_light_counter>=8) running_light_counter=0;

    if (uavcan_light_mode.data.value == LIGHT_MODE_RED)
      light_red();
    else if (uavcan_light_mode.data.value == LIGHT_MODE_GREEN)
      light_green();
    else if (uavcan_light_mode.data.value == LIGHT_MODE_BLUE)
      light_blue();
    else if (uavcan_light_mode.data.value == LIGHT_MODE_WHITE)
      light_white();
    else if (uavcan_light_mode.data.value == LIGHT_MODE_AMBER)
      light_amber();
    else if (uavcan_light_mode.data.value == LIGHT_MODE_RUN_RED||uavcan_light_mode.data.value == LIGHT_MODE_RUN_GREEN||uavcan_light_mode.data.value == LIGHT_MODE_RUN_BLUE||uavcan_light_mode.data.value == LIGHT_MODE_RUN_WHITE||uavcan_light_mode.data.value == LIGHT_MODE_RUN_AMBER)
    {
      if (uavcan_light_mode.data.value == LIGHT_MODE_RUN_RED)
      {
        pixels.setPixelColor(running_light_counter, pixels.Color(55, 0, 0));
        pixels.setPixelColor((running_light_counter+7)%8, pixels.Color(27, 0, 0));
        pixels.setPixelColor((running_light_counter+6)%8, pixels.Color(14, 0, 0));
        pixels.setPixelColor((running_light_counter+5)%8, pixels.Color(7, 0, 0));
        pixels.setPixelColor((running_light_counter+4)%8, pixels.Color(0, 0, 0));
        pixels.show();
      }
      else if (uavcan_light_mode.data.value == LIGHT_MODE_RUN_GREEN)
      {
        pixels.setPixelColor(running_light_counter, pixels.Color(0, 55, 0));
        pixels.setPixelColor((running_light_counter+7)%8, pixels.Color(0, 27, 0));
        pixels.setPixelColor((running_light_counter+6)%8, pixels.Color(0, 14, 0));
        pixels.setPixelColor((running_light_counter+5)%8, pixels.Color(0, 7, 0));
        pixels.setPixelColor((running_light_counter+4)%8, pixels.Color(0, 0, 0));
        pixels.show();
      }
      else if (uavcan_light_mode.data.value == LIGHT_MODE_RUN_BLUE)
      {
        pixels.setPixelColor(running_light_counter, pixels.Color(0, 0, 55));
        pixels.setPixelColor((running_light_counter+7)%8, pixels.Color(0, 0, 27));
        pixels.setPixelColor((running_light_counter+6)%8, pixels.Color(0, 0, 14));
        pixels.setPixelColor((running_light_counter+5)%8, pixels.Color(0, 0, 7));
        pixels.setPixelColor((running_light_counter+4)%8, pixels.Color(0, 0, 0));
        pixels.show();
      }
      else if (uavcan_light_mode.data.value == LIGHT_MODE_RUN_WHITE)
      {
        pixels.setPixelColor(running_light_counter, pixels.Color(55, 55, 55));
        pixels.setPixelColor((running_light_counter+7)%8, pixels.Color(27, 27, 27));
        pixels.setPixelColor((running_light_counter+6)%8, pixels.Color(14, 14, 14));
        pixels.setPixelColor((running_light_counter+5)%8, pixels.Color(7, 7, 7));
        pixels.setPixelColor((running_light_counter+4)%8, pixels.Color(0, 0, 0));
        pixels.show();
      }
      else if (uavcan_light_mode.data.value == LIGHT_MODE_RUN_AMBER)
      {
        pixels.setPixelColor(running_light_counter, pixels.Color(55, 40, 0));
        pixels.setPixelColor((running_light_counter+7)%8, pixels.Color(27, 20, 0));
        pixels.setPixelColor((running_light_counter+6)%8, pixels.Color(14, 10, 0));
        pixels.setPixelColor((running_light_counter+5)%8, pixels.Color(7, 5, 0));
        pixels.setPixelColor((running_light_counter+4)%8, pixels.Color(0, 0, 0));
        pixels.show();
      }
    }
    else if (is_light_on&&(uavcan_light_mode.data.value == LIGHT_MODE_BLINK_RED||uavcan_light_mode.data.value == LIGHT_MODE_BLINK_GREEN||uavcan_light_mode.data.value == LIGHT_MODE_BLINK_BLUE||uavcan_light_mode.data.value == LIGHT_MODE_BLINK_WHITE||uavcan_light_mode.data.value == LIGHT_MODE_BLINK_AMBER))
    {
      if (uavcan_light_mode.data.value == LIGHT_MODE_BLINK_GREEN)
        light_green();
      else if (uavcan_light_mode.data.value == LIGHT_MODE_BLINK_BLUE)
        light_blue();
      else if (uavcan_light_mode.data.value == LIGHT_MODE_BLINK_WHITE)
        light_white();
      else if (uavcan_light_mode.data.value == LIGHT_MODE_BLINK_AMBER)
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
    node_hdl.publish(hb);
    prev_hearbeat = now;
  }
  if((now - prev_battery_voltage) > (updateinterval_inputvoltage))
  {
    float const analog = analogRead(ANALOG_PIN)*3.3*11.0/1023.0;
    Serial.print("Analog Pin: ");
    Serial.println(analog);
    Real32_1_0<ID_INPUT_VOLTAGE> uavcan_input_voltage;
    uavcan_input_voltage.data.value = analog;
    node_hdl.publish(uavcan_input_voltage);
    prev_battery_voltage = now;
  }
  if((now - prev_internal_temperature) > (updateinterval_internaltemperature))
  {
    float const temperature = analogReadTemp();
    Serial.print("Temperature: ");
    Serial.println(temperature);
    Real32_1_0<ID_INTERNAL_TEMPERATURE> uavcan_internal_temperature;
    uavcan_internal_temperature.data.value = temperature;
    node_hdl.publish(uavcan_internal_temperature);
    prev_internal_temperature = now;
  }

  /* Handling of inputs */
  if((now - prev_input0) > updateinterval_input0)
  {
    Bit_1_0<ID_INPUT0> uavcan_input0;
    uavcan_input0.data.value = digitalRead(INPUT0_PIN);
    node_hdl.publish(uavcan_input0);
    prev_input0 = now;
  }
  if((now - prev_input1) > updateinterval_input1)
  {
    Bit_1_0<ID_INPUT1> uavcan_input1;
    uavcan_input1.data.value = digitalRead(INPUT1_PIN);
    node_hdl.publish(uavcan_input1);
    prev_input1 = now;
  }
  if((now - prev_input2) > updateinterval_input2)
  {
    Bit_1_0<ID_INPUT2> uavcan_input2;
    uavcan_input2.data.value = digitalRead(INPUT2_PIN);
    node_hdl.publish(uavcan_input2);
    prev_input2 = now;
  }
  if((now - prev_input3) > updateinterval_input3)
  {
    Bit_1_0<ID_INPUT3> uavcan_input3;
    uavcan_input3.data.value = digitalRead(INPUT3_PIN);
    node_hdl.publish(uavcan_input3);
    prev_input3 = now;
  }
  if((now - prev_analog_input0) > updateinterval_analoginput0)
  {
    Integer16_1_0<ID_ANALOG_INPUT0> uavcan_analog_input0;
    uavcan_analog_input0.data.value = analogRead(ANALOG_INPUT0_PIN);
    node_hdl.publish(uavcan_analog_input0);
    prev_analog_input0 = now;
  }
  if((now - prev_analog_input1) > updateinterval_analoginput1)
  {
    Integer16_1_0<ID_ANALOG_INPUT1> uavcan_analog_input1;
    uavcan_analog_input1.data.value = analogRead(ANALOG_INPUT1_PIN);
    node_hdl.publish(uavcan_analog_input1);
    prev_analog_input1 = now;
  }

  /* Feed the watchdog to keep it from biting. */
//  Watchdog.reset();
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const & frame)
{
  node_hdl.onCanFrameReceived(frame, micros());
}

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

void onGetInfo_1_0_Request_Received(CanardRxTransfer const &transfer, Node & node_hdl)
{
  GetInfo_1_0::Response<> rsp = GetInfo_1_0::Response<>();
  rsp.data = GET_INFO_DATA;
  Serial.println("onGetInfo_1_0_Request_Received");
  node_hdl.respond(rsp, transfer.metadata.remote_node_id, transfer.metadata.transfer_id);
}
