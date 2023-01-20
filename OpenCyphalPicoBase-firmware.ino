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

#include <pico/stdlib.h>
#include <hardware/watchdog.h>

#include <SPI.h>
#include <Wire.h>
#include <Servo.h>

#include <I2C_eeprom.h>
#include <107-Arduino-Cyphal.h>
#include <107-Arduino-MCP2515.h>
#include <107-Arduino-UniqueId.h>
#include <107-Arduino-CriticalSection.h>

#undef max
#undef min
#include <algorithm>

#include "src/PortId.h"
#include "src/ServoControl.h"
#include "src/NeoPixelControl.h"
#include "src/DigitalOutControl.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;
using namespace uavcan::_register;
using namespace uavcan::primitive::scalar;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int const INPUT0_PIN        =  6;
static int const INPUT1_PIN        =  7;
static int const INPUT2_PIN        =  8;
static int const INPUT3_PIN        =  9;
static int const OUTPUT0_PIN       = 10;
static int const OUTPUT1_PIN       = 11;
static int const NEOPIXEL_PIN      = 13; /* Raspberry Pi Pico */
static int const SERVO0_PIN        = 14;
static int const SERVO1_PIN        = 15;
static int const MCP2515_CS_PIN    = 17;
static int const MCP2515_INT_PIN   = 20;
static int const LED2_PIN          = 21;
static int const LED3_PIN          = 22;
static int const ANALOG_PIN        = 26;
static int const ANALOG_INPUT0_PIN = 27;
static int const ANALOG_INPUT1_PIN = 28;

static int const NEOPIXEL_NUM_PIXELS = 8; /* Popular NeoPixel ring size */

static CanardNodeID const DEFAULT_AUX_CONTROLLER_NODE_ID = 99;

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
/* Cyphal Subscription Callbacks */
void onLed1_Received(Bit_1_0<ID_LED1> const & msg);
void onLightMode_Received(Integer8_1_0<ID_LIGHT_MODE> const & msg);
/* Cyphal Service Request Callbacks */
ExecuteCommand_1_0::Response<> onExecuteCommand_1_0_Request_Received(ExecuteCommand_1_0::Request<> const &);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ArduinoMCP2515 mcp2515([]()
                       {
                         noInterrupts();
                         SPI.beginTransaction(MCP2515x_SPI_SETTING);
                         digitalWrite(MCP2515_CS_PIN, LOW);
                       },
                       []()
                       {
                         digitalWrite(MCP2515_CS_PIN, HIGH);
                         SPI.endTransaction();
                         interrupts();
                       },
                       [](uint8_t const d) { return SPI.transfer(d); },
                       micros,
                       onReceiveBufferFull,
                       nullptr);

Node::Heap<Node::DEFAULT_O1HEAP_SIZE> node_heap;
Node node_hdl(node_heap.data(), node_heap.size(), micros, [] (CanardFrame const & frame) { return mcp2515.transmit(frame); }, DEFAULT_AUX_CONTROLLER_NODE_ID);

Publisher<Heartbeat_1_0<>> heartbeat_pub = node_hdl.create_publisher<Heartbeat_1_0<>>
  (Heartbeat_1_0<>::PORT_ID, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Real32_1_0<ID_INPUT_VOLTAGE>> input_voltage_pub = node_hdl.create_publisher<Real32_1_0<ID_INPUT_VOLTAGE>>
  (ID_INPUT_VOLTAGE, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Real32_1_0<ID_INTERNAL_TEMPERATURE>> internal_temperature_pub = node_hdl.create_publisher<Real32_1_0<ID_INTERNAL_TEMPERATURE>>
  (ID_INTERNAL_TEMPERATURE, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Bit_1_0<ID_INPUT0>> input_0_pub = node_hdl.create_publisher<Bit_1_0<ID_INPUT0>>
  (ID_INPUT0, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Bit_1_0<ID_INPUT1>> input_1_pub = node_hdl.create_publisher<Bit_1_0<ID_INPUT1>>
  (ID_INPUT1, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Bit_1_0<ID_INPUT2>> input_2_pub = node_hdl.create_publisher<Bit_1_0<ID_INPUT2>>
  (ID_INPUT2, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Bit_1_0<ID_INPUT3>> input_3_pub = node_hdl.create_publisher<Bit_1_0<ID_INPUT3>>
  (ID_INPUT3, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Integer16_1_0<ID_ANALOG_INPUT0>> analog_input_0_pub = node_hdl.create_publisher<Integer16_1_0<ID_ANALOG_INPUT0>>
  (ID_ANALOG_INPUT0, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Integer16_1_0<ID_ANALOG_INPUT1>> analog_input_1_pub = node_hdl.create_publisher<Integer16_1_0<ID_ANALOG_INPUT1>>
  (ID_ANALOG_INPUT1, 1*1000*1000UL /* = 1 sec in usecs. */);

Subscription led_subscription =
  node_hdl.create_subscription<Bit_1_0<ID_LED1>>(ID_LED1, CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, onLed1_Received);

static Integer8_1_0<ID_LIGHT_MODE> light_mode_msg;
Subscription light_mode_subscription =
  node_hdl.create_subscription<Integer8_1_0<ID_LIGHT_MODE>>(
    ID_LIGHT_MODE,
    CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
    [&light_mode_msg](Integer8_1_0<ID_LIGHT_MODE> const & msg)
    {
      light_mode_msg = msg;
    });

ServiceServer execute_command_srv = node_hdl.create_service_server<ExecuteCommand_1_0::Request<>, ExecuteCommand_1_0::Response<>>(
  ExecuteCommand_1_0::Request<>::PORT_ID,
  2*1000*1000UL,
  onExecuteCommand_1_0_Request_Received);


ServoControl servo_ctrl(SERVO0_PIN, SERVO1_PIN, node_hdl);
DigitalOutControl digital_out_ctrl(OUTPUT0_PIN, OUTPUT1_PIN, node_hdl);
NeoPixelControl neo_pixel_ctrl(NEOPIXEL_PIN, NEOPIXEL_NUM_PIXELS);


static uint16_t update_period_ms_inputvoltage        =  3*1000;
static uint16_t update_period_ms_internaltemperature = 10*1000;
static uint16_t update_period_ms_input0              =     500;
static uint16_t update_period_ms_input1              =     500;
static uint16_t update_period_ms_input2              =     500;
static uint16_t update_period_ms_input3              =     500;
static uint16_t update_period_ms_analoginput0        =     500;
static uint16_t update_period_ms_analoginput1        =     500;
static uint16_t update_period_ms_light               =     250;

/* REGISTER ***************************************************************************/

static RegisterNatural8  reg_rw_uavcan_node_id                          ("uavcan.node.id",                         Register::Access::ReadWrite, Register::Persistent::No, DEFAULT_AUX_CONTROLLER_NODE_ID, [&node_hdl](uint8_t const & val) { node_hdl.setNodeId(val); });
static RegisterString    reg_ro_uavcan_node_description                 ("uavcan.node.description",                Register::Access::ReadWrite, Register::Persistent::No, "L3X-Z AUX_CONTROLLER");
static RegisterNatural16 reg_ro_uavcan_pub_inputvoltage_id              ("uavcan.pub.inputvoltage.id",             Register::Access::ReadOnly,  Register::Persistent::No, ID_INPUT_VOLTAGE);
static RegisterString    reg_ro_uavcan_pub_inputvoltage_type            ("uavcan.pub.inputvoltage.type",           Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Real32.1.0");
static RegisterNatural16 reg_ro_uavcan_pub_internaltemperature_id       ("uavcan.pub.internaltemperature.id",      Register::Access::ReadOnly,  Register::Persistent::No, ID_INTERNAL_TEMPERATURE);
static RegisterString    reg_ro_uavcan_pub_internaltemperature_type     ("uavcan.pub.internaltemperature.type",    Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Real32.1.0");
static RegisterNatural16 reg_ro_uavcan_pub_input0_id                    ("uavcan.pub.input0.id",                   Register::Access::ReadOnly,  Register::Persistent::No, ID_INPUT0);
static RegisterString    reg_ro_uavcan_pub_input0_type                  ("uavcan.pub.input0.type",                 Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Bit.1.0");
static RegisterNatural16 reg_ro_uavcan_pub_input1_id                    ("uavcan.pub.input1.id",                   Register::Access::ReadOnly,  Register::Persistent::No, ID_INPUT1);
static RegisterString    reg_ro_uavcan_pub_input1_type                  ("uavcan.pub.input1.type",                 Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Bit.1.0");
static RegisterNatural16 reg_ro_uavcan_pub_input2_id                    ("uavcan.pub.input2.id",                   Register::Access::ReadOnly,  Register::Persistent::No, ID_INPUT2);
static RegisterString    reg_ro_uavcan_pub_input2_type                  ("uavcan.pub.input2.type",                 Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Bit.1.0");
static RegisterNatural16 reg_ro_uavcan_pub_input3_id                    ("uavcan.pub.input3.id",                   Register::Access::ReadOnly,  Register::Persistent::No, ID_INPUT3);
static RegisterString    reg_ro_uavcan_pub_input3_type                  ("uavcan.pub.input3.type",                 Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Bit.1.0");
static RegisterNatural16 reg_ro_uavcan_pub_analoginput0_id              ("uavcan.pub.analoginput0.id",             Register::Access::ReadOnly,  Register::Persistent::No, ID_ANALOG_INPUT0);
static RegisterString    reg_ro_uavcan_pub_analoginput0_type            ("uavcan.pub.analoginput0.type",           Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Integer16.1.0");
static RegisterNatural16 reg_ro_uavcan_pub_analoginput1_id              ("uavcan.pub.analoginput1.id",             Register::Access::ReadOnly,  Register::Persistent::No, ID_ANALOG_INPUT1);
static RegisterString    reg_ro_uavcan_pub_analoginput1_type            ("uavcan.pub.analoginput1.type",           Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Integer16.1.0");
static RegisterNatural16 reg_ro_uavcan_sub_led1_id                      ("uavcan.sub.led1.id",                     Register::Access::ReadOnly,  Register::Persistent::No, ID_LED1);
static RegisterString    reg_ro_uavcan_sub_led1_type                    ("uavcan.sub.led1.type",                   Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Bit.1.0");
static RegisterNatural16 reg_ro_uavcan_sub_output0_id                   ("uavcan.sub.output0.id",                  Register::Access::ReadOnly,  Register::Persistent::No, ID_OUTPUT0);
static RegisterString    reg_ro_uavcan_sub_output0_type                 ("uavcan.sub.output0.type",                Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Bit.1.0");
static RegisterNatural16 reg_ro_uavcan_sub_output1_id                   ("uavcan.sub.output1.id",                  Register::Access::ReadOnly,  Register::Persistent::No, ID_OUTPUT1);
static RegisterString    reg_ro_uavcan_sub_output1_type                 ("uavcan.sub.output1.type",                Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Bit.1.0");
static RegisterNatural16 reg_ro_uavcan_sub_servo0_id                    ("uavcan.sub.servo0.id",                   Register::Access::ReadOnly,  Register::Persistent::No, ID_SERVO0);
static RegisterString    reg_ro_uavcan_sub_servo0_type                  ("uavcan.sub.servo0.type",                 Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Integer16.1.0");
static RegisterNatural16 reg_ro_uavcan_sub_servo1_id                    ("uavcan.sub.servo1.id",                   Register::Access::ReadOnly,  Register::Persistent::No, ID_SERVO1);
static RegisterString    reg_ro_uavcan_sub_servo1_type                  ("uavcan.sub.servo1.type",                 Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Integer16.1.0");
static RegisterNatural16 reg_ro_uavcan_sub_lightmode_id                 ("uavcan.sub.lightmode.id",                Register::Access::ReadOnly,  Register::Persistent::No, ID_LIGHT_MODE);
static RegisterString    reg_ro_uavcan_sub_lightmode_type               ("uavcan.sub.lightmode.type",              Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Integer8.1.0");
static RegisterNatural16 reg_rw_aux_update_period_ms_inputvoltage       ("aux.update_period_ms.inputvoltage",        Register::Access::ReadWrite, Register::Persistent::No, update_period_ms_inputvoltage,        nullptr, nullptr , [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(100)); });
static RegisterNatural16 reg_rw_aux_update_period_ms_internaltemperature("aux.update_period_ms.internaltemperature", Register::Access::ReadWrite, Register::Persistent::No, update_period_ms_internaltemperature, nullptr, nullptr , [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(100)); });
static RegisterNatural16 reg_rw_aux_update_period_ms_input0             ("aux.update_period_ms.input0",              Register::Access::ReadWrite, Register::Persistent::No, update_period_ms_input0,              nullptr, nullptr , [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(100)); });
static RegisterNatural16 reg_rw_aux_update_period_ms_input1             ("aux.update_period_ms.input1",              Register::Access::ReadWrite, Register::Persistent::No, update_period_ms_input1,              nullptr, nullptr , [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(100)); });
static RegisterNatural16 reg_rw_aux_update_period_ms_input2             ("aux.update_period_ms.input2",              Register::Access::ReadWrite, Register::Persistent::No, update_period_ms_input2,              nullptr, nullptr , [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(100)); });
static RegisterNatural16 reg_rw_aux_update_period_ms_input3             ("aux.update_period_ms.input3",              Register::Access::ReadWrite, Register::Persistent::No, update_period_ms_input3,              nullptr, nullptr , [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(100)); });
static RegisterNatural16 reg_rw_aux_update_period_ms_analoginput0       ("aux.update_period_ms.analoginput0",        Register::Access::ReadWrite, Register::Persistent::No, update_period_ms_analoginput0,        nullptr, nullptr , [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(100)); });
static RegisterNatural16 reg_rw_aux_update_period_ms_analoginput1       ("aux.update_period_ms.analoginput1",        Register::Access::ReadWrite, Register::Persistent::No, update_period_ms_analoginput1,        nullptr, nullptr , [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(100)); });
static RegisterNatural16 reg_rw_aux_update_period_ms_light              ("aux.update_period_ms.light",               Register::Access::ReadWrite, Register::Persistent::No, update_period_ms_light,               nullptr, nullptr , [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(100)); });
static RegisterList      reg_list(node_hdl);

/* NODE INFO **************************************************************************/

static NodeInfo node_info
(
  node_hdl,
  /* uavcan.node.Version.1.0 protocol_version */
  1, 0,
  /* uavcan.node.Version.1.0 hardware_version */
  1, 0,
  /* uavcan.node.Version.1.0 software_version */
  0, 1,
  /* saturated uint64 software_vcs_revision_id */
  0,
  /* saturated uint8[16] unique_id */
  OpenCyphalUniqueId(),
  /* saturated uint8[<=50] name */
  "107-systems.l3xz-fw_aux-controller"
);

Heartbeat_1_0<> hb_msg;

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  //while (!Serial) { }

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

  servo_ctrl.begin();
  digital_out_ctrl.begin();
  neo_pixel_ctrl.begin();

  /* Setup SPI access */
  SPI.begin();
  pinMode(MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MCP2515_CS_PIN, HIGH);

  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  pinMode(MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MCP2515_INT_PIN), []() { mcp2515.onExternalEventHandler(); }, LOW);

  /* Initialize MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);
  mcp2515.setNormalMode();

  /* Configure initial heartbeat */
  light_mode_msg.data.value = LIGHT_MODE_RUN_BLUE;

  hb_msg.data.uptime = 0;
  hb_msg.data.health.value = uavcan_node_Health_1_0_NOMINAL;
  hb_msg.data.mode.value = uavcan_node_Mode_1_0_INITIALIZATION;
  hb_msg.data.vendor_specific_status_code = 0;

  /* Register callbacks.
   */
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
  reg_list.add(reg_rw_aux_update_period_ms_inputvoltage);
  reg_list.add(reg_rw_aux_update_period_ms_internaltemperature);
  reg_list.add(reg_rw_aux_update_period_ms_input0);
  reg_list.add(reg_rw_aux_update_period_ms_input1);
  reg_list.add(reg_rw_aux_update_period_ms_input2);
  reg_list.add(reg_rw_aux_update_period_ms_input3);
  reg_list.add(reg_rw_aux_update_period_ms_analoginput0);
  reg_list.add(reg_rw_aux_update_period_ms_analoginput1);
  reg_list.add(reg_rw_aux_update_period_ms_light);

  neo_pixel_ctrl.light_red();
  delay(100);
  neo_pixel_ctrl.light_amber();
  delay(100);
  neo_pixel_ctrl.light_green();
  delay(100);
  neo_pixel_ctrl.light_blue();
  delay(100);
  neo_pixel_ctrl.light_white();
  delay(100);
  neo_pixel_ctrl.light_off();
}

void loop()
{
  /* Process all pending OpenCyphal actions.
   */
  {
    CriticalSection crit_sec;
    node_hdl.spinSome();
  }

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
  if((now - prev_led) > update_period_ms_light)
  {
    static bool is_light_on = false;
    is_light_on = !is_light_on;
    static int running_light_counter = 0;
    running_light_counter ++;
    if(running_light_counter>=8) running_light_counter=0;

    if      (light_mode_msg.data.value == LIGHT_MODE_RED)
      neo_pixel_ctrl.light_red();
    else if (light_mode_msg.data.value == LIGHT_MODE_GREEN)
      neo_pixel_ctrl.light_green();
    else if (light_mode_msg.data.value == LIGHT_MODE_BLUE)
      neo_pixel_ctrl.light_blue();
    else if (light_mode_msg.data.value == LIGHT_MODE_WHITE)
      neo_pixel_ctrl.light_white();
    else if (light_mode_msg.data.value == LIGHT_MODE_AMBER)
      neo_pixel_ctrl.light_amber();
    else if (light_mode_msg.data.value == LIGHT_MODE_RUN_RED||light_mode_msg.data.value == LIGHT_MODE_RUN_GREEN||light_mode_msg.data.value == LIGHT_MODE_RUN_BLUE||light_mode_msg.data.value == LIGHT_MODE_RUN_WHITE||light_mode_msg.data.value == LIGHT_MODE_RUN_AMBER)
    {
      if (light_mode_msg.data.value == LIGHT_MODE_RUN_RED)
      {
        neo_pixel_ctrl.pixels().setPixelColor(running_light_counter,       neo_pixel_ctrl.pixels().Color(55, 0, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+7)%8, neo_pixel_ctrl.pixels().Color(27, 0, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+6)%8, neo_pixel_ctrl.pixels().Color(14, 0, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+5)%8, neo_pixel_ctrl.pixels().Color(7, 0, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+4)%8, neo_pixel_ctrl.pixels().Color(0, 0, 0));
        neo_pixel_ctrl.pixels().show();
      }
      else if (light_mode_msg.data.value == LIGHT_MODE_RUN_GREEN)
      {
        neo_pixel_ctrl.pixels().setPixelColor(running_light_counter,       neo_pixel_ctrl.pixels().Color(0, 55, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+7)%8, neo_pixel_ctrl.pixels().Color(0, 27, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+6)%8, neo_pixel_ctrl.pixels().Color(0, 14, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+5)%8, neo_pixel_ctrl.pixels().Color(0, 7, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+4)%8, neo_pixel_ctrl.pixels().Color(0, 0, 0));
        neo_pixel_ctrl.pixels().show();
      }
      else if (light_mode_msg.data.value == LIGHT_MODE_RUN_BLUE)
      {
        neo_pixel_ctrl.pixels().setPixelColor(running_light_counter,       neo_pixel_ctrl.pixels().Color(0, 0, 55));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+7)%8, neo_pixel_ctrl.pixels().Color(0, 0, 27));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+6)%8, neo_pixel_ctrl.pixels().Color(0, 0, 14));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+5)%8, neo_pixel_ctrl.pixels().Color(0, 0, 7));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+4)%8, neo_pixel_ctrl.pixels().Color(0, 0, 0));
        neo_pixel_ctrl.pixels().show();
      }
      else if (light_mode_msg.data.value == LIGHT_MODE_RUN_WHITE)
      {
        neo_pixel_ctrl.pixels().setPixelColor(running_light_counter,       neo_pixel_ctrl.pixels().Color(55, 55, 55));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+7)%8, neo_pixel_ctrl.pixels().Color(27, 27, 27));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+6)%8, neo_pixel_ctrl.pixels().Color(14, 14, 14));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+5)%8, neo_pixel_ctrl.pixels().Color(7, 7, 7));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+4)%8, neo_pixel_ctrl.pixels().Color(0, 0, 0));
        neo_pixel_ctrl.pixels().show();
      }
      else if (light_mode_msg.data.value == LIGHT_MODE_RUN_AMBER)
      {
        neo_pixel_ctrl.pixels().setPixelColor(running_light_counter,       neo_pixel_ctrl.pixels().Color(55, 40, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+7)%8, neo_pixel_ctrl.pixels().Color(27, 20, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+6)%8, neo_pixel_ctrl.pixels().Color(14, 10, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+5)%8, neo_pixel_ctrl.pixels().Color(7, 5, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+4)%8, neo_pixel_ctrl.pixels().Color(0, 0, 0));
        neo_pixel_ctrl.pixels().show();
      }
    }
    else if (is_light_on&&(light_mode_msg.data.value == LIGHT_MODE_BLINK_RED||light_mode_msg.data.value == LIGHT_MODE_BLINK_GREEN||light_mode_msg.data.value == LIGHT_MODE_BLINK_BLUE||light_mode_msg.data.value == LIGHT_MODE_BLINK_WHITE||light_mode_msg.data.value == LIGHT_MODE_BLINK_AMBER))
    {
      if (light_mode_msg.data.value == LIGHT_MODE_BLINK_GREEN)
        neo_pixel_ctrl.light_green();
      else if (light_mode_msg.data.value == LIGHT_MODE_BLINK_BLUE)
        neo_pixel_ctrl.light_blue();
      else if (light_mode_msg.data.value == LIGHT_MODE_BLINK_WHITE)
        neo_pixel_ctrl.light_white();
      else if (light_mode_msg.data.value == LIGHT_MODE_BLINK_AMBER)
        neo_pixel_ctrl.light_amber();
      else
        neo_pixel_ctrl.light_red();
    }
    else
      neo_pixel_ctrl.light_off();

    prev_led = now;
  }

  /* Update the heartbeat object */
  hb_msg.data.uptime = millis() / 1000;
  hb_msg.data.mode.value = uavcan_node_Mode_1_0_OPERATIONAL;

  /* Publish the heartbeat once/second */
  if(now - prev_hearbeat > 1000) {
    heartbeat_pub->publish(hb_msg);
    prev_hearbeat = now;
  }
  if((now - prev_battery_voltage) > (update_period_ms_inputvoltage))
  {
    float const analog = analogRead(ANALOG_PIN)*3.3*11.0/1023.0;
    Serial.print("Analog Pin: ");
    Serial.println(analog);
    Real32_1_0<ID_INPUT_VOLTAGE> uavcan_input_voltage;
    uavcan_input_voltage.data.value = analog;
    input_voltage_pub->publish(uavcan_input_voltage);
    prev_battery_voltage = now;
  }
  if((now - prev_internal_temperature) > (update_period_ms_internaltemperature))
  {
    float const temperature = analogReadTemp();
    Serial.print("Temperature: ");
    Serial.println(temperature);
    Real32_1_0<ID_INTERNAL_TEMPERATURE> uavcan_internal_temperature;
    uavcan_internal_temperature.data.value = temperature;
    internal_temperature_pub->publish(uavcan_internal_temperature);
    prev_internal_temperature = now;
  }

  /* Handling of inputs */
  if((now - prev_input0) > update_period_ms_input0)
  {
    Bit_1_0<ID_INPUT0> uavcan_input0;
    uavcan_input0.data.value = digitalRead(INPUT0_PIN);
    input_0_pub->publish(uavcan_input0);
    prev_input0 = now;
  }
  if((now - prev_input1) > update_period_ms_input1)
  {
    Bit_1_0<ID_INPUT1> uavcan_input1;
    uavcan_input1.data.value = digitalRead(INPUT1_PIN);
    input_1_pub->publish(uavcan_input1);
    prev_input1 = now;
  }
  if((now - prev_input2) > update_period_ms_input2)
  {
    Bit_1_0<ID_INPUT2> uavcan_input2;
    uavcan_input2.data.value = digitalRead(INPUT2_PIN);
    input_2_pub->publish(uavcan_input2);
    prev_input2 = now;
  }
  if((now - prev_input3) > update_period_ms_input3)
  {
    Bit_1_0<ID_INPUT3> uavcan_input3;
    uavcan_input3.data.value = digitalRead(INPUT3_PIN);
    input_3_pub->publish(uavcan_input3);
    prev_input3 = now;
  }
  if((now - prev_analog_input0) > update_period_ms_analoginput0)
  {
    Integer16_1_0<ID_ANALOG_INPUT0> uavcan_analog_input0;
    uavcan_analog_input0.data.value = analogRead(ANALOG_INPUT0_PIN);
    analog_input_0_pub->publish(uavcan_analog_input0);
    prev_analog_input0 = now;
  }
  if((now - prev_analog_input1) > update_period_ms_analoginput1)
  {
    Integer16_1_0<ID_ANALOG_INPUT1> uavcan_analog_input1;
    uavcan_analog_input1.data.value = analogRead(ANALOG_INPUT1_PIN);
    analog_input_1_pub->publish(uavcan_analog_input1);
    prev_analog_input1 = now;
  }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const & frame)
{
  node_hdl.onCanFrameReceived(frame);
}

void onLed1_Received(Bit_1_0<ID_LED1> const & msg)
{
  if(msg.data.value)
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

ExecuteCommand_1_0::Response<> onExecuteCommand_1_0_Request_Received(ExecuteCommand_1_0::Request<> const & req)
{
  ExecuteCommand_1_0::Response<> rsp;

  Serial.print("onExecuteCommand_1_1_Request_Received: ");
  Serial.println(req.data.command);

  if (req.data.command == uavcan_node_ExecuteCommand_Request_1_1_COMMAND_RESTART)
  {
    /* Send the response. */
    rsp.data.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_SUCCESS;
    watchdog_reboot(0,0,1000);
  }
  else if (req.data.command == uavcan_node_ExecuteCommand_Request_1_1_COMMAND_POWER_OFF)
  {
    /* Send the response. */
    rsp.data.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_SUCCESS;

    digitalWrite(LED2_PIN, HIGH);
    digitalWrite(LED3_PIN, HIGH);
    neo_pixel_ctrl.light_off();
  }
  else if (req.data.command == uavcan_node_ExecuteCommand_Request_1_1_COMMAND_BEGIN_SOFTWARE_UPDATE)
  {
    /* Send the response. */
    rsp.data.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_BAD_COMMAND;
    /* not implemented yet */
  }
  else if (req.data.command == uavcan_node_ExecuteCommand_Request_1_1_COMMAND_FACTORY_RESET)
  {
    /* set factory settings */
    update_period_ms_inputvoltage=3*1000;
    update_period_ms_internaltemperature=10*1000;
    update_period_ms_input0=500;
    update_period_ms_input1=500;
    update_period_ms_input2=500;
    update_period_ms_input3=500;
    update_period_ms_analoginput0=500;
    update_period_ms_analoginput1=500;
    update_period_ms_light=250;

    /* Send the response. */
    rsp.data.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_SUCCESS;
  }
  else if (req.data.command == uavcan_node_ExecuteCommand_Request_1_1_COMMAND_EMERGENCY_STOP)
  {
    /* Send the response. */
    rsp.data.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_BAD_COMMAND;
    /* not implemented yet */
  }
  else if (req.data.command == uavcan_node_ExecuteCommand_Request_1_1_COMMAND_STORE_PERSISTENT_STATES)
  {
    /* Send the response. */
    rsp.data.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_BAD_COMMAND;
    /* not implemented yet */
  }
  else
  {
    /* Send the response. */
    rsp.data.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_BAD_COMMAND;
  }

  return rsp;
}