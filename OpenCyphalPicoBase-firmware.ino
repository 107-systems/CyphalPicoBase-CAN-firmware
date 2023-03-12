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
void onLed1_Received(Bit_1_0 const & msg);
void onLightMode_Received(Integer8_1_0 const & msg);
/* Cyphal Service Request Callbacks */
ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(ExecuteCommand::Request_1_1 const &);

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

Publisher<Heartbeat_1_0> heartbeat_pub = node_hdl.create_publisher<Heartbeat_1_0>
  (Heartbeat_1_0::_traits_::FixedPortId, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Real32_1_0> input_voltage_pub = node_hdl.create_publisher<Real32_1_0>
  (ID_INPUT_VOLTAGE, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Real32_1_0> internal_temperature_pub = node_hdl.create_publisher<Real32_1_0>
  (ID_INTERNAL_TEMPERATURE, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Bit_1_0> input_0_pub = node_hdl.create_publisher<Bit_1_0>
  (ID_INPUT0, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Bit_1_0> input_1_pub = node_hdl.create_publisher<Bit_1_0>
  (ID_INPUT1, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Bit_1_0> input_2_pub = node_hdl.create_publisher<Bit_1_0>
  (ID_INPUT2, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Bit_1_0> input_3_pub = node_hdl.create_publisher<Bit_1_0>
  (ID_INPUT3, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Integer16_1_0> analog_input_0_pub = node_hdl.create_publisher<Integer16_1_0>
  (ID_ANALOG_INPUT0, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Integer16_1_0> analog_input_1_pub = node_hdl.create_publisher<Integer16_1_0>
  (ID_ANALOG_INPUT1, 1*1000*1000UL /* = 1 sec in usecs. */);

Subscription led_subscription =
  node_hdl.create_subscription<Bit_1_0>(ID_LED1, CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, onLed1_Received);

static Integer8_1_0 light_mode_msg;
Subscription light_mode_subscription =
  node_hdl.create_subscription<Integer8_1_0>(
    ID_LIGHT_MODE,
    CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
    [&light_mode_msg](Integer8_1_0 const & msg)
    {
      light_mode_msg = msg;
    });

ServiceServer execute_command_srv = node_hdl.create_service_server<ExecuteCommand::Request_1_1, ExecuteCommand::Response_1_1>(
  ExecuteCommand::Request_1_1::_traits_::FixedPortId,
  2*1000*1000UL,
  onExecuteCommand_1_1_Request_Received);


ServoControl servo_ctrl(SERVO0_PIN, SERVO1_PIN, node_hdl);
DigitalOutControl digital_out_ctrl(OUTPUT0_PIN, OUTPUT1_PIN, node_hdl);
NeoPixelControl neo_pixel_ctrl(NEOPIXEL_PIN, NEOPIXEL_NUM_PIXELS);

/* REGISTER ***************************************************************************/

static CanardNodeID node_id = DEFAULT_AUX_CONTROLLER_NODE_ID;

static uint16_t update_period_ms_inputvoltage        =  3*1000;
static uint16_t update_period_ms_internaltemperature = 10*1000;
static uint16_t update_period_ms_input0              =     500;
static uint16_t update_period_ms_input1              =     500;
static uint16_t update_period_ms_input2              =     500;
static uint16_t update_period_ms_input3              =     500;
static uint16_t update_period_ms_analoginput0        =     500;
static uint16_t update_period_ms_analoginput1        =     500;
static uint16_t update_period_ms_light               =     250;

#if __GNUC__ >= 11

Registry reg(node_hdl, micros);

const auto reg_rw_cyphal_node_id                           = reg.expose("cyphal.node.id", node_id);
const auto reg_ro_cyphal_node_description                  = reg.route ("cyphal.node.description",             {true}, []() { return "L3X-Z AUX_CONTROLLER"; });
const auto reg_ro_cyphal_pub_inputvoltage_id               = reg.route ("cyphal.pub.inputvoltage.id",          {true}, []() { return ID_INPUT_VOLTAGE; });
const auto reg_ro_cyphal_pub_inputvoltage_type             = reg.route ("cyphal.pub.inputvoltage.type",        {true}, []() { return "cyphal.primitive.scalar.Real32.1.0"; });
const auto reg_ro_cyphal_pub_internaltemperature_id        = reg.route ("cyphal.pub.internaltemperature.id",   {true}, []() { return ID_INTERNAL_TEMPERATURE; });
const auto reg_ro_cyphal_pub_internaltemperature_type      = reg.route ("cyphal.pub.internaltemperature.type", {true}, []() { return "cyphal.primitive.scalar.Real32.1.0" });
const auto reg_ro_cyphal_pub_input0_id                     = reg.route ("cyphal.pub.input0.id",                {true}, []() { return ID_INPUT0; });
const auto reg_ro_cyphal_pub_input0_type                   = reg.route ("cyphal.pub.input0.type",              {true}, []() { return "cyphal.primitive.scalar.Bit.1.0"; });
const auto reg_ro_cyphal_pub_input1_id                     = reg.route ("cyphal.pub.input1.id",                {true}, []() { return ID_INPUT1; });
const auto reg_ro_cyphal_pub_input1_type                   = reg.route ("cyphal.pub.input1.type",              {true}, []() { return "cyphal.primitive.scalar.Bit.1.0"; });
const auto reg_ro_cyphal_pub_input2_id                     = reg.route ("cyphal.pub.input2.id",                {true}, []() { return ID_INPUT2; });
const auto reg_ro_cyphal_pub_input2_type                   = reg.route ("cyphal.pub.input2.type",              {true}, []() { return "cyphal.primitive.scalar.Bit.1.0"; });
const auto reg_ro_cyphal_pub_input3_id                     = reg.route ("cyphal.pub.input3.id",                {true}, []() { return ID_INPUT3; });
const auto reg_ro_cyphal_pub_input3_type                   = reg.route ("cyphal.pub.input3.type",              {true}, []() { return "cyphal.primitive.scalar.Bit.1.0"; });
const auto reg_ro_cyphal_pub_analoginput0_id               = reg.route ("cyphal.pub.analoginput0.id",          {true}, []() { return ID_ANALOG_INPUT0; });
const auto reg_ro_cyphal_pub_analoginput0_type             = reg.route ("cyphal.pub.analoginput0.type",        {true}, []() { return "cyphal.primitive.scalar.Integer16.1.0"; });
const auto reg_ro_cyphal_pub_analoginput1_id               = reg.route ("cyphal.pub.analoginput1.id",          {true}, []() { return ID_ANALOG_INPUT1; });
const auto reg_ro_cyphal_pub_analoginput1_type             = reg.route ("cyphal.pub.analoginput1.type",        {true}, []() { return "cyphal.primitive.scalar.Integer16.1.0"; });
const auto reg_ro_cyphal_sub_led1_id                       = reg.route ("cyphal.sub.led1.id",                  {true}, []() { return ID_LED1; });
const auto reg_ro_cyphal_sub_led1_type                     = reg.route ("cyphal.sub.led1.type",                {true}, []() { return "cyphal.primitive.scalar.Bit.1.0"; });
const auto reg_ro_cyphal_sub_output0_id                    = reg.route ("cyphal.sub.output0.id",               {true}, []() { return ID_OUTPUT0; });
const auto reg_ro_cyphal_sub_output0_type                  = reg.route ("cyphal.sub.output0.type",             {true}, []() { return "cyphal.primitive.scalar.Bit.1.0"; });
const auto reg_ro_cyphal_sub_output1_id                    = reg.route ("cyphal.sub.output1.id",               {true}, []() { return ID_OUTPUT1; });
const auto reg_ro_cyphal_sub_output1_type                  = reg.route ("cyphal.sub.output1.type",             {true}, []() { return "cyphal.primitive.scalar.Bit.1.0"; });
const auto reg_ro_cyphal_sub_servo0_id                     = reg.route ("cyphal.sub.servo0.id",                {true}, []() { return ID_SERVO0; });
const auto reg_ro_cyphal_sub_servo0_type                   = reg.route ("cyphal.sub.servo0.type",              {true}, []() { return "cyphal.primitive.scalar.Integer16.1.0"; });
const auto reg_ro_cyphal_sub_servo1_id                     = reg.route ("cyphal.sub.servo1.id",                {true}, []() { return ID_SERVO1; });
const auto reg_ro_cyphal_sub_servo1_type                   = reg.route ("cyphal.sub.servo1.type",              {true}, []() { return "cyphal.primitive.scalar.Integer16.1.0"; });
const auto reg_ro_cyphal_sub_lightmode_id                  = reg.route ("cyphal.sub.lightmode.id",             {true}, []() { return ID_LIGHT_MODE; });
const auto reg_ro_cyphal_sub_lightmode_type                = reg.route ("cyphal.sub.lightmode.type",           {true}, []() { return "cyphal.primitive.scalar.Integer8.1.0"; });
const auto reg_rw_aux_update_period_ms_inputvoltage        = reg.expose("aux.update_period_ms.inputvoltage", update_period_ms_inputvoltage);
const auto reg_rw_aux_update_period_ms_internaltemperature = reg.expose("aux.update_period_ms.internaltemperature", update_period_ms_internaltemperature);
const auto reg_rw_aux_update_period_ms_input0              = reg.expose("aux.update_period_ms.input0", update_period_ms_input0);
const auto reg_rw_aux_update_period_ms_input1              = reg.expose("aux.update_period_ms.input1", update_period_ms_input1);
const auto reg_rw_aux_update_period_ms_input2              = reg.expose("aux.update_period_ms.input2", update_period_ms_input2);
const auto reg_rw_aux_update_period_ms_input3              = reg.expose("aux.update_period_ms.input3", update_period_ms_input3);
const auto reg_rw_aux_update_period_ms_analoginput0        = reg.expose("aux.update_period_ms.analoginput0", update_period_ms_analoginput0);
const auto reg_rw_aux_update_period_ms_analoginput1        = reg.expose("aux.update_period_ms.analoginput1", update_period_ms_analoginput1);
const auto reg_rw_aux_update_period_ms_light               = reg.expose("aux.update_period_ms.light", update_period_ms_light);

#endif /* __GNUC__ >= 11 */

/* NODE INFO **************************************************************************/

static NodeInfo node_info
(
  node_hdl,
  /* cyphal.node.Version.1.0 protocol_version */
  1, 0,
  /* cyphal.node.Version.1.0 hardware_version */
  1, 0,
  /* cyphal.node.Version.1.0 software_version */
  0, 1,
  /* saturated uint64 software_vcs_revision_id */
  0,
  /* saturated uint8[16] unique_id */
  OpenCyphalUniqueId(),
  /* saturated uint8[<=50] name */
  "107-systems.l3xz-fw_aux-controller"
);

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
  light_mode_msg.value = LIGHT_MODE_RUN_BLUE;

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

    if      (light_mode_msg.value == LIGHT_MODE_RED)
      neo_pixel_ctrl.light_red();
    else if (light_mode_msg.value == LIGHT_MODE_GREEN)
      neo_pixel_ctrl.light_green();
    else if (light_mode_msg.value == LIGHT_MODE_BLUE)
      neo_pixel_ctrl.light_blue();
    else if (light_mode_msg.value == LIGHT_MODE_WHITE)
      neo_pixel_ctrl.light_white();
    else if (light_mode_msg.value == LIGHT_MODE_AMBER)
      neo_pixel_ctrl.light_amber();
    else if (light_mode_msg.value == LIGHT_MODE_RUN_RED||light_mode_msg.value == LIGHT_MODE_RUN_GREEN||light_mode_msg.value == LIGHT_MODE_RUN_BLUE||light_mode_msg.value == LIGHT_MODE_RUN_WHITE||light_mode_msg.value == LIGHT_MODE_RUN_AMBER)
    {
      if (light_mode_msg.value == LIGHT_MODE_RUN_RED)
      {
        neo_pixel_ctrl.pixels().setPixelColor(running_light_counter,       neo_pixel_ctrl.pixels().Color(55, 0, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+7)%8, neo_pixel_ctrl.pixels().Color(27, 0, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+6)%8, neo_pixel_ctrl.pixels().Color(14, 0, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+5)%8, neo_pixel_ctrl.pixels().Color(7, 0, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+4)%8, neo_pixel_ctrl.pixels().Color(0, 0, 0));
        neo_pixel_ctrl.pixels().show();
      }
      else if (light_mode_msg.value == LIGHT_MODE_RUN_GREEN)
      {
        neo_pixel_ctrl.pixels().setPixelColor(running_light_counter,       neo_pixel_ctrl.pixels().Color(0, 55, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+7)%8, neo_pixel_ctrl.pixels().Color(0, 27, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+6)%8, neo_pixel_ctrl.pixels().Color(0, 14, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+5)%8, neo_pixel_ctrl.pixels().Color(0, 7, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+4)%8, neo_pixel_ctrl.pixels().Color(0, 0, 0));
        neo_pixel_ctrl.pixels().show();
      }
      else if (light_mode_msg.value == LIGHT_MODE_RUN_BLUE)
      {
        neo_pixel_ctrl.pixels().setPixelColor(running_light_counter,       neo_pixel_ctrl.pixels().Color(0, 0, 55));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+7)%8, neo_pixel_ctrl.pixels().Color(0, 0, 27));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+6)%8, neo_pixel_ctrl.pixels().Color(0, 0, 14));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+5)%8, neo_pixel_ctrl.pixels().Color(0, 0, 7));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+4)%8, neo_pixel_ctrl.pixels().Color(0, 0, 0));
        neo_pixel_ctrl.pixels().show();
      }
      else if (light_mode_msg.value == LIGHT_MODE_RUN_WHITE)
      {
        neo_pixel_ctrl.pixels().setPixelColor(running_light_counter,       neo_pixel_ctrl.pixels().Color(55, 55, 55));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+7)%8, neo_pixel_ctrl.pixels().Color(27, 27, 27));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+6)%8, neo_pixel_ctrl.pixels().Color(14, 14, 14));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+5)%8, neo_pixel_ctrl.pixels().Color(7, 7, 7));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+4)%8, neo_pixel_ctrl.pixels().Color(0, 0, 0));
        neo_pixel_ctrl.pixels().show();
      }
      else if (light_mode_msg.value == LIGHT_MODE_RUN_AMBER)
      {
        neo_pixel_ctrl.pixels().setPixelColor(running_light_counter,       neo_pixel_ctrl.pixels().Color(55, 40, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+7)%8, neo_pixel_ctrl.pixels().Color(27, 20, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+6)%8, neo_pixel_ctrl.pixels().Color(14, 10, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+5)%8, neo_pixel_ctrl.pixels().Color(7, 5, 0));
        neo_pixel_ctrl.pixels().setPixelColor((running_light_counter+4)%8, neo_pixel_ctrl.pixels().Color(0, 0, 0));
        neo_pixel_ctrl.pixels().show();
      }
    }
    else if (is_light_on&&(light_mode_msg.value == LIGHT_MODE_BLINK_RED||light_mode_msg.value == LIGHT_MODE_BLINK_GREEN||light_mode_msg.value == LIGHT_MODE_BLINK_BLUE||light_mode_msg.value == LIGHT_MODE_BLINK_WHITE||light_mode_msg.value == LIGHT_MODE_BLINK_AMBER))
    {
      if (light_mode_msg.value == LIGHT_MODE_BLINK_GREEN)
        neo_pixel_ctrl.light_green();
      else if (light_mode_msg.value == LIGHT_MODE_BLINK_BLUE)
        neo_pixel_ctrl.light_blue();
      else if (light_mode_msg.value == LIGHT_MODE_BLINK_WHITE)
        neo_pixel_ctrl.light_white();
      else if (light_mode_msg.value == LIGHT_MODE_BLINK_AMBER)
        neo_pixel_ctrl.light_amber();
      else
        neo_pixel_ctrl.light_red();
    }
    else
      neo_pixel_ctrl.light_off();

    prev_led = now;
  }

  /* Publish the heartbeat once/second */
  if(now - prev_hearbeat > 1000)
  {
    Heartbeat_1_0 msg;

    msg.uptime = millis() / 1000;
    msg.health.value = uavcan::node::Health_1_0::NOMINAL;
    msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
    msg.vendor_specific_status_code = 0;

    heartbeat_pub->publish(msg);

    prev_hearbeat = now;
  }
  if((now - prev_battery_voltage) > (update_period_ms_inputvoltage))
  {
    float const analog = analogRead(ANALOG_PIN)*3.3*11.0/1023.0;
    Serial.print("Analog Pin: ");
    Serial.println(analog);

    Real32_1_0 uavcan_input_voltage;
    uavcan_input_voltage.value = analog;
    input_voltage_pub->publish(uavcan_input_voltage);

    prev_battery_voltage = now;
  }
  if((now - prev_internal_temperature) > (update_period_ms_internaltemperature))
  {
    float const temperature = analogReadTemp();
    Serial.print("Temperature: ");
    Serial.println(temperature);

    Real32_1_0 uavcan_internal_temperature;
    uavcan_internal_temperature.value = temperature;
    internal_temperature_pub->publish(uavcan_internal_temperature);

    prev_internal_temperature = now;
  }

  /* Handling of inputs */
  if((now - prev_input0) > update_period_ms_input0)
  {
    Bit_1_0 uavcan_input0;
    uavcan_input0.value = digitalRead(INPUT0_PIN);
    input_0_pub->publish(uavcan_input0);

    prev_input0 = now;
  }
  if((now - prev_input1) > update_period_ms_input1)
  {
    Bit_1_0 uavcan_input1;
    uavcan_input1.value = digitalRead(INPUT1_PIN);
    input_1_pub->publish(uavcan_input1);

    prev_input1 = now;
  }
  if((now - prev_input2) > update_period_ms_input2)
  {
    Bit_1_0 uavcan_input2;
    uavcan_input2.value = digitalRead(INPUT2_PIN);
    input_2_pub->publish(uavcan_input2);

    prev_input2 = now;
  }
  if((now - prev_input3) > update_period_ms_input3)
  {
    Bit_1_0 uavcan_input3;
    uavcan_input3.value = digitalRead(INPUT3_PIN);
    input_3_pub->publish(uavcan_input3);

    prev_input3 = now;
  }
  if((now - prev_analog_input0) > update_period_ms_analoginput0)
  {
    Integer16_1_0 uavcan_analog_input0;
    uavcan_analog_input0.value = analogRead(ANALOG_INPUT0_PIN);
    analog_input_0_pub->publish(uavcan_analog_input0);

    prev_analog_input0 = now;
  }
  if((now - prev_analog_input1) > update_period_ms_analoginput1)
  {
    Integer16_1_0 uavcan_analog_input1;
    uavcan_analog_input1.value = analogRead(ANALOG_INPUT1_PIN);
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

void onLed1_Received(Bit_1_0 const & msg)
{
  if(msg.value)
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

ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(ExecuteCommand::Request_1_1 const & req)
{
  ExecuteCommand::Response_1_1 rsp;

  Serial.print("onExecuteCommand_1_1_Request_Received: ");
  Serial.println(req.command);

  if (req.command == ExecuteCommand::Request_1_1::COMMAND_RESTART)
  {
    /* Send the response. */
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
    watchdog_reboot(0,0,1000);
  }
  else if (req.command == ExecuteCommand::Request_1_1::COMMAND_POWER_OFF)
  {
    /* Send the response. */
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;

    digitalWrite(LED2_PIN, HIGH);
    digitalWrite(LED3_PIN, HIGH);
    neo_pixel_ctrl.light_off();
  }
  else if (req.command == ExecuteCommand::Request_1_1::COMMAND_BEGIN_SOFTWARE_UPDATE)
  {
    /* Send the response. */
    rsp.status = ExecuteCommand::Response_1_1::STATUS_BAD_COMMAND;
    /* not implemented yet */
  }
  else if (req.command == ExecuteCommand::Request_1_1::COMMAND_FACTORY_RESET)
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
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else if (req.command == ExecuteCommand::Request_1_1::COMMAND_EMERGENCY_STOP)
  {
    /* Send the response. */
    rsp.status = ExecuteCommand::Response_1_1::STATUS_BAD_COMMAND;
    /* not implemented yet */
  }
  else if (req.command == ExecuteCommand::Request_1_1::COMMAND_STORE_PERSISTENT_STATES)
  {
    /* Send the response. */
    rsp.status = ExecuteCommand::Response_1_1::STATUS_BAD_COMMAND;
    /* not implemented yet */
  }
  else
  {
    /* Send the response. */
    rsp.status = ExecuteCommand::Response_1_1::STATUS_BAD_COMMAND;
  }

  return rsp;
}
