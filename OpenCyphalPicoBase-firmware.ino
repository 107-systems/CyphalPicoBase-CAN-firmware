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
#include <107-Arduino-Cyphal-Support.h>

#include <107-Arduino-MCP2515.h>
#include <107-Arduino-littlefs.h>
#include <107-Arduino-24LCxx.hpp>

#define DBG_ENABLE_ERROR
#define DBG_ENABLE_WARNING
#define DBG_ENABLE_INFO
#define DBG_ENABLE_DEBUG
//#define DBG_ENABLE_VERBOSE
#include <107-Arduino-Debug.hpp>

#undef max
#undef min
#include <algorithm>

#include "src/ServoControl.h"
#include "src/NeoPixelControl.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static uint8_t const EEPROM_I2C_DEV_ADDR = 0x50;

static int const INPUT0_PIN        =  6;
static int const INPUT1_PIN        =  7;
static int const INPUT2_PIN        =  8;
static int const INPUT3_PIN        =  9;
static int const OUTPUT0_PIN       = 10;
static int const OUTPUT1_PIN       = 11;
static int const NEOPIXEL_PIN      = 13; /* Raspberry Pi Pico */
static int const SERVO0_PIN        = 14;
static int const SERVO1_PIN        = 15;
static int const MCP2515_CS_PIN  = 17;
static int const MCP2515_INT_PIN = 20;
static int const LED2_PIN        = 21; /* GP21 */
static int const LED3_PIN        = 22; /* GP22 */
static int const ANALOG_PIN        = 26;
static int const ANALOG_INPUT0_PIN = 27;
static int const ANALOG_INPUT1_PIN = 28;

static int const NEOPIXEL_NUM_PIXELS = 8; /* Popular NeoPixel ring size */

static SPISettings const MCP2515x_SPI_SETTING{10*1000*1000UL, MSBFIRST, SPI_MODE0};

static uint16_t const UPDATE_PERIOD_HEARTBEAT_ms = 1000;

static uint32_t const WATCHDOG_DELAY_ms = 1000;

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
void onLed1_Received(uavcan::primitive::scalar::Bit_1_0 const & msg);
void onLightMode_Received(uavcan::primitive::scalar::Integer8_1_0 const & msg);
/* Cyphal Service Request Callbacks */
ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(ExecuteCommand::Request_1_1 const &);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

DEBUG_INSTANCE(80, Serial);

ArduinoMCP2515 mcp2515([]() { digitalWrite(MCP2515_CS_PIN, LOW); },
                       []() { digitalWrite(MCP2515_CS_PIN, HIGH); },
                       [](uint8_t const d) { return SPI.transfer(d); },
                       micros,
                       onReceiveBufferFull,
                       nullptr,
                       [](MCP2515::EFLG const err_flag) { DBG_ERROR("MCP2515::OnError, error code = \"%s\"", MCP2515::toStr(err_flag)); },
                       [](MCP2515::EFLG const err_flag) { DBG_ERROR("MCP2515::OnWarning, warning code = \"%s\"", MCP2515::toStr(err_flag)); });

Node::Heap<Node::DEFAULT_O1HEAP_SIZE> node_heap;
Node node_hdl(node_heap.data(), node_heap.size(), micros, [] (CanardFrame const & frame) { return mcp2515.transmit(frame); });

Publisher<Heartbeat_1_0> heartbeat_pub = node_hdl.create_publisher<Heartbeat_1_0>(1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<uavcan::primitive::scalar::Real32_1_0> input_voltage_pub;
Publisher<uavcan::primitive::scalar::Real32_1_0> internal_temperature_pub;
Publisher<uavcan::primitive::scalar::Bit_1_0> input_0_pub;
Publisher<uavcan::primitive::scalar::Bit_1_0> input_1_pub;
Publisher<uavcan::primitive::scalar::Bit_1_0> input_2_pub;
Publisher<uavcan::primitive::scalar::Bit_1_0> input_3_pub;
Publisher<uavcan::primitive::scalar::Integer16_1_0> analog_input_0_pub;
Publisher<uavcan::primitive::scalar::Integer16_1_0> analog_input_1_pub;

Subscription led_subscription;

uavcan::primitive::scalar::Integer8_1_0 light_mode_msg{LIGHT_MODE_WHITE};
Subscription light_mode_subscription;

Subscription output_0_subscription, output_1_subscription;

ServiceServer execute_command_srv = node_hdl.create_service_server<ExecuteCommand::Request_1_1, ExecuteCommand::Response_1_1>(2*1000*1000UL, onExecuteCommand_1_1_Request_Received);

ServoControl servo_ctrl(SERVO0_PIN, SERVO1_PIN, node_hdl);
NeoPixelControl neo_pixel_ctrl(NEOPIXEL_PIN, NEOPIXEL_NUM_PIXELS);

/* LITTLEFS/EEPROM ********************************************************************/

static EEPROM_24LCxx eeprom(EEPROM_24LCxx_Type::LC64,
                            EEPROM_I2C_DEV_ADDR,
                            [](size_t const dev_addr) { Wire.beginTransmission(dev_addr); },
                            [](uint8_t const data) { Wire.write(data); },
                            []() { return Wire.endTransmission(); },
                            [](uint8_t const dev_addr, size_t const len) -> size_t { return Wire.requestFrom(dev_addr, len); },
                            []() { return Wire.available(); },
                            []() { return Wire.read(); });

static littlefs::FilesystemConfig filesystem_config
  (
    +[](const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) -> int
    {
      eeprom.read_page((block * c->block_size) + off, (uint8_t *)buffer, size);
      return LFS_ERR_OK;
    },
    +[](const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) -> int
    {
      eeprom.write_page((block * c->block_size) + off, (uint8_t const *)buffer, size);
      return LFS_ERR_OK;
    },
    +[](const struct lfs_config *c, lfs_block_t block) -> int
    {
      for(size_t off = 0; off < c->block_size; off += eeprom.page_size())
        eeprom.fill_page((block * c->block_size) + off, 0xFF);
      return LFS_ERR_OK;
    },
    +[](const struct lfs_config *c) -> int
    {
      return LFS_ERR_OK;
    },
    eeprom.page_size(),
    eeprom.page_size(),
    (eeprom.page_size() * 4), /* littlefs demands (erase) block size to exceed read/prog size. */
    eeprom.device_size() / (eeprom.page_size() * 4),
    500,
    eeprom.page_size(),
    eeprom.page_size()
  );
static littlefs::Filesystem filesystem(filesystem_config);

#if __GNUC__ >= 11
cyphal::support::platform::storage::littlefs::KeyValueStorage kv_storage(filesystem);
#endif /* __GNUC__ >= 11 */

/* REGISTER ***************************************************************************/

static uint16_t     node_id                      = std::numeric_limits<uint16_t>::max();
static CanardPortID port_id_input_voltage        = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_led1                 = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_internal_temperature = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_input0               = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_input1               = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_input2               = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_input3               = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_output0              = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_output1              = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_servo0               = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_servo1               = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_analog_input0        = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_analog_input1        = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_light_mode           = std::numeric_limits<CanardPortID>::max();

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

const auto node_registry = node_hdl.create_registry();

const auto reg_rw_cyphal_node_id                           = node_registry->expose("cyphal.node.id",                           {true}, node_id);
const auto reg_ro_cyphal_node_description                  = node_registry->route ("cyphal.node.description",                  {true}, []() { return "L3X-Z AUX_CONTROLLER"; });
const auto reg_rw_cyphal_pub_inputvoltage_id               = node_registry->expose("cyphal.pub.inputvoltage.id",               {true}, port_id_input_voltage);
const auto reg_ro_cyphal_pub_inputvoltage_type             = node_registry->route ("cyphal.pub.inputvoltage.type",             {true}, []() { return "cyphal.primitive.scalar.Real32.1.0"; });
const auto reg_rw_cyphal_pub_internaltemperature_id        = node_registry->expose("cyphal.pub.internaltemperature.id",        {true}, port_id_internal_temperature);
const auto reg_ro_cyphal_pub_internaltemperature_type      = node_registry->route ("cyphal.pub.internaltemperature.type",      {true}, []() { return "cyphal.primitive.scalar.Real32.1.0"; });
const auto reg_rw_cyphal_pub_input0_id                     = node_registry->expose("cyphal.pub.input0.id",                     {true}, port_id_input0);
const auto reg_ro_cyphal_pub_input0_type                   = node_registry->route ("cyphal.pub.input0.type",                   {true}, []() { return "cyphal.primitive.scalar.Bit.1.0"; });
const auto reg_rw_cyphal_pub_input1_id                     = node_registry->expose("cyphal.pub.input1.id",                     {true}, port_id_input1);
const auto reg_ro_cyphal_pub_input1_type                   = node_registry->route ("cyphal.pub.input1.type",                   {true}, []() { return "cyphal.primitive.scalar.Bit.1.0"; });
const auto reg_rw_cyphal_pub_input2_id                     = node_registry->expose("cyphal.pub.input2.id",                     {true}, port_id_input2);
const auto reg_ro_cyphal_pub_input2_type                   = node_registry->route ("cyphal.pub.input2.type",                   {true}, []() { return "cyphal.primitive.scalar.Bit.1.0"; });
const auto reg_rw_cyphal_pub_input3_id                     = node_registry->expose("cyphal.pub.input3.id",                     {true}, port_id_input3);
const auto reg_ro_cyphal_pub_input3_type                   = node_registry->route ("cyphal.pub.input3.type",                   {true}, []() { return "cyphal.primitive.scalar.Bit.1.0"; });
const auto reg_rw_cyphal_pub_analoginput0_id               = node_registry->expose("cyphal.pub.analoginput0.id",               {true}, port_id_analog_input0);
const auto reg_ro_cyphal_pub_analoginput0_type             = node_registry->route ("cyphal.pub.analoginput0.type",             {true}, []() { return "cyphal.primitive.scalar.Integer16.1.0"; });
const auto reg_rw_cyphal_pub_analoginput1_id               = node_registry->expose("cyphal.pub.analoginput1.id",               {true}, port_id_analog_input1);
const auto reg_ro_cyphal_pub_analoginput1_type             = node_registry->route ("cyphal.pub.analoginput1.type",             {true}, []() { return "cyphal.primitive.scalar.Integer16.1.0"; });
const auto reg_rw_cyphal_sub_led1_id                       = node_registry->expose("cyphal.sub.led1.id",                       {true}, port_id_led1);
const auto reg_ro_cyphal_sub_led1_type                     = node_registry->route ("cyphal.sub.led1.type",                     {true}, []() { return "cyphal.primitive.scalar.Bit.1.0"; });
const auto reg_rw_cyphal_sub_output0_id                    = node_registry->expose("cyphal.sub.output0.id",                    {true}, port_id_output0);
const auto reg_ro_cyphal_sub_output0_type                  = node_registry->route ("cyphal.sub.output0.type",                  {true}, []() { return "cyphal.primitive.scalar.Bit.1.0"; });
const auto reg_rw_cyphal_sub_output1_id                    = node_registry->expose("cyphal.sub.output1.id",                    {true}, port_id_output1);
const auto reg_ro_cyphal_sub_output1_type                  = node_registry->route ("cyphal.sub.output1.type",                  {true}, []() { return "cyphal.primitive.scalar.Bit.1.0"; });
const auto reg_rw_cyphal_sub_servo0_id                     = node_registry->expose("cyphal.sub.servo0.id",                     {true}, port_id_servo0);
const auto reg_ro_cyphal_sub_servo0_type                   = node_registry->route ("cyphal.sub.servo0.type",                   {true}, []() { return "cyphal.primitive.scalar.Integer16.1.0"; });
const auto reg_rw_cyphal_sub_servo1_id                     = node_registry->expose("cyphal.sub.servo1.id",                     {true}, port_id_servo1);
const auto reg_ro_cyphal_sub_servo1_type                   = node_registry->route ("cyphal.sub.servo1.type",                   {true}, []() { return "cyphal.primitive.scalar.Integer16.1.0"; });
const auto reg_rw_cyphal_sub_lightmode_id                  = node_registry->expose("cyphal.sub.lightmode.id",                  {true}, port_id_light_mode);
const auto reg_ro_cyphal_sub_lightmode_type                = node_registry->route ("cyphal.sub.lightmode.type",                {true}, []() { return "cyphal.primitive.scalar.Integer8.1.0"; });
const auto reg_rw_aux_update_period_ms_inputvoltage        = node_registry->expose("aux.update_period_ms.inputvoltage",        {}, update_period_ms_inputvoltage);
const auto reg_rw_aux_update_period_ms_internaltemperature = node_registry->expose("aux.update_period_ms.internaltemperature", {}, update_period_ms_internaltemperature);
const auto reg_rw_aux_update_period_ms_input0              = node_registry->expose("aux.update_period_ms.input0",              {}, update_period_ms_input0);
const auto reg_rw_aux_update_period_ms_input1              = node_registry->expose("aux.update_period_ms.input1",              {}, update_period_ms_input1);
const auto reg_rw_aux_update_period_ms_input2              = node_registry->expose("aux.update_period_ms.input2",              {}, update_period_ms_input2);
const auto reg_rw_aux_update_period_ms_input3              = node_registry->expose("aux.update_period_ms.input3",              {}, update_period_ms_input3);
const auto reg_rw_aux_update_period_ms_analoginput0        = node_registry->expose("aux.update_period_ms.analoginput0",        {}, update_period_ms_analoginput0);
const auto reg_rw_aux_update_period_ms_analoginput1        = node_registry->expose("aux.update_period_ms.analoginput1",        {}, update_period_ms_analoginput1);
const auto reg_rw_aux_update_period_ms_light               = node_registry->expose("aux.update_period_ms.light",               {}, update_period_ms_light);

#endif /* __GNUC__ >= 11 */

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  // while(!Serial) { } /* only for debug */
  delay(1000);

  Debug.prettyPrintOn(); /* Enable pretty printing on a shell. */

  /* LITTLEFS/EEPROM ********************************************************************/
  Wire.begin();
  Wire.setClock(400*1000UL); /* Set fast mode. */

  if (!eeprom.isConnected()) {
    DBG_ERROR("Connecting to EEPROM failed.");
    return;
  }
  Serial.println(eeprom);

  if (auto const err_mount = filesystem.mount(); err_mount.has_value()) {
    DBG_ERROR("Mounting failed with error code %d", static_cast<int>(err_mount.value()));
    (void)filesystem.format();
  }

  if (auto const err_mount = filesystem.mount(); err_mount.has_value()) {
    DBG_ERROR("Mounting failed again with error code %d", static_cast<int>(err_mount.value()));
    return;
  }

#if __GNUC__ >= 11
  auto const rc_load = cyphal::support::load(kv_storage, *node_registry);
  if (rc_load.has_value()) {
    DBG_ERROR("cyphal::support::load failed with %d", static_cast<int>(rc_load.value()));
    return;
  }
#endif /* __GNUC__ >= 11 */

  (void)filesystem.unmount();

  /* If the node ID contained in the register points to an undefined
   * node ID, assign node ID 0 to this node.
   */
  if (node_id > CANARD_NODE_ID_MAX)
    node_id = 0;
  node_hdl.setNodeId(static_cast<CanardNodeID>(node_id));

  if (port_id_input_voltage != std::numeric_limits<CanardPortID>::max())
    input_voltage_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Real32_1_0>(port_id_input_voltage, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_led1 != std::numeric_limits<CanardPortID>::max())
    led_subscription = node_hdl.create_subscription<uavcan::primitive::scalar::Bit_1_0>(port_id_led1, onLed1_Received);
  if (port_id_internal_temperature != std::numeric_limits<CanardPortID>::max())
    internal_temperature_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Real32_1_0>(port_id_internal_temperature, 1*1000*1000UL /* = 1 sec in usecs. */);

  if (port_id_output0 != std::numeric_limits<CanardPortID>::max())
    output_0_subscription = node_hdl.create_subscription<uavcan::primitive::scalar::Bit_1_0>(
      port_id_output0,
      1*1000*1000UL /* = 1 sec in usecs. */,
      [this](uavcan::primitive::scalar::Bit_1_0 const & msg)
      {
        if(msg.value)
          digitalWrite(OUTPUT0_PIN, HIGH);
        else
          digitalWrite(OUTPUT0_PIN, LOW);
      });

  if (port_id_output1 != std::numeric_limits<CanardPortID>::max())
    output_1_subscription = node_hdl.create_subscription<uavcan::primitive::scalar::Bit_1_0>(
      port_id_output1,
      1*1000*1000UL /* = 1 sec in usecs. */,
      [this](uavcan::primitive::scalar::Bit_1_0 const & msg)
      {
        if(msg.value)
          digitalWrite(OUTPUT1_PIN, HIGH);
        else
          digitalWrite(OUTPUT1_PIN, LOW);
      });

  if (port_id_input0 != std::numeric_limits<CanardPortID>::max())
    input_0_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Bit_1_0>(port_id_input0, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_input1 != std::numeric_limits<CanardPortID>::max())
    input_1_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Bit_1_0>(port_id_input1, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_input2 != std::numeric_limits<CanardPortID>::max())
    input_2_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Bit_1_0>(port_id_input2, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_input3 != std::numeric_limits<CanardPortID>::max())
    input_3_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Bit_1_0>(port_id_input3, 1*1000*1000UL /* = 1 sec in usecs. */);

  if (port_id_analog_input0 != std::numeric_limits<CanardPortID>::max())
    analog_input_0_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Integer16_1_0>(port_id_analog_input0, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_analog_input1 != std::numeric_limits<CanardPortID>::max())
    analog_input_1_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Integer16_1_0>(port_id_analog_input1, 1*1000*1000UL /* = 1 sec in usecs. */);

  if (port_id_light_mode != std::numeric_limits<CanardPortID>::max())
    light_mode_subscription = node_hdl.create_subscription<uavcan::primitive::scalar::Integer8_1_0>(port_id_light_mode, [](uavcan::primitive::scalar::Integer8_1_0 const & msg) { light_mode_msg = msg; });

  /* NODE INFO **************************************************************************/
  static const auto node_info = node_hdl.create_node_info
  (
    /* cyphal.node.Version.1.0 protocol_version */
    1, 0,
    /* cyphal.node.Version.1.0 hardware_version */
    1, 0,
    /* cyphal.node.Version.1.0 software_version */
    0, 1,
    /* saturated uint64 software_vcs_revision_id */
#ifdef CYPHAL_NODE_INFO_GIT_VERSION
    CYPHAL_NODE_INFO_GIT_VERSION,
#else
    0,
#endif
    /* saturated uint8[16] unique_id */
    cyphal::support::UniqueId::instance().value(),
    /* saturated uint8[<=50] name */
    "107-systems.open-cyphal-pico-base"
  );

  /* Setup LED pins and initialize */
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(INPUT0_PIN, INPUT_PULLUP);
  pinMode(INPUT1_PIN, INPUT_PULLUP);
  pinMode(INPUT2_PIN, INPUT_PULLUP);
  pinMode(INPUT3_PIN, INPUT_PULLUP);

  /* Setup OUT0/OUT1. */
  pinMode(OUT0_PIN, OUTPUT);
  pinMode(OUT1_PIN, OUTPUT);
  digitalWrite(OUT0_PIN, LOW);
  digitalWrite(OUT1_PIN, LOW);

  servo_ctrl.begin();
  neo_pixel_ctrl.begin();

  /* Setup SPI access */
  SPI.begin();
  SPI.beginTransaction(MCP2515x_SPI_SETTING);
  pinMode(MCP2515_INT_PIN, INPUT_PULLUP);
  pinMode(MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MCP2515_CS_PIN, HIGH);

  /* Initialize MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);

  CanardFilter const CAN_FILTER_SERVICES = canardMakeFilterForServices(node_id);
  DBG_INFO("CAN Filter #1\n\r\tExt. Mask : %8X\n\r\tExt. ID   : %8X",
           CAN_FILTER_SERVICES.extended_mask,
           CAN_FILTER_SERVICES.extended_can_id);

  /* Only pass service requests/responses for this node ID through to receive buffer #0. */
  uint32_t const RXMB0_MASK = CAN_FILTER_SERVICES.extended_mask;
  size_t const RXMB0_FILTER_SIZE = 2;
  uint32_t const RXMB0_FILTER[RXMB0_FILTER_SIZE] =
    {
      MCP2515::CAN_EFF_BITMASK | CAN_FILTER_SERVICES.extended_can_id,
      MCP2515::CAN_EFF_BITMASK | 0
    };
  mcp2515.enableFilter(MCP2515::RxB::RxB0, RXMB0_MASK, RXMB0_FILTER, RXMB0_FILTER_SIZE);

  /* Only pass messages with ID 0 to receive buffer #1 (filtering out most). */
  uint32_t const RXMB1_MASK = 0x01FFFFFF;
  size_t const RXMB1_FILTER_SIZE = 4;
  uint32_t const RXMB1_FILTER[RXMB1_FILTER_SIZE] =
  {
    MCP2515::CAN_EFF_BITMASK | 0,
    MCP2515::CAN_EFF_BITMASK | 0,
    MCP2515::CAN_EFF_BITMASK | 0,
    MCP2515::CAN_EFF_BITMASK | 0
  };
  mcp2515.enableFilter(MCP2515::RxB::RxB1, RXMB1_MASK, RXMB1_FILTER, RXMB1_FILTER_SIZE);

  /* Leave configuration and enable MCP2515. */
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

  /* Enable watchdog. */
  rp2040.wdt_begin(WATCHDOG_DELAY_ms);
  rp2040.wdt_reset();

  DBG_INFO("Init complete.");
}

void loop()
{
  /* Deal with all pending events of the MCP2515 -
   * signaled by the INT pin being driven LOW.
   */
  while(digitalRead(MCP2515_INT_PIN) == LOW)
    mcp2515.onExternalEventHandler();

  /* Process all pending Cyphal actions.
   */
  node_hdl.spinSome();

  /* Publish all the gathered data, although at various
   * different intervals.
   */
  static unsigned long prev_led = 0;
  static unsigned long prev_led_toggle = 0;
  static unsigned long prev_heartbeat = 0;
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
  if((now - prev_heartbeat) > UPDATE_PERIOD_HEARTBEAT_ms)
  {
    prev_heartbeat = now;

    Heartbeat_1_0 msg;

    msg.uptime = millis() / 1000;
    msg.health.value = uavcan::node::Health_1_0::NOMINAL;
    msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
    msg.vendor_specific_status_code = 0;

    heartbeat_pub->publish(msg);

    digitalWrite(LED2_PIN, !digitalRead(LED2_PIN));
  }
  if((now - prev_battery_voltage) > (update_period_ms_inputvoltage))
  {
    float const analog = analogRead(ANALOG_PIN)*3.3*11.0/1023.0;
    Serial.print("Analog Pin: ");
    Serial.println(analog);

    uavcan::primitive::scalar::Real32_1_0 uavcan_input_voltage;
    uavcan_input_voltage.value = analog;
    if(input_voltage_pub) input_voltage_pub->publish(uavcan_input_voltage);

    prev_battery_voltage = now;
  }
  if((now - prev_internal_temperature) > (update_period_ms_internaltemperature))
  {
    float const temperature = analogReadTemp();
    Serial.print("Temperature: ");
    Serial.println(temperature);

    uavcan::primitive::scalar::Real32_1_0 uavcan_internal_temperature;
    uavcan_internal_temperature.value = temperature;
    if(internal_temperature_pub) internal_temperature_pub->publish(uavcan_internal_temperature);

    prev_internal_temperature = now;
  }

  /* Handling of inputs */
  if((now - prev_input0) > update_period_ms_input0)
  {
    uavcan::primitive::scalar::Bit_1_0 uavcan_input0;
    uavcan_input0.value = digitalRead(INPUT0_PIN);
    if(input_0_pub) input_0_pub->publish(uavcan_input0);

    prev_input0 = now;
  }
  if((now - prev_input1) > update_period_ms_input1)
  {
    uavcan::primitive::scalar::Bit_1_0 uavcan_input1;
    uavcan_input1.value = digitalRead(INPUT1_PIN);
    if(input_1_pub) input_1_pub->publish(uavcan_input1);

    prev_input1 = now;
  }
  if((now - prev_input2) > update_period_ms_input2)
  {
    uavcan::primitive::scalar::Bit_1_0 uavcan_input2;
    uavcan_input2.value = digitalRead(INPUT2_PIN);
    if(input_2_pub) input_2_pub->publish(uavcan_input2);

    prev_input2 = now;
  }
  if((now - prev_input3) > update_period_ms_input3)
  {
    uavcan::primitive::scalar::Bit_1_0 uavcan_input3;
    uavcan_input3.value = digitalRead(INPUT3_PIN);
    if(input_3_pub) input_3_pub->publish(uavcan_input3);

    prev_input3 = now;
  }
  if((now - prev_analog_input0) > update_period_ms_analoginput0)
  {
    uavcan::primitive::scalar::Integer16_1_0 uavcan_analog_input0;
    uavcan_analog_input0.value = analogRead(ANALOG_INPUT0_PIN);
    if(analog_input_0_pub) analog_input_0_pub->publish(uavcan_analog_input0);

    prev_analog_input0 = now;
  }
  if((now - prev_analog_input1) > update_period_ms_analoginput1)
  {
    uavcan::primitive::scalar::Integer16_1_0 uavcan_analog_input1;
    uavcan_analog_input1.value = analogRead(ANALOG_INPUT1_PIN);
    if(analog_input_1_pub) analog_input_1_pub->publish(uavcan_analog_input1);

    prev_analog_input1 = now;
  }

  /* Feed the watchdog only if not an async reset is
   * pending because we want to restart via yakut.
   */
  if (!cyphal::support::platform::is_async_reset_pending())
    rp2040.wdt_reset();
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const & frame)
{
  digitalWrite(LED3_PIN, !digitalRead(LED3_PIN));
  node_hdl.onCanFrameReceived(frame);
}

void onLed1_Received(uavcan::primitive::scalar::Bit_1_0 const & msg)
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

  if (req.command == ExecuteCommand::Request_1_1::COMMAND_RESTART)
  {
    if (auto const opt_err = cyphal::support::platform::reset_async(std::chrono::milliseconds(1000)); opt_err.has_value())
    {
      DBG_ERROR("reset_async failed with error code %d", static_cast<int>(opt_err.value()));
      rsp.status = ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else if (req.command == ExecuteCommand::Request_1_1::COMMAND_STORE_PERSISTENT_STATES)
  {
    if (auto const err_mount = filesystem.mount(); err_mount.has_value())
    {
      DBG_ERROR("Mounting failed with error code %d", static_cast<int>(err_mount.value()));
      rsp.status = ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
    /* Feed the watchdog. */
    rp2040.wdt_reset();
#if __GNUC__ >= 11
    auto const rc_save = cyphal::support::save(kv_storage, *node_registry, []() { rp2040.wdt_reset(); });
    if (rc_save.has_value())
    {
      DBG_ERROR("cyphal::support::save failed with %d", static_cast<int>(rc_save.value()));
      rsp.status = ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
    /* Feed the watchdog. */
    rp2040.wdt_reset();
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
#endif /* __GNUC__ >= 11 */
    (void)filesystem.unmount();
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
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
  else {
    rsp.status = ExecuteCommand::Response_1_1::STATUS_BAD_COMMAND;
  }

  return rsp;
}
