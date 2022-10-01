/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-Cyphal/graphs/contributors.
 */

#ifndef NODE_INFO_H_
#define NODE_INFO_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <107-Arduino-Cyphal.h>
#include "pico/stdlib.h"

/**************************************************************************************
 * DEFINES
 **************************************************************************************/

#define IDSIZE 8

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

union UniqueId
{
  uint8_t byte_buf[16];
};

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

UniqueId const UNIQUE_ID = []()
{
  UniqueId uid;
  pico_unique_board_id_t pico;
  pico_get_unique_board_id(&pico);
  for (int i = 0; i < IDSIZE; i++) {
    uid.byte_buf[i] = pico.id[i];
  }
  return uid;
} ();

static const uavcan_node_GetInfo_Response_1_0 NODE_INFO = {
    /* uavcan.node.Version.1.0 protocol_version */
    {1, 0},
    /* uavcan.node.Version.1.0 hardware_version */
    {1, 0},
    /* uavcan.node.Version.1.0 software_version */
    {0, 1},
    /* saturated uint64 software_vcs_revision_id */
    NULL,
    /* saturated uint8[16] unique_id */
    {
      UNIQUE_ID.byte_buf[ 0], UNIQUE_ID.byte_buf[ 1], UNIQUE_ID.byte_buf[ 2], UNIQUE_ID.byte_buf[ 3],
      UNIQUE_ID.byte_buf[ 4], UNIQUE_ID.byte_buf[ 5], UNIQUE_ID.byte_buf[ 6], UNIQUE_ID.byte_buf[ 7],
      UNIQUE_ID.byte_buf[ 8], UNIQUE_ID.byte_buf[ 9], UNIQUE_ID.byte_buf[10], UNIQUE_ID.byte_buf[11],
      UNIQUE_ID.byte_buf[12], UNIQUE_ID.byte_buf[13], UNIQUE_ID.byte_buf[14], UNIQUE_ID.byte_buf[15]
    },
    /* saturated uint8[<=50] name */
    {
        "107-systems.l3xz-fw_aux-controller",
        strlen("107-systems.l3xz-fw_aux-controller")
    },
};

#endif /* NODE_INFO_H_ */
