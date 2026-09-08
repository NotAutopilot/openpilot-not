#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "opendbc/safety/can.h"

bool ignition_can = false;
uint32_t ignition_can_cnt = 0U;

// Pre-AP 0x348 sequence state is independent per accepted bus and is cleared
// by the production ignition CAN stale lease.
static int prev_counter_tesla_preap[2] = {-1, -1};

void ignition_can_1hz_tick(void) {
  // Keep the production ignition CAN stale lease shared across platforms.
  if (ignition_can_cnt > 2U) {
    ignition_can = false;
    prev_counter_tesla_preap[0] = -1;
    prev_counter_tesla_preap[1] = -1;
  }
  ignition_can_cnt += 1U;
}

void ignition_can_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    int len = GET_LEN(msg);

    // GM exception
    if ((msg->addr == 0x1F1U) && (len == 8)) {
      // SystemPowerMode (2=Run, 3=Crank Request)
      ignition_can = (msg->data[0] & 0x2U) != 0U;
      ignition_can_cnt = 0U;
    }

    // Rivian R1S/T GEN1 exception
    if ((msg->addr == 0x152U) && (len == 8)) {
      // 0x152 overlaps with Subaru pre-global which has this bit as the high beam
      int counter = msg->data[1] & 0xFU;  // max is only 14

      static int prev_counter_rivian = -1;
      if ((counter == ((prev_counter_rivian + 1) % 15)) && (prev_counter_rivian != -1)) {
        // VDM_OutputSignals->VDM_EpasPowerMode
        ignition_can = ((msg->data[7] >> 4U) & 0x3U) == 1U;  // VDM_EpasPowerMode_Drive_On=1
        ignition_can_cnt = 0U;
      }
      prev_counter_rivian = counter;
    }

    // Tesla Model 3/Y exception
    if ((msg->addr == 0x221U) && (len == 8)) {
      // 0x221 overlaps with Rivian which has random data on byte 0
      int counter = msg->data[6] >> 4;

      static int prev_counter_tesla = -1;
      if ((counter == ((prev_counter_tesla + 1) % 16)) && (prev_counter_tesla != -1)) {
        // VCFRONT_LVPowerState->VCFRONT_vehiclePowerState
        int power_state = (msg->data[0] >> 5U) & 0x3U;
        ignition_can = power_state == 0x3;  // VEHICLE_POWER_STATE_DRIVE=3
        ignition_can_cnt = 0U;
      }
      prev_counter_tesla = counter;
    }

    // Mazda exception
    if ((msg->addr == 0x9EU) && (len == 8)) {
      ignition_can = (msg->data[0] >> 5) == 0x6U;
      ignition_can_cnt = 0U;
    }

    // Volkswagen MEB exception
    if ((msg->addr == 0x3C0U) && (len == 4)) {
      int counter = msg->data[1] & 0xFU;

      static int prev_counter_vw_meb = -1;
      if ((counter == ((prev_counter_vw_meb + 1) % 16)) && (prev_counter_vw_meb != -1)) {
        // Klemmen_Status_01->ZAS_Kl_15
        ignition_can = ((msg->data[2] >> 1) & 1U) != 0U;
        ignition_can_cnt = 0U;
      }
      prev_counter_vw_meb = counter;
    }
  }

  // Tesla Pre-AP Model S: trusted GTW_status 0x348 on buses 0 and 1.
  // GTW_statusCounter is Motorola start-bit 51 / 4 bits = data[6] low nibble.
  // GTW_driveRailReq is data[0] bit 0. Invalid frames leave sequence state untouched.
  if (((msg->bus == 0U) || (msg->bus == 1U)) && (msg->addr == 0x348U) && (GET_LEN(msg) == 8)) {
    uint8_t checksum = 0x4BU;
    for (int i = 0; i < 7; i++) {
      checksum += msg->data[i];
    }
    if (msg->data[7] != checksum) {
      return;
    }

    // Stale-gap counter reset lives in ignition_can_1hz_tick. A per-frame reset
    // while cnt stays >2 would prevent any post-timeout pair from completing.
    const uint8_t bus_idx = msg->bus;
    int counter = msg->data[6] & 0xFU;
    const int prev = prev_counter_tesla_preap[bus_idx];
    if ((counter == ((prev + 1) % 16)) && (prev != -1)) {
      ignition_can = (msg->data[0] & 0x1U) != 0U;
      ignition_can_cnt = 0U;
    }
    prev_counter_tesla_preap[bus_idx] = counter;
  }

  // TODO: this is too loose, Teslas have 0x222
  // body v2 exception
  // if (((msg->bus == 0U) || (msg->bus == 2U)) && (msg->addr == 0x222U)) {
  //   ignition_can = true;
  //   ignition_can_cnt = 0U;
  // }
}
