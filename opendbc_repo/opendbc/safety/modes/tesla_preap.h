#pragma once

// ============================================
// SAFETY_TESLA_PREAP — Pre-Autopilot Tesla Model S (2012-2014)
// ============================================
//
// Standalone safety mode for Pre-AP Tesla Model S. These cars have NO
// Autopilot ECU, NO harness relay, and a different EPAS/CAN layout than
// HW1+ Teslas. This is the spiritual successor to Tinkla (Boggyver's
// Pre-AP openpilot fork, tesla_unity_betaC3 branch).
//
// WHY check_relay=false AND disable_static_blocking=true:
//
//   Pre-AP has no harness relay hardware. Standard openpilot uses a relay
//   on the harness to switch between stock AP ECU and openpilot — when
//   openpilot is not active, the relay routes CAN to the stock ECU. On
//   Pre-AP there is no AP ECU and no relay; the panda connects directly
//   to the car's CAN bus. Setting check_relay=true would cause the panda
//   to falsely detect a "relay malfunction" and block ALL TX permanently.
//   disable_static_blocking=true is required for the same reason — without
//   a relay, the panda's static blocking logic (which assumes relay state)
//   would incorrectly block messages.
//
//   Tinkla handled this identically via generic_rx_checks(false) in the
//   older panda API, with the comment "PreAP has no relay" (safety_tesla.h
//   line 1071, tesla_unity_betaC3 branch). The modern API added check_relay
//   and disable_static_blocking with restrictive defaults, so we explicitly
//   set them to get the same behavior Tinkla had implicitly.
//
// WHY ignore_checksum=true AND ignore_counter=true on RX:
//
//   Pre-AP EPAS firmware uses a byte-sum checksum, but the exact algorithm
//   has not been fully verified against all firmware versions. A checksum
//   mismatch caused a silent 21-second steering dropout during testing.
//   Tinkla's RX checks also had no checksum/counter validation (frequency
//   set to 0 for all messages). Once the checksum algorithm is verified
//   across all Pre-AP EPAS firmware versions, these can be re-enabled.
//
// ALL ACTUAL SAFETY CHECKS REMAIN FULLY ACTIVE:
//   - Steering angle + rate limits via steer_angle_cmd_checks_vm()
//   - controls_allowed gating on all TX
//   - Hands-on level >= 2: default full disengage; pause (bit 8) inhibits lat only
//   - Disengage on EPAS error codes 6-9
//   - Disengage on door open, gear out of Drive
//   - Disengage on stalk cancel (with 600ms echo filter)
//   - AEB events blocked from openpilot
//   - EPB_epasControl mode validation
//   - Pedal TX gated by PREAP_FLAG_ENABLE_PEDAL + get_longitudinal_allowed()
//
// Completely independent from tesla_legacy.h — has its own hooks struct,
// counter/checksum functions, init, RX/TX/fwd hooks, and GTW emulation.
// Registered as SAFETY_TESLA_PREAP in declarations.h.

#include "opendbc/safety/declarations.h"

// Forward declarations for panda firmware CAN send (defined in can_common.h)
#if defined(STM32H7) || defined(STM32F4)
void can_send(CANPacket_t *to_push, uint8_t bus_number, bool skip_tx_hook);
void can_set_checksum(CANPacket_t *packet);
#endif

// ============================================
// Byte manipulation macros
// ============================================

#define PREAP_GET_BYTES_04(msg) ((msg)->data[0] | ((msg)->data[1] << 8) | ((msg)->data[2] << 16) | ((msg)->data[3] << 24))
#define PREAP_GET_BYTES_48(msg) ((msg)->data[4] | ((msg)->data[5] << 8) | ((msg)->data[6] << 16) | ((msg)->data[7] << 24))
#define PREAP_WORD_TO_BYTES(dst8, src32) 0[dst8] = ((src32) & 0xFFU); 1[dst8] = (((src32) >> 8U) & 0xFFU); 2[dst8] = (((src32) >> 16U) & 0xFFU); 3[dst8] = (((src32) >> 24U) & 0xFFU)

// ============================================
// Safety param flags
// ============================================
// Longitudinal is gated by PREAP_FLAG_ENABLE_PEDAL + get_longitudinal_allowed().
// There is no separate LONG_CONTROL flag — the framework's get_longitudinal_allowed()
// is a derived check (controls_allowed && !gas_pressed_prev), not a settable flag.
// This matches how tesla.h, honda.h, and hyundai.h handle longitudinal gating.

#define PREAP_FLAG_ENABLE_PEDAL         1U
#define PREAP_FLAG_RADAR_EMULATION      2U
// Leftover bit. Position comes from host 0x560, not this flag.
#define PREAP_FLAG_RADAR_BEHIND_NOSECONE 4U
#define PREAP_FLAG_HANDS_ON_PAUSE        8U
#define PREAP_FLAG_PEDAL_BUS_ZERO       (1U << 5)
#define PREAP_FLAG_PEDAL_CALIBRATION    (1U << 6)
#define PREAP_HANDS_ON_DISENGAGE_LEVEL  2
#define PREAP_CALIBRATION_SOURCE_TIMEOUT_US 1000000U
#define PREAP_HANDS_ON_RESUME_US        1000000U

// ============================================
// State variables
// ============================================

static bool preap_enable_pedal = false;
static bool preap_radar_emulation = false;
static bool preap_pedal_calibration = false;
static bool preap_hands_on_pause = false;
static uint8_t preap_pedal_bus = 2U;
static int preap_hands_on_level = 0;
static bool preap_hands_on_clear_timing = false;
static uint32_t preap_hands_on_clear_ts = 0U;

static int preap_pedal_can = -1;

// Gear and door checks
static int preap_gear = 4;        // init to Drive to avoid false disables on startup
static int preap_gear_prev = 4;
static bool preap_doors_open = false;
static bool preap_di_brake_pressed = false;
static bool preap_brake_message_pressed = false;
static bool preap_gear_seen = false;
static uint32_t preap_gear_ts = 0U;
static bool preap_di_brake_seen = false;
static uint32_t preap_di_brake_ts = 0U;
static bool preap_brake_message_seen = false;
static uint32_t preap_brake_message_ts = 0U;
static bool preap_pedal_tx_counter_seen = false;
static uint8_t preap_pedal_tx_counter = 0U;

// Stalk echo filter
static uint32_t preap_last_stalk_engage_us = 0;
#define PREAP_CANCEL_ECHO_WINDOW_US 600000U  // 600ms

// Radar emulation state
static int preap_radar_status = 0;
static uint32_t preap_last_radar_signal = 0;
static int preap_radar_epas_type = 0;
static int preap_radar_position = 0;
static uint8_t preap_radar_vin[17];
static uint8_t preap_radar_vin_complete = 0;
static bool preap_radar_should_send = false;

// Host→panda donor config. 0x560 never goes on the car; tesla_preap_tx_hook
// consumes it. Layout matches Tinkla 0.6.6 create_radar_VIN_msg.
#define PREAP_RADAR_VIN_ADDR 0x560U
#define PREAP_RADAR_UDS_ADDR 0x641U

static bool preap_f190_payload_allowed(const CANPacket_t *msg) {
  static const uint8_t tester[8] = {0x02U, 0x3EU, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t default_session[8] = {0x02U, 0x10U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t extended_session[8] = {0x02U, 0x10U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t read_f190[8] = {0x03U, 0x22U, 0xF1U, 0x90U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t flow_control[8] = {0x30U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t cleanup_marker[8] = {0x02U, 0x3EU, 0x80U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  const uint8_t *allowed[] = {tester, default_session, extended_session, read_f190, flow_control, cleanup_marker};
  if (GET_LEN(msg) != 8U) {
    return false;
  }
  for (unsigned int i = 0U; i < (sizeof(allowed) / sizeof(allowed[0])); i++) {
    bool match = true;
    for (int b = 0; b < 8; b++) {
      if (msg->data[b] != allowed[i][b]) {
        match = false;
        break;
      }
    }
    if (match) {
      return true;
    }
  }
  return false;
}

static bool preap_f190_tx_ok(const CANPacket_t *msg) {
  // Read-only F190 on the radar bus. Never writes, routines, or security.
  if (!preap_radar_emulation || controls_allowed || controls_allowed_lateral) {
    return false;
  }
  if (GET_BUS(msg) != 1U) {
    return false;
  }
  return preap_f190_payload_allowed(msg);
}

static uint32_t preap_radar_vin_char(int pos, int shift) {
  return ((uint32_t)preap_radar_vin[pos]) << (shift * 8);
}

static bool preap_radar_ready(void) {
  // Tinkla 0.6.6 sent no radar-bus GTW frames until the host 0x560
  // stream was complete and useRadar was set. Talking earlier lets the
  // radar hear this-car VIN/2WD/EPAS0, then a different contract a
  // second later.
  return (preap_radar_vin_complete == 7U) && preap_radar_should_send;
}

static bool preap_radar_donor_active(void) {
  if (preap_radar_vin_complete != 7U) {
    return false;
  }
  // 0.6.6 default was 17 spaces. Treat that as "use this car."
  for (int i = 0; i < 17; i++) {
    if ((preap_radar_vin[i] != 0U) && (preap_radar_vin[i] != (uint8_t)' ')) {
      return true;
    }
  }
  return false;
}

static void preap_apply_radar_vin_msg(const CANPacket_t *msg) {
  const int rec = msg->data[0];
  if (rec == 0) {
    preap_radar_should_send = (msg->data[2] & 0x01U) != 0U;
    preap_radar_position = (msg->data[2] >> 1) & 0x03;
    preap_radar_epas_type = (msg->data[2] >> 3) & 0x07;
    preap_radar_vin[0] = msg->data[5];
    preap_radar_vin[1] = msg->data[6];
    preap_radar_vin[2] = msg->data[7];
    preap_radar_vin_complete |= 1U;
  } else if (rec == 1) {
    preap_radar_vin[3] = msg->data[1];
    preap_radar_vin[4] = msg->data[2];
    preap_radar_vin[5] = msg->data[3];
    preap_radar_vin[6] = msg->data[4];
    preap_radar_vin[7] = msg->data[5];
    preap_radar_vin[8] = msg->data[6];
    preap_radar_vin[9] = msg->data[7];
    preap_radar_vin_complete |= 2U;
  } else if (rec == 2) {
    preap_radar_vin[10] = msg->data[1];
    preap_radar_vin[11] = msg->data[2];
    preap_radar_vin[12] = msg->data[3];
    preap_radar_vin[13] = msg->data[4];
    preap_radar_vin[14] = msg->data[5];
    preap_radar_vin[15] = msg->data[6];
    preap_radar_vin[16] = msg->data[7];
    preap_radar_vin_complete |= 4U;
  }
}

// ============================================
// Checksum and counter (for EPAS validation)
// ============================================

static uint8_t tesla_preap_get_counter(const CANPacket_t *msg) {
  if (msg->addr == 0x370U) {
    return msg->data[6] & 0x0FU;  // EPAS_sysStatusCounter
  }
  return 0U;
}

static uint32_t tesla_preap_get_checksum(const CANPacket_t *msg) {
  if (msg->addr == 0x370U) {
    return msg->data[7];  // EPAS_sysStatusChecksum at byte 7
  }
  if (msg->addr == 0x488U) {
    return msg->data[3];  // DAS_steeringControlChecksum at byte 3
  }
  return 0U;
}

static uint32_t tesla_preap_compute_checksum(const CANPacket_t *msg) {
  // Tesla byte-sum checksum: sum of address bytes + all data bytes except checksum byte
  int checksum_byte = -1;
  if (msg->addr == 0x370U) {
    checksum_byte = 7;
  } else if (msg->addr == 0x488U) {
    checksum_byte = 3;
  }
  if (checksum_byte == -1) {
    return 0U;
  }

  uint8_t chksum = (uint8_t)(msg->addr & 0xFFU) + (uint8_t)((msg->addr >> 8) & 0xFFU);
  int len = GET_LEN(msg);
  for (int i = 0; i < len; i++) {
    if (i != checksum_byte) {
      chksum += msg->data[i];
    }
  }
  return chksum;
}

static bool tesla_preap_source_fresh(bool seen, uint32_t ts, uint32_t now) {
  return seen && (safety_get_ts_elapsed(now, ts) <= PREAP_CALIBRATION_SOURCE_TIMEOUT_US);
}

static bool tesla_preap_calibration_window_open(uint32_t now) {
  return preap_pedal_calibration &&
         tesla_preap_source_fresh(preap_gear_seen, preap_gear_ts, now) &&
         tesla_preap_source_fresh(preap_di_brake_seen, preap_di_brake_ts, now) &&
         tesla_preap_source_fresh(preap_brake_message_seen, preap_brake_message_ts, now) &&
         (preap_gear == 3) &&
         (preap_di_brake_pressed || preap_brake_message_pressed);
}

static void tesla_preap_mads_exit(const DisengageReason reason) {
  steering_control_inhibited = false;
  preap_hands_on_clear_timing = false;
  mads_exit_controls(reason);
}


// CRC-8 lookup table (polynomial 0x1D) for steering angle re-addressing
static const int preap_crc_lookup[256] = {
  0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
  0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
  0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
  0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
  0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
  0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
  0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
  0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
  0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
  0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
  0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
  0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
  0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
  0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
  0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
  0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4
};

static int preap_compute_crc8(uint32_t lo, uint32_t hi, int msg_len) {
  int crc = 0xFF;
  for (int x = 0; x < msg_len; x++) {
    int v = (x <= 3) ? ((lo >> (x * 8)) & 0xFF) : ((hi >> ((x - 4) * 8)) & 0xFF);
    crc = preap_crc_lookup[crc ^ v];
  }
  return crc ^ 0xFF;
}

// ============================================
// GTW Emulation helpers
// ============================================

static void preap_radar_readdr(const CANPacket_t *src, uint16_t new_addr) {
#if defined(STM32H7) || defined(STM32F4)
  CANPacket_t pkt;
  pkt.returned = 0U;
  pkt.rejected = 0U;
  pkt.extended = src->extended;
  pkt.bus = 1;
  pkt.addr = new_addr;
  pkt.data_len_code = src->data_len_code;
  for (int i = 0; i < GET_LEN(src); i++) {
    pkt.data[i] = src->data[i];
  }
  can_set_checksum(&pkt);
  can_send(&pkt, 1, true);
#else
  (void)src;
  (void)new_addr;
#endif
}

static void preap_transform_radar_car_config(const CANPacket_t *src, CANPacket_t *dst) {
  *dst = (CANPacket_t){.returned = 0U, .rejected = 0U, .extended = src->extended,
                       .bus = 1, .addr = 0x2A9, .data_len_code = src->data_len_code};
  uint32_t lo = PREAP_GET_BYTES_04(src);
  uint32_t hi = PREAP_GET_BYTES_48(src);
  lo = (lo & 0xFFFFF33F) | 0x100 | 0x440;  // country=US, radar_type=Bosch
  hi = (hi & 0xCFFF0F0F) | 0x10000000 | (preap_radar_position << 4) | (preap_radar_epas_type << 12);
  // Bosch xWD checks 0x2A9 against the VIN on 0x2B9, not chassis 0x398.
  // Tesla char 8 '2'/'4' is dual motor. This car VIN 5YJSA1E25FF106153 is
  // '2'; GTW still declares 2WD. Honest 2WD then latches xwdValidity and
  // freezes the object table (~12 s). Tinkla 0.6.6 ORed 4WD from that char.
  // Routes 8/9 with the OR kept a live table. Empty donor leaves 0x398 xWD
  // (old A, char 8 '1', 2WD, rock-solid).
  if (preap_radar_donor_active()) {
    const uint8_t drive = preap_radar_vin[7];
    if ((drive == (uint8_t)'2') || (drive == (uint8_t)'4')) {
      lo |= 0x08U;
    }
  }
  PREAP_WORD_TO_BYTES(&dst->data[0], lo);
  PREAP_WORD_TO_BYTES(&dst->data[4], hi);
}

static void preap_transform_radar_vin_feed(const CANPacket_t *src, CANPacket_t *dst) {
  *dst = (CANPacket_t){.returned = 0U, .rejected = 0U, .extended = src->extended,
                       .bus = 1, .addr = 0x2B9, .data_len_code = src->data_len_code};
  uint32_t lo = PREAP_GET_BYTES_04(src);
  uint32_t hi = PREAP_GET_BYTES_48(src);
  if (preap_radar_donor_active() && ((lo & 0x10U) == 0x10U)) {
    const int rec = (int)(lo & 0xFFU);
    if (rec == 0x10) {
      lo = (uint32_t)rec;
      hi = preap_radar_vin_char(0, 1) | preap_radar_vin_char(1, 2) | preap_radar_vin_char(2, 3);
    } else if (rec == 0x11) {
      lo = (uint32_t)rec | preap_radar_vin_char(3, 1) | preap_radar_vin_char(4, 2) | preap_radar_vin_char(5, 3);
      hi = preap_radar_vin_char(6, 0) | preap_radar_vin_char(7, 1) | preap_radar_vin_char(8, 2) | preap_radar_vin_char(9, 3);
    } else if (rec == 0x12) {
      lo = (uint32_t)rec | preap_radar_vin_char(10, 1) | preap_radar_vin_char(11, 2) | preap_radar_vin_char(12, 3);
      hi = preap_radar_vin_char(13, 0) | preap_radar_vin_char(14, 1) | preap_radar_vin_char(15, 2) | preap_radar_vin_char(16, 3);
    }
  }
  PREAP_WORD_TO_BYTES(&dst->data[0], lo);
  PREAP_WORD_TO_BYTES(&dst->data[4], hi);
}

#if defined(ALLOW_DEBUG) && !defined(STM32H7) && !defined(STM32F4)
static bool preap_radar_car_config_captured = false;
static CANPacket_t preap_radar_car_config_capture;
static bool preap_radar_vin_feed_captured = false;
static CANPacket_t preap_radar_vin_feed_capture;
#endif

// ============================================
// GTW Emulation: CAN0 → CAN1 for Bosch radar
// ============================================

static void tesla_preap_gtw_emulation(const CANPacket_t *to_fwd) {
  int bus_num = GET_BUS(to_fwd);
  int addr = GET_ADDR(to_fwd);

  if (bus_num == 0 && preap_radar_emulation && preap_radar_ready()) {
    // Group A: Simple re-addresses
    switch (addr) {
      case 0x45:   preap_radar_readdr(to_fwd, 0x219); break;  // STW_ACTN_RQ
      case 0x108:  preap_radar_readdr(to_fwd, 0x109); break;  // DI_torque1
      case 0x145:  preap_radar_readdr(to_fwd, 0x149); break;  // ESP_145h
      case 0x20A:  preap_radar_readdr(to_fwd, 0x159); break;  // BrakeMessage -> ESP_C
      case 0x308:  preap_radar_readdr(to_fwd, 0x209); break;  // GTW_odo
      case 0x30A:  preap_radar_readdr(to_fwd, 0x2D9); break;  // BC_status
      default: break;
    }

    if (addr == 0x405) {
      CANPacket_t vin_pkt;
      preap_transform_radar_vin_feed(to_fwd, &vin_pkt);
#if defined(ALLOW_DEBUG) && !defined(STM32H7) && !defined(STM32F4)
      preap_radar_vin_feed_capture = vin_pkt;
      preap_radar_vin_feed_captured = true;
#endif
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&vin_pkt);
      can_send(&vin_pkt, 1, true);
#endif
    }

    // Group B: GTW_carConfig (0x398) → 0x2A9 with bitfield patching
    if (addr == 0x398) {
      CANPacket_t pkt;
      preap_transform_radar_car_config(to_fwd, &pkt);
#if defined(ALLOW_DEBUG) && !defined(STM32H7) && !defined(STM32F4)
      preap_radar_car_config_capture = pkt;
      preap_radar_car_config_captured = true;
#endif
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&pkt);
      can_send(&pkt, 1, true);
#endif
    }

    // Group B: STW_ANGLHP_STAT (0x0E) → 0x199 with SNA replacement
    if (addr == 0x0E) {
      CANPacket_t pkt = {.returned = 0U, .rejected = 0U, .extended = to_fwd->extended,
                         .bus = 1, .addr = 0x199, .data_len_code = to_fwd->data_len_code};
      uint32_t lo = PREAP_GET_BYTES_04(to_fwd);
      uint32_t hi = PREAP_GET_BYTES_48(to_fwd);
      if (((lo >> 16) & 0xFF3F) == 0xFF3F) {
        lo = (lo & 0x00C0FFFF) | (0x0020 << 16);
        hi = (hi & 0x00FFFFF0) | 0x00000004;  // force DELPHI sensor ID
        int crc = preap_compute_crc8(lo, hi, 7);
        hi = hi | ((uint32_t)crc << 24);
      }
      PREAP_WORD_TO_BYTES(&pkt.data[0], lo);
      PREAP_WORD_TO_BYTES(&pkt.data[4], hi);
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&pkt);
      can_send(&pkt, 1, true);
#endif
    }

    // Group C: ESP_115h (0x115) → 0x129 + synthetic DI_espControl (0x1A9)
    if (addr == 0x115) {
      preap_radar_readdr(to_fwd, 0x129);
      uint32_t hi_src = PREAP_GET_BYTES_48(to_fwd);
      int counter = ((hi_src & 0xF0) >> 4) & 0x0F;
      uint32_t syn_lo = 0x000C0000U | ((uint32_t)counter << 28);
      int cksm = (0x38 + 0x0C + (counter << 4)) & 0xFF;
      CANPacket_t pkt = {.returned = 0U, .rejected = 0U, .extended = 0,
                         .bus = 1, .addr = 0x1A9, .data_len_code = 5};
      PREAP_WORD_TO_BYTES(&pkt.data[0], syn_lo);
      PREAP_WORD_TO_BYTES(&pkt.data[4], (uint32_t)cksm);
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&pkt);
      can_send(&pkt, 1, true);
#endif
    }

    // Group C: DI_torque2 (0x118) → 0x119 + synthetic ESP_wheelSpeeds (0x169)
    if (addr == 0x118) {
      preap_radar_readdr(to_fwd, 0x119);
      uint32_t lo = PREAP_GET_BYTES_04(to_fwd);
      int ws_counter = PREAP_GET_BYTES_48(to_fwd) & 0x0F;
      int raw_speed = (int)((0xFFF0000U & lo) >> 16);
      int speed;
      if (raw_speed == 0xFFF) {
        speed = 0x1FFF;
      } else {
        int mph_x100 = raw_speed * 5 - 2500;
        int kph_x100 = mph_x100 * 1609 / 1000;
        speed = (kph_x100 < 0) ? 0 : ((kph_x100 / 4) & 0x1FFF);
      }
      uint32_t ws_lo = (uint32_t)(speed | (speed << 13) | (speed << 26));
      uint32_t ws_hi = (uint32_t)((speed >> 6) | (speed << 7) | (ws_counter << 20)) & 0x00FFFFFFU;
      int ws_cksm = 0x76;
      ws_cksm = (ws_cksm + (int)(ws_lo & 0xFF) + (int)((ws_lo >> 8) & 0xFF) + (int)((ws_lo >> 16) & 0xFF) + (int)((ws_lo >> 24) & 0xFF)) & 0xFF;
      ws_cksm = (ws_cksm + (int)(ws_hi & 0xFF) + (int)((ws_hi >> 8) & 0xFF) + (int)((ws_hi >> 16) & 0xFF)) & 0xFF;
      ws_hi = ws_hi | ((uint32_t)ws_cksm << 24);
      CANPacket_t pkt = {.returned = 0U, .rejected = 0U, .extended = 0,
                         .bus = 1, .addr = 0x169, .data_len_code = 8};
      PREAP_WORD_TO_BYTES(&pkt.data[0], ws_lo);
      PREAP_WORD_TO_BYTES(&pkt.data[4], ws_hi);
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&pkt);
      can_send(&pkt, 1, true);
#endif
    }
  }

  // Radar status tracking (CAN1 → informational only)
  if (bus_num == 1 && preap_radar_emulation) {
    if (addr == 0x631 && preap_radar_status == 0) {
      preap_radar_status = 1;
      preap_last_radar_signal = microsecond_timer_get();
    }
    if (addr == 0x300 && preap_radar_status == 1) {
      preap_radar_status = 2;
      preap_last_radar_signal = microsecond_timer_get();
    }
  }
}

#if defined(ALLOW_DEBUG) && !defined(STM32H7) && !defined(STM32F4)
bool tesla_preap_radar_car_config_captured(void) {
  return preap_radar_car_config_captured;
}

uint32_t tesla_preap_radar_car_config_addr(void) {
  return preap_radar_car_config_capture.addr;
}

uint8_t tesla_preap_radar_car_config_bus(void) {
  return preap_radar_car_config_capture.bus;
}

uint8_t tesla_preap_radar_car_config_dlc(void) {
  return preap_radar_car_config_capture.data_len_code;
}

uint8_t tesla_preap_radar_car_config_data(int index) {
  if ((index < 0) || (index >= 8)) {
    return 0U;
  }
  return preap_radar_car_config_capture.data[index];
}

bool tesla_preap_radar_vin_feed_captured(void) {
  return preap_radar_vin_feed_captured;
}

uint8_t tesla_preap_radar_vin_feed_data(int index) {
  if ((index < 0) || (index >= 8)) {
    return 0U;
  }
  return preap_radar_vin_feed_capture.data[index];
}

bool tesla_preap_radar_donor_active_debug(void) {
  return preap_radar_donor_active();
}

bool tesla_preap_radar_ready_debug(void) {
  return preap_radar_ready();
}
#endif

// ============================================
// RX Hook
// ============================================

static void tesla_preap_rx_hook(const CANPacket_t *msg) {
  // Pedal interceptor (0x552) — may arrive on bus 0 OR bus 2 depending on wiring.
  // Must be handled BEFORE the bus-0-only bailout below.
  // Whitelisted on both bus 0 and bus 2 in preap_rx_checks; the framework has
  // already verified the message matches one of them, so accept either here.
  //
  // Gas-press threshold: 650 raw, chosen from real Pre-AP drive data:
  //   - At-rest noise (driver not pressing): raw range 424-633, mean 470 (p99.9=602)
  //   - Actual gas press: raw range 441-1246, mean 799 (p10=607, p50=802)
  // The original threshold of 450 was inside the resting noise distribution and
  // caused false gas_pressed readings that blocked pedal TX → pedal wouldn't engage.
  // 650 gives zero false positives on rest noise while still catching the vast
  // majority of real driver presses. Python-layer DI_pedalPos is the primary
  // gas-override detection; the panda threshold here is a safety backstop.
  if (preap_enable_pedal && (msg->addr == 0x552U)) {
    int pedal_val = ((msg->data[0] << 8) | msg->data[1]);
    gas_pressed = (pedal_val > 650);
    if (preap_pedal_can == -1) {
      preap_pedal_can = msg->bus;
    }
    return;
  }

  // All other RX handlers are bus 0 only.
  if (msg->bus != 0U) return;

  // EPAS (0x370): steering angle, hands-on level, disengage detection
  if (msg->addr == 0x370U) {
    const int angle_meas_new = (((msg->data[4] & 0x3FU) << 8) | msg->data[5]) - 8192U;
    update_sample(&angle_meas, angle_meas_new);

    const int hands_on_level = msg->data[4] >> 6;
    const int eac_status = msg->data[6] >> 5;
    const int eac_error_code = msg->data[2] >> 4;

    bool epas_rejecting = (eac_status == 0) && (eac_error_code >= 6) && (eac_error_code <= 9);
    const bool eac_fault = (eac_status == 3);
    preap_hands_on_level = hands_on_level;
    const bool hands_on = hands_on_level >= PREAP_HANDS_ON_DISENGAGE_LEVEL;
    if (preap_hands_on_pause && !eac_fault && !epas_rejecting && hands_on && controls_allowed_lateral) {
      // Pause lat only. Keep already-active long; do not pcm_cruise_check(false)
      // and do not admit a new long request while inhibited.
      steering_control_inhibited = true;
      preap_hands_on_clear_timing = false;
      steering_disengage = false;
    } else if (epas_rejecting || eac_fault || (hands_on && !preap_hands_on_pause)) {
      steering_disengage = true;
      tesla_preap_mads_exit(MADS_DISENGAGE_REASON_STEERING_DISENGAGE);
      pcm_cruise_check(false);
    } else if (preap_hands_on_pause && steering_control_inhibited) {
      steering_disengage = false;
      const bool clear_ok = (hands_on_level < PREAP_HANDS_ON_DISENGAGE_LEVEL) &&
                            (preap_gear == 4) && !preap_doors_open &&
                            controls_allowed_lateral;
      if (!clear_ok) {
        preap_hands_on_clear_timing = false;
      } else {
        const uint32_t now = microsecond_timer_get();
        if (!preap_hands_on_clear_timing) {
          preap_hands_on_clear_timing = true;
          preap_hands_on_clear_ts = now;
        } else if (safety_get_ts_elapsed(now, preap_hands_on_clear_ts) >= PREAP_HANDS_ON_RESUME_US) {
          steering_control_inhibited = false;
          preap_hands_on_clear_timing = false;
        }
      }
    } else {
      steering_disengage = false;
    }
  }

  // Vehicle speed (ESP_B: 0x155) — derive vehicle_moving from actual speed
  if (msg->addr == 0x155U) {
    float speed = (((msg->data[5] << 8) | msg->data[6]) * 0.01f) * KPH_TO_MS;
    UPDATE_VEHICLE_SPEED(speed);
    vehicle_moving = speed > (0.5f * KPH_TO_MS);
  }

  // Gas pressed from DI_torque1 (0x108) — only when pedal interceptor is not active.
  // (The pedal interceptor path is handled above the bus-0-only bailout since it may
  // arrive on bus 0 or bus 2.)
  if (msg->addr == 0x108U) {
    if (!preap_enable_pedal) {
      gas_pressed = msg->data[6] != 0U;
    }
  }

  // Brake (0x20a) — latch pedal authority separately, but keep the framework
  // brake false so generic_rx_checks doesn't drop lateral controls_allowed.
  if (msg->addr == 0x20aU) {
    preap_brake_message_pressed = ((msg->data[0] >> 2) & 0x03U) == 2U;
    brake_pressed = false;
    preap_brake_message_seen = true;
    preap_brake_message_ts = microsecond_timer_get();
  }

  // Cruise state (DI_state: 0x368) — vehicle_moving only, engagement via stalk
  if (msg->addr == 0x368U) {
    int cruise_state = (msg->data[1] >> 4) & 0x07U;
    // Backup vehicle_moving from cruise state (standstill detection)
    if (cruise_state == 3) {
      vehicle_moving = false;
    }
  }

  // DI brake closes the interval before the slower BrakeMessage arrives.
  // The same frame also disables controls on leaving Drive.
  if (msg->addr == 0x118U) {
    preap_di_brake_pressed = ((msg->data[1] >> 7) & 0x01U) != 0U;
    preap_gear = (msg->data[1] >> 4) & 0x07;
    preap_di_brake_seen = true;
    preap_di_brake_ts = microsecond_timer_get();
    preap_gear_seen = true;
    preap_gear_ts = preap_di_brake_ts;
    if ((preap_gear_prev == 4) && (preap_gear != 4)) {
      controls_allowed = false;
      tesla_preap_mads_exit(MADS_DISENGAGE_REASON_ACC_MAIN_OFF);
    }
    preap_gear_prev = preap_gear;
  }

  // Door check (GTW_carState: 0x318)
  if (msg->addr == 0x318U) {
    int d_fl = (msg->data[1] >> 4) & 0x03;
    int d_fr = (msg->data[1] >> 6) & 0x03;
    int d_rl = (msg->data[2] >> 6) & 0x03;
    int d_rr = (msg->data[3] >> 5) & 0x03;
    int d_ft = (msg->data[6] >> 2) & 0x03;
    int d_tr = (msg->data[5] >> 6) & 0x03;
    preap_doors_open = (d_fl == 1) || (d_fr == 1) || (d_rl == 1) || (d_rr == 1) || (d_ft == 1) || (d_tr == 1);
    if (preap_doors_open) {
      controls_allowed = false;
      tesla_preap_mads_exit(MADS_DISENGAGE_REASON_ACC_MAIN_OFF);
    }
  }

  // Stalk engagement (STW_ACTN_RQ: 0x45) with echo-filtered cancel.
  // pcm_cruise_check latches only on a rising cruise_engaged_prev. Board
  // heartbeat can drop controls_allowed after 3s of lat-only without clearing
  // that prev, so a later held lever==2 would not re-admit. Force a rising
  // edge only on a release-to-pull; held MAIN and inhibit/hands recovery
  // alone must not rearm. Reuses cruise_button_prev (IDLE=0) for the edge.
  if (msg->addr == 0x45U) {
    int lever = msg->data[0] & 0x3FU;
    const bool fresh_pull = (lever == 2) && (cruise_button_prev != 2);
    if (lever == 2) {  // RWD = pull toward driver = enable
      if (fresh_pull && (preap_gear == 4) && !preap_doors_open &&
          (preap_hands_on_level < PREAP_HANDS_ON_DISENGAGE_LEVEL) &&
          !steering_control_inhibited) {
        cruise_engaged_prev = false;
        pcm_cruise_check(true);
        preap_last_stalk_engage_us = microsecond_timer_get();
      }
    } else if (lever == 1) {  // FWD = push away = cancel
      uint32_t elapsed = microsecond_timer_get() - preap_last_stalk_engage_us;
      if (elapsed > PREAP_CANCEL_ECHO_WINDOW_US) {
        pcm_cruise_check(false);
        tesla_preap_mads_exit(MADS_DISENGAGE_REASON_BUTTON);
      }
    }
    cruise_button_prev = lever;
  }

  // No relay, so stock_ecu_check never runs. Stalk pull's controls_allowed
  // rising edge requests MADS lateral; hands-on/EPAS steering_disengage exits it.
  mads_state_update(vehicle_moving, acc_main_on, controls_allowed, brake_pressed || regen_braking, steering_disengage);
  if (preap_pedal_calibration) {
    controls_allowed = false;
    controls_allowed_lateral = false;
  }
}

// ============================================
// TX Hook
// ============================================

static bool tesla_preap_tx_hook(const CANPacket_t *msg) {
  const AngleSteeringLimits PREAP_STEERING_LIMITS = {
    .max_angle = 3600,  // 360 deg, EPAS faults above this
    .angle_deg_to_can = 10,
    .frequency = 50U,
  };

  // Pre-AP Model S is physically the same car as HW1/HW2/HW3 Model S.
  // These values MUST match VehicleModel(TESLA_MODEL_S_HW3) in carcontroller.py.
  // Verified: mass=2100+STD_CARGO_KG, wheelbase=2.960, steerRatio=15.0
  //           → slip_factor = -0.0005666 (calc_slip_factor)
  // Confirmed by Lukas (xnor-tech, former comma employee, Tesla port author).
  const AngleSteeringParams PREAP_STEERING_PARAMS = {
    .slip_factor = -0.0005666,
    .steer_ratio = 15.,
    .wheelbase = 2.96,
  };

  bool tx = true;
  bool violation = false;

  // Host→panda donor VIN/config. Intercept; do not put 0x560 on the car.
  if (msg->addr == PREAP_RADAR_VIN_ADDR) {
    preap_apply_radar_vin_msg(msg);
    return false;
  }

  // Radar UDS on bus 1. Allow only the F190 read sequence while disengaged.
  if (msg->addr == PREAP_RADAR_UDS_ADDR) {
    return preap_f190_tx_ok(msg);
  }

  // DAS_steeringControl (0x488)
  if (msg->addr == 0x488U) {
    int raw_angle_can = ((msg->data[0] & 0x7FU) << 8) | msg->data[1];
    int desired_angle = raw_angle_can - 16384;
    int steer_control_type = msg->data[2] >> 6;
    bool steer_control_enabled = steer_control_type == 1;

    if (steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled, PREAP_STEERING_LIMITS, PREAP_STEERING_PARAMS)) {
      violation = true;
    }
    if ((steer_control_type != 0) && (steer_control_type != 1)) {
      violation = true;
    }
    if (steering_control_inhibited && steer_control_enabled) {
      violation = true;
    }
  }

  // EPB_epasControl (0x214): only allow valid EAC modes (0=disable, 1=enable)
  if (msg->addr == 0x214U) {
    int epas_control_type = msg->data[0] & 0x07U;  // EPB_epasEACAllow: bits 2:0 of byte 0
    if (epas_control_type > 1) {
      violation = true;
    }
  }

  // DAS_control (0x2B9): no AEB events from openpilot
  if (msg->addr == 0x2B9U) {
    int aeb_event = msg->data[2] & 0x03U;
    if (aeb_event != 0) {
      violation = true;
    }
  }

  // Pedal interceptor (0x551 GAS_COMMAND): parse ENABLE bit and GAS_COMMAND
  // value to distinguish authoritative accel commands from release commands.
  //   DBC: SG_ ENABLE : 39|1@0+  →  bit 7 of data[4]
  //   DBC: SG_ GAS_COMMAND : 7|16@0+  →  bytes 0-1 big-endian (physical 0 = raw 450)
  //
  //   ENABLE=0: openpilot is releasing control. Comma Pedal ignores GAS_COMMAND
  //   and passes the driver's OEM pedal voltage through. The controller sends
  //   one disabled-zero frame when relinquishing active authority or resetting
  //   faulted firmware, then stays silent.
  //   Defense-in-depth: we still require the GAS_COMMAND raw value to be at or
  //   below the zero point (raw <= 500, which is ~2.5% physical) so a bugged or
  //   malicious ENABLE=0 + high-value message can't sneak through a potential
  //   Comma Pedal firmware bug.
  //
  //   ENABLE=1: authoritative actuation command. Gated by get_longitudinal_allowed()
  //   (controls_allowed && !gas_pressed_prev).
  if (msg->addr == 0x551U) {
    if (preap_pedal_calibration) {
      const bool pedal_enable = (msg->data[4] & 0x80U) != 0U;
      const int raw_gas_cmd = (msg->data[0] << 8) | msg->data[1];
      const int raw_gas_cmd2 = (msg->data[2] << 8) | msg->data[3];
      const uint8_t counter = msg->data[4] & 0x0FU;
      uint8_t chksum = (uint8_t)(msg->addr & 0xFFU) + (uint8_t)((msg->addr >> 8) & 0xFFU);
      for (int i = 0; i < 5; i++) {
        chksum += msg->data[i];
      }
      const bool protocol_valid = (GET_BUS(msg) == preap_pedal_bus) &&
                                  (GET_LEN(msg) == 6U) &&
                                  !msg->fd &&
                                  ((msg->data[4] & 0x70U) == 0U) &&
                                  (chksum == msg->data[5]) &&
                                  (raw_gas_cmd < 65535) && (raw_gas_cmd2 < 65535) &&
                                  (!preap_pedal_tx_counter_seen ||
                                   (counter == (uint8_t)((preap_pedal_tx_counter + 1U) & 0x0FU)));
      if (!protocol_valid) {
        violation = true;
      } else if (pedal_enable) {
        if (!tesla_preap_calibration_window_open(microsecond_timer_get())) {
          violation = true;
        }
      } else if ((raw_gas_cmd > 500) || (raw_gas_cmd2 > 500)) {
        violation = true;
      }
      if (!violation) {
        preap_pedal_tx_counter_seen = true;
        preap_pedal_tx_counter = counter;
      }
    } else if (!preap_enable_pedal) {
      violation = true;
    } else {
      bool pedal_enable = (msg->data[4] & 0x80U) != 0U;
      int raw_gas_cmd = (msg->data[0] << 8) | msg->data[1];
      if (pedal_enable) {
        if (!get_longitudinal_allowed() || preap_di_brake_pressed || preap_brake_message_pressed) {
          violation = true;
        }
      } else {
        // ENABLE=0: only allow near-zero GAS_COMMAND values (defense-in-depth).
        // This admits the production raw-zero release and DBC physical zero.
        if (raw_gas_cmd > 500) {
          violation = true;
        }
      }
    }
  }

  // DAS_bodyControls (0x3E9): turn-signal actuation. Gate on controls_allowed
  // (matches all other Pre-AP TX) so the indicator can only be driven while
  // openpilot is engaged — on disengage controlsd clears the blinker anyway,
  // and this is the defense-in-depth backstop. Also bound the turn-indicator
  // request to valid values (0-3); the field is 2 bits (bit 8 = byte 1 bits
  // 0-1) so it cannot exceed 3 — a guard if the field width ever changes.
  if (msg->addr == 0x3E9U) {
    int turn_req = (msg->data[1] & 0x03U);  // DAS_turnIndicatorRequest at bit 8
    if (turn_req > 3) {
      violation = true;
    }
    if (!(controls_allowed || controls_allowed_lateral)) {
      violation = true;
    }
  }

  if (violation) {
    tx = false;
  }
  return tx;
}

// ============================================
// Forwarding Hook
// ============================================

static bool tesla_preap_fwd_hook(int bus_num, int addr) {
  (void)bus_num;
  (void)addr;
  // Pre-AP has no AP ECU on bus 2. Block default 0↔2 forwarding to avoid
  // flooding a dead TX queue.
  return true;
}

// ============================================
// Init
// ============================================

static safety_config tesla_preap_init(uint16_t param) {
  const bool calib_requested = GET_FLAG(param, PREAP_FLAG_PEDAL_CALIBRATION);
  const bool mixed_calib = calib_requested &&
                           (GET_FLAG(param, PREAP_FLAG_ENABLE_PEDAL) ||
                            GET_FLAG(param, PREAP_FLAG_RADAR_EMULATION) ||
                            GET_FLAG(param, PREAP_FLAG_RADAR_BEHIND_NOSECONE) ||
                            GET_FLAG(param, PREAP_FLAG_HANDS_ON_PAUSE));
  preap_pedal_calibration = calib_requested && !mixed_calib;
  preap_enable_pedal = GET_FLAG(param, PREAP_FLAG_ENABLE_PEDAL) && !preap_pedal_calibration && !mixed_calib;
  preap_radar_emulation = GET_FLAG(param, PREAP_FLAG_RADAR_EMULATION) && !preap_pedal_calibration && !mixed_calib;
  preap_hands_on_pause = GET_FLAG(param, PREAP_FLAG_HANDS_ON_PAUSE) && !preap_pedal_calibration && !mixed_calib;
  preap_pedal_bus = GET_FLAG(param, PREAP_FLAG_PEDAL_BUS_ZERO) ? 0U : 2U;
  steering_control_inhibited = false;
  preap_hands_on_level = 0;
  preap_hands_on_clear_timing = false;
  preap_hands_on_clear_ts = 0U;
  preap_gear = 4;
  preap_gear_prev = 4;
  preap_doors_open = false;
  preap_di_brake_pressed = false;
  preap_brake_message_pressed = false;
  preap_gear_seen = false;
  preap_gear_ts = 0U;
  preap_di_brake_seen = false;
  preap_di_brake_ts = 0U;
  preap_brake_message_seen = false;
  preap_brake_message_ts = 0U;
  preap_pedal_tx_counter_seen = false;
  preap_pedal_tx_counter = 0U;
  preap_pedal_can = -1;
  preap_radar_status = 0;
  preap_last_radar_signal = 0;
  preap_last_stalk_engage_us = 0;
  preap_radar_position = 0;
  preap_radar_epas_type = 0;
  preap_radar_vin_complete = 0;
  preap_radar_should_send = false;
  for (int i = 0; i < 17; i++) {
    preap_radar_vin[i] = (uint8_t)' ';
  }
#if defined(ALLOW_DEBUG) && !defined(STM32H7) && !defined(STM32F4)
  preap_radar_car_config_captured = false;
  preap_radar_vin_feed_captured = false;
#endif

  // TX whitelist — no harness relay on Pre-AP
  static const CanMsg PREAP_TX_MSGS[] = {
    {0x488, 0, 4, .check_relay = false, .disable_static_blocking = true},  // DAS_steeringControl
    {0x2B9, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_control
    {0x214, 0, 3, .check_relay = false, .disable_static_blocking = true},  // EPB_epasControl
    {0x551, 0, 6, .check_relay = false, .disable_static_blocking = true},  // Pedal on bus 0
    {0x551, 2, 6, .check_relay = false, .disable_static_blocking = true},  // Pedal on bus 2
    {0x45,  0, 8, .check_relay = false, .disable_static_blocking = true},  // STW_ACTN_RQ (stalk spoof)
    {0x3E9, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_bodyControls (turn signal)
    {0x560, 0, 8, .check_relay = false, .disable_static_blocking = true},  // donor VIN/config to panda
    {0x641, 1, 8, .check_relay = false, .disable_static_blocking = true},  // radar F190 read
  };

  // RX checks — disable EPAS counter/checksum until we verify the Pre-AP
  // EPAS firmware's checksum matches our compute_checksum exactly.
  // Mismatched validation caused silent 21s steering dropout.
  static RxCheck preap_rx_checks[] = {
    {.msg = {{0x370, 0, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // EPAS_sysStatus
    {.msg = {{0x108, 0, 8, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},  // DI_torque1
    {.msg = {{0x118, 0, 6, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},  // DI_torque2
    {.msg = {{0x20a, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // BrakeMessage
    {.msg = {{0x368, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DI_state
    {.msg = {{0x318, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // GTW_carState
    {.msg = {{0x45,  0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // STW_ACTN_RQ
    {.msg = {{0x155, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // ESP_B
  };

  // Pedal-enabled variant: adds 0x552 (GAS_SENSOR) to rx_checks so the
  // framework routes it to the rx hook. Split into its own array because
  // frequency=0 causes divide-by-zero in safety_tick (safety.h:330), which
  // marks the check as lagging and trips safetyRxChecksInvalid → controls
  // mismatch on cars without a pedal. 50Hz matches the Comma Pedal firmware.
  static RxCheck preap_rx_checks_with_pedal[] = {
    {.msg = {{0x370, 0, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x108, 0, 8, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x118, 0, 6, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x20a, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x368, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x318, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x45,  0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x155, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x552, 0, 6, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true},
             {0x552, 2, 6, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }}},  // GAS_SENSOR
  };

  static const CanMsg PREAP_TX_MSGS_CAL_BUS0[] = {
    {0x551, 0, 6, .check_relay = false, .disable_static_blocking = true},
  };
  static const CanMsg PREAP_TX_MSGS_CAL_BUS2[] = {
    {0x551, 2, 6, .check_relay = false, .disable_static_blocking = true},
  };
  if (preap_pedal_calibration) {
    return (preap_pedal_bus == 0U) ? BUILD_SAFETY_CFG(preap_rx_checks, PREAP_TX_MSGS_CAL_BUS0)
                                   : BUILD_SAFETY_CFG(preap_rx_checks, PREAP_TX_MSGS_CAL_BUS2);
  }
  return preap_enable_pedal ? BUILD_SAFETY_CFG(preap_rx_checks_with_pedal, PREAP_TX_MSGS)
                            : BUILD_SAFETY_CFG(preap_rx_checks, PREAP_TX_MSGS);
}

// ============================================
// Hooks struct
// ============================================

const safety_hooks tesla_preap_hooks = {
  .init = tesla_preap_init,
  .rx = tesla_preap_rx_hook,
  .rx_all = tesla_preap_gtw_emulation,  // must see ALL CAN traffic for radar GTW forwarding
  .tx = tesla_preap_tx_hook,
  .fwd = tesla_preap_fwd_hook,
  .get_counter = tesla_preap_get_counter,
  .get_checksum = tesla_preap_get_checksum,
  .compute_checksum = tesla_preap_compute_checksum,
  .get_quality_flag_valid = NULL,
};
