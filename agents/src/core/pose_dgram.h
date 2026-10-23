/*
 * pose_dgram.h - UDP pose datagram, version 1 (DESIGN.md §11.1).
 *
 * 128 bytes, little-endian, written field by field (independent of compiler
 * padding). Decoder: tools/pose_listen.py.
 *
 *   off  type     field
 *     0  char[4]  magic "OPE1"
 *     4  u32      seq
 *     8  u64      t_pub_ns   (HLC CLOCK_MONOTONIC)
 *    16  u64      t_rx_ns    (arrival of the newest input frame)
 *    24  u32      t_llc_ms
 *    28  u16      mode
 *    30  u16      status bits (PE_ST_*)
 *    32  f64 x3   x_m, y_m, theta_rad (wrapped to [-pi, pi))
 *    56  f64 x2   omega_rad_s, bias_rad_s
 *    72  f64 x4   P_xx, P_yy, P_xy, P_thth
 *   104  f64      s_m (distance travelled)
 *   112  f64      delta (slip indicator)
 *   120  u32      reserved (0)
 *   124  u32      CRC-32 (IEEE) of bytes 0..123
 *
 * GPS datagram, version 1 (DESIGN.md §11.4): one per fix epoch, sent to its own
 * port (io.udp_gps_port). 80 bytes, same conventions. Decoder: tools/olympus-pose_gui.py.
 *
 *   off  type     field
 *     0  char[4]  magic "OGP1"
 *     4  u32      seq
 *     8  u64      t_rx_ns    (HLC CLOCK_MONOTONIC at the end of the epoch)
 *    16  f64 x2   lat_deg, lon_deg (north and east positive)
 *    32  f64      alt_m
 *    40  f64      hdop
 *    48  f64      speed_mps
 *    56  f64      course_deg
 *    64  u32      utc_ms_of_day
 *    68  u8 x4    fix_quality, n_sats, valid, reserved (0)
 *    72  u32      valid fixes so far
 *    76  u32      CRC-32 (IEEE) of bytes 0..75
 */
#ifndef PE_POSE_DGRAM_H
#define PE_POSE_DGRAM_H

#include <stdint.h>
#include "pe_msgs.h"

#define PE_POSE_DGRAM_LEN 128
#define PE_GPS_DGRAM_LEN  80

void     pe_pose_dgram_encode(const pe_pose_msg_t* m, uint32_t seq, uint64_t t_pub_ns,
                              uint8_t out[PE_POSE_DGRAM_LEN]);
void     pe_gps_dgram_encode(const pe_gps_msg_t* g, uint32_t seq, uint32_t n_valid,
                             uint8_t out[PE_GPS_DGRAM_LEN]);
uint32_t pe_crc32(const uint8_t* p, uint32_t n);

#endif
