/*
 * pose_dgram.c - Serialiser of the UDP pose datagram. Portable C99.
 */
#include "pose_dgram.h"

#include <string.h>

static void put_u16(uint8_t* p, uint16_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t* p, uint32_t v)
{
	put_u16(p, (uint16_t)v);
	put_u16(p + 2, (uint16_t)(v >> 16));
}

static void put_u64(uint8_t* p, uint64_t v)
{
	put_u32(p, (uint32_t)v);
	put_u32(p + 4, (uint32_t)(v >> 32));
}

static void put_f64(uint8_t* p, double d)
{
	uint64_t v;
	memcpy(&v, &d, sizeof(v));   /* IEEE-754 binary64 on every target we use */
	put_u64(p, v);
}

uint32_t pe_crc32(const uint8_t* p, uint32_t n)
{
	uint32_t crc = 0xFFFFFFFFu, i;
	int k;
	for (i = 0; i < n; i++) {
		crc ^= p[i];
		for (k = 0; k < 8; k++) {
			crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
		}
	}
	return ~crc;
}

void pe_pose_dgram_encode(const pe_pose_msg_t* m, uint32_t seq, uint64_t t_pub_ns,
                          uint8_t out[PE_POSE_DGRAM_LEN])
{
	memset(out, 0, PE_POSE_DGRAM_LEN);
	memcpy(out, "OPE1", 4);
	put_u32(out + 4, seq);
	put_u64(out + 8, t_pub_ns);
	put_u64(out + 16, m->hdr.t_rx_ns);
	put_u32(out + 24, m->t_llc_ms);
	put_u16(out + 28, m->mode);
	put_u16(out + 30, m->status);
	put_f64(out + 32, m->x[0]);
	put_f64(out + 40, m->x[1]);
	put_f64(out + 48, m->x[2]);
	put_f64(out + 56, m->x[3]);
	put_f64(out + 64, m->x[4]);
	put_f64(out + 72, m->P[0]);        /* P_xx   (5x5, column-major) */
	put_f64(out + 80, m->P[6]);        /* P_yy */
	put_f64(out + 88, m->P[1]);        /* P_xy */
	put_f64(out + 96, m->P[12]);       /* P_thth */
	put_f64(out + 104, m->s_m);
	put_f64(out + 112, m->diag[0]);
	put_u32(out + 120, 0u);
	put_u32(out + 124, pe_crc32(out, 124u));
}
