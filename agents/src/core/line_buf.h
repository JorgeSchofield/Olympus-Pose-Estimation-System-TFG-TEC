/*
 * line_buf.h - Assembles '\n'-terminated lines from a byte stream. Lines
 * longer than the buffer are discarded up to the next '\n' and counted.
 * Portable C99, no allocation.
 */
#ifndef PE_LINE_BUF_H
#define PE_LINE_BUF_H

#include <stddef.h>
#include <stdint.h>

#define PE_LINE_MAX 160

typedef struct {
	char     buf[PE_LINE_MAX];
	size_t   len;
	int      overflow;         /* discarding until the next '\n' */
	uint32_t n_overflow;
} pe_line_buf_t;

typedef void (*pe_line_cb_t)(void* ctx, const char* line, size_t len);

void pe_line_buf_reset(pe_line_buf_t* lb);
/* Feeds n bytes; calls cb once per complete line (without the '\n'). */
void pe_line_buf_feed(pe_line_buf_t* lb, const char* data, size_t n, pe_line_cb_t cb, void* ctx);

#endif
