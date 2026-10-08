#include "line_buf.h"

#include <string.h>

void pe_line_buf_reset(pe_line_buf_t* lb)
{
	memset(lb, 0, sizeof(*lb));
}

void pe_line_buf_feed(pe_line_buf_t* lb, const char* data, size_t n, pe_line_cb_t cb, void* ctx)
{
	size_t i;
	for (i = 0; i < n; i++) {
		char c = data[i];
		if (c == '\n') {
			if (!lb->overflow) {
				lb->buf[lb->len] = '\0';
				cb(ctx, lb->buf, lb->len);
			}
			lb->len = 0;
			lb->overflow = 0;
		}
		else if (!lb->overflow) {
			if (lb->len < PE_LINE_MAX - 1) {
				lb->buf[lb->len++] = c;
			}
			else {
				lb->overflow = 1;
				lb->n_overflow++;
				lb->len = 0;
			}
		}
	}
}
