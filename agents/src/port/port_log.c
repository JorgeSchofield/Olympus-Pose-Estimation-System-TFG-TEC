/*
 * port_log.c - Asynchronous log writer (DESIGN.md §7.4, §11.2).
 *
 * Producers (agents, llcmux) copy records into one static ring under a mutex
 * and return at once; a SCHED_OTHER thread writes them to their files. An SD
 * card stall therefore never blocks a real-time thread. A full ring drops the
 * record and counts it.
 */
#include "port.h"

#include <errno.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define RING_BYTES (1u << 20)        /* 1 MiB: ~20 s of every stream at full rate */
#define REC_HDR 3u                   /* stream id (1 byte) + length (2 bytes) */
#define REC_MAX 1024u

static unsigned char   ring[RING_BYTES];
static uint32_t        r_head, r_tail;         /* bytes; head = write, tail = read */
static pthread_mutex_t r_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  r_cond = PTHREAD_COND_INITIALIZER;   /* data available */
static pthread_cond_t  r_space = PTHREAD_COND_INITIALIZER;  /* space freed (blocking mode) */
static int             blocking;                            /* replay: wait instead of dropping */
static FILE*           files[PE_LOG_MAX_STREAMS];
static int             n_streams;
static int             running;
static uint32_t        drops;
static pthread_t       writer;
static char            run_dir_path[512];

static uint32_t used_bytes(void)
{
	return r_head - r_tail;   /* free-running counters, wrap-safe */
}

static void ring_put(const unsigned char* p, uint32_t n)
{
	uint32_t i;
	for (i = 0; i < n; i++) {
		ring[(r_head + i) % RING_BYTES] = p[i];
	}
	r_head += n;
}

static void ring_get(unsigned char* p, uint32_t n)
{
	uint32_t i;
	for (i = 0; i < n; i++) {
		p[i] = ring[(r_tail + i) % RING_BYTES];
	}
	r_tail += n;
}

static void* writer_main(void* arg)
{
	unsigned char rec[REC_HDR + REC_MAX];
	(void)arg;
	pthread_mutex_lock(&r_lock);
	for (;;) {
		while (used_bytes() == 0 && running) {
			pthread_cond_wait(&r_cond, &r_lock);
		}
		if (used_bytes() == 0 && !running) {
			break;
		}
		{
			uint32_t len;
			int id;
			ring_get(rec, REC_HDR);
			id = rec[0];
			len = (uint32_t)rec[1] | ((uint32_t)rec[2] << 8);
			ring_get(rec + REC_HDR, len);
			pthread_cond_broadcast(&r_space);
			pthread_mutex_unlock(&r_lock);
			if (id < n_streams && files[id] != NULL) {
				fwrite(rec + REC_HDR, 1, len, files[id]);
			}
			pthread_mutex_lock(&r_lock);
		}
		if (used_bytes() == 0) {
			int i;
			pthread_mutex_unlock(&r_lock);
			for (i = 0; i < n_streams; i++) {
				if (files[i] != NULL) {
					fflush(files[i]);
				}
			}
			pthread_mutex_lock(&r_lock);
		}
	}
	pthread_mutex_unlock(&r_lock);
	return NULL;
}

int pe_log_start(const char* base, char* run_dir, size_t run_dir_len)
{
	time_t now = time(NULL);
	struct tm tmv;
	char stamp[32];
	localtime_r(&now, &tmv);
	strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &tmv);
	{   /* mkdir -p base */
		char p[512];
		size_t k;
		snprintf(p, sizeof(p), "%s", base);
		for (k = 1; p[k] != '\0'; k++) {
			if (p[k] == '/') {
				p[k] = '\0';
				mkdir(p, 0755);
				p[k] = '/';
			}
		}
		mkdir(p, 0755);
	}
	snprintf(run_dir_path, sizeof(run_dir_path), "%s/%s", base, stamp);
	if (mkdir(run_dir_path, 0755) != 0 && errno != EEXIST) {
		return -1;
	}
	if (run_dir != NULL) {
		snprintf(run_dir, run_dir_len, "%s", run_dir_path);
	}
	running = 1;
	if (pthread_create(&writer, NULL, writer_main, NULL) != 0) {
		running = 0;
		return -1;
	}
	return 0;
}

int pe_log_open(const char* name)
{
	char path[600];
	FILE* f;
	if (n_streams >= PE_LOG_MAX_STREAMS) {
		return -1;
	}
	snprintf(path, sizeof(path), "%s/%s", run_dir_path, name);
	f = fopen(path, "w");
	if (f == NULL) {
		return -1;
	}
	setvbuf(f, NULL, _IOFBF, 64 * 1024);
	pthread_mutex_lock(&r_lock);
	files[n_streams] = f;
	n_streams++;
	pthread_mutex_unlock(&r_lock);
	return n_streams - 1;
}

void pe_log_write(int stream, const char* text, size_t len)
{
	unsigned char hdr[REC_HDR];
	if (stream < 0 || !running) {
		return;
	}
	if (len > REC_MAX) {
		len = REC_MAX;
	}
	hdr[0] = (unsigned char)stream;
	hdr[1] = (unsigned char)(len & 0xFFu);
	hdr[2] = (unsigned char)(len >> 8);
	pthread_mutex_lock(&r_lock);
	while (blocking && running && RING_BYTES - used_bytes() < REC_HDR + (uint32_t)len) {
		pthread_cond_wait(&r_space, &r_lock);
	}
	if (RING_BYTES - used_bytes() < REC_HDR + (uint32_t)len) {
		drops++;
	}
	else {
		ring_put(hdr, REC_HDR);
		ring_put((const unsigned char*)text, (uint32_t)len);
		pthread_cond_signal(&r_cond);
	}
	pthread_mutex_unlock(&r_lock);
}

void pe_log_printf(int stream, const char* fmt, ...)
{
	char buf[REC_MAX];
	int n;
	va_list ap;
	va_start(ap, fmt);
	n = vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	if (n > 0) {
		pe_log_write(stream, buf, (size_t)n < sizeof(buf) ? (size_t)n : sizeof(buf) - 1);
	}
}

void pe_log_set_blocking(int on)
{
	pthread_mutex_lock(&r_lock);
	blocking = on;
	pthread_mutex_unlock(&r_lock);
}

uint32_t pe_log_drops(void)
{
	uint32_t d;
	pthread_mutex_lock(&r_lock);
	d = drops;
	pthread_mutex_unlock(&r_lock);
	return d;
}

void pe_log_stop(void)
{
	int i;
	if (!running) {
		return;
	}
	pthread_mutex_lock(&r_lock);
	running = 0;
	pthread_cond_signal(&r_cond);
	pthread_cond_broadcast(&r_space);
	pthread_mutex_unlock(&r_lock);
	pthread_join(writer, NULL);
	for (i = 0; i < n_streams; i++) {
		if (files[i] != NULL) {
			fflush(files[i]);
			fsync(fileno(files[i]));
			fclose(files[i]);
			files[i] = NULL;
		}
	}
}
