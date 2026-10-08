/*
 * port.h - Linux portability layer of the application (DESIGN.md D12).
 *
 * Everything outside this folder and the CMAES library is plain C99 with no
 * OS calls (PE-RNF-007). Only these functions touch POSIX/Linux.
 */
#ifndef PE_PORT_H
#define PE_PORT_H

#include <stddef.h>
#include <stdint.h>

/* ---- time --------------------------------------------------------------- */
uint64_t pe_now_ns(void);                        /* CLOCK_MONOTONIC */

/* ---- byte-stream input (serial port, pty or replay file) ---------------- */
/* Opens path for reading (and writing if it is a tty). A tty is set to raw
 * 8N1 at baud; a regular file is read as is. Returns an fd or -1. */
int  pe_stream_open(const char* path, int baud);
/* Waits up to timeout_ms for data. 1 = readable, 0 = timeout,
 * -1 = error or hang-up (reopen). */
int  pe_stream_wait(int fd, int timeout_ms);
/* Reads what is available. >0 bytes, 0 = end of file, -1 = error. */
long pe_stream_read(int fd, char* buf, size_t n);
void pe_stream_close(int fd);

/* ---- UDP ---------------------------------------------------------------- */
typedef struct {
	int  fd;
	char addr[32];       /* opaque sockaddr_in storage */
} pe_udp_dest_t;

int  pe_udp_sender(pe_udp_dest_t* d, const char* host, int port);   /* 0 ok, -1 error */
int  pe_udp_send(const pe_udp_dest_t* d, const void* data, size_t n); /* non-blocking */
int  pe_udp_listener(int port);                                       /* fd, non-blocking, 127.0.0.1 */
/* Receives one datagram if available: >0 bytes, 0 none. Fills a reply
 * address so pe_udp_reply can answer. */
long pe_udp_recv(int fd, void* buf, size_t n, pe_udp_dest_t* from);
int  pe_udp_reply(int fd, const pe_udp_dest_t* to, const void* data, size_t n);

/* ---- process ------------------------------------------------------------ */
int  pe_lock_memory(void);          /* mlockall(MCL_CURRENT | MCL_FUTURE) */
long pe_rss_kb(void);               /* resident set size, -1 if unknown */
double pe_cpu_seconds(void);        /* process CPU time */

/* ---- asynchronous log writer (DESIGN.md §11.2) ---------------------------
 * Agents never block on storage: pe_log_write copies the text into a ring
 * buffer and a low-priority thread writes it. If the ring is full the record
 * is dropped and counted. */
#define PE_LOG_MAX_STREAMS 8

/* Creates <base>/<YYYYmmdd-HHMMSS>/ and starts the writer thread.
 * run_dir receives the path. Returns 0 or -1. */
int  pe_log_start(const char* base, char* run_dir, size_t run_dir_len);
/* Opens <run_dir>/<name> and returns a stream id (>= 0) or -1. */
int  pe_log_open(const char* name);
void pe_log_write(int stream, const char* text, size_t len);
void pe_log_printf(int stream, const char* fmt, ...);
/* Replay mode: producers wait for space instead of dropping, so a
 * recorded run is logged completely. Off (drop) in live operation. */
void pe_log_set_blocking(int on);
uint32_t pe_log_drops(void);
/* Drains the ring, flushes and closes every file. */
void pe_log_stop(void);

#endif
