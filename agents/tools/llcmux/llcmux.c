/*
 * llcmux.c - LLC link multiplexer (DESIGN.md §6, decisions D1, D8, D13).
 *
 * Owns the Arduino Mega serial port and shares it:
 *
 *   real port  <->  pty A  (/dev/arduino_mega): the LLC stream WITHOUT the
 *                    RAW: lines, for the unmodified olympus_hlc; its commands go
 *                    back to the LLC unchanged. olympus_hlc reads one line per
 *                    cycle, so RAW: lines at 50 Hz would bury its TLM lines.
 *              ->  pty B  (/run/olympus/llc_raw): RAW: and TLM: lines prefixed
 *                    with "@<t_rx_ns> ", the arrival time on CLOCK_MONOTONIC,
 *                    for the olympus-pose acquisition agent.
 *              ->  optional record (every line, same "@t " format), which
 *                    olympus-pose can replay with -r.
 *
 * Never blocks on a consumer: writes to the ptys are non-blocking and a full
 * pty drops (and counts) instead of stalling the serial port. When pty A is
 * opened, DTR is pulsed on the real port so the Arduino resets exactly as it
 * did when olympus_hlc opened the port itself.
 *
 *   llcmux [--hw /dev/arduino_mega_hw] [--baud 115200]
 *          [--pty-link /dev/arduino_mega] [--raw-link /run/olympus/llc_raw]
 *          [--record-dir DIR] [--no-dtr-reset] [--prio 45] [--cpu 3]
 *          [--stats-every 10]
 *
 * If the fail-safe matters: when llcmux dies, olympus_hlc stops sending PING
 * and the LLC watchdog stops the motors within 2 s.
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "line_buf.h"
#include "port.h"
#include "stats.h"

/* ------------------------------------------------------------- options */
static const char* hw_path = "/dev/arduino_mega_hw";
static int         baud = 115200;
static const char* pty_link = "/dev/arduino_mega";
static const char* raw_link = "/run/olympus/llc_raw";
static const char* record_dir = NULL;
static int         dtr_reset = 1;
static int         prio = 45;
static int         cpu = 3;
static int         stats_every = 10;

/* --------------------------------------------------------------- state */
typedef struct {
	int  master;
	char slave[64];
	int  open;           /* a process has the slave side open */
} pty_t;

static int       hw = -1;
static pty_t     pa, pb;
static int       rec = -1;
static volatile sig_atomic_t stop;

/* filter for pty A: hold the first 4 bytes of each line to decide */
static char      hold[4];
static int       hold_len;
static enum { F_HOLD, F_PASS, F_DROP } fstate = F_HOLD;

static pe_line_buf_t lb;
static pe_hist_t     raw_int;
static uint64_t      last_raw_ns;
static uint64_t      now_ns;

static struct {
	unsigned long long bytes_in, bytes_a, bytes_cmd;
	unsigned long raw_lines, tlm_lines, other_lines, raw_filtered;
	unsigned long drop_a, drop_b, hw_reopen, dtr_pulses, overflow;
} st;

/* ------------------------------------------------------------- helpers */
static void on_signal(int s)
{
	(void)s;
	stop = 1;
}

static speed_t baud_const(int b)
{
	switch (b) {
	case 9600: return B9600;
	case 57600: return B57600;
	default: return B115200;
	}
}

static int open_hw(void)
{
	struct termios t;
	int fd = open(hw_path, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
	if (fd < 0) {
		return -1;
	}
	if (tcgetattr(fd, &t) == 0) {
		cfmakeraw(&t);
		t.c_cflag |= CLOCAL | CREAD;
		t.c_cflag &= ~(tcflag_t)(CSTOPB | PARENB | CRTSCTS);
		cfsetispeed(&t, baud_const(baud));
		cfsetospeed(&t, baud_const(baud));
		tcsetattr(fd, TCSANOW, &t);
	}
	return fd;
}

static int make_pty(pty_t* p, const char* link)
{
	struct termios t;
	int s;
	const char* name;
	p->master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
	if (p->master < 0 || grantpt(p->master) != 0 || unlockpt(p->master) != 0) {
		return -1;
	}
	name = ptsname(p->master);
	if (name == NULL) {
		return -1;
	}
	snprintf(p->slave, sizeof(p->slave), "%s", name);
	chmod(p->slave, 0666);
	s = open(p->slave, O_RDWR | O_NOCTTY | O_CLOEXEC);   /* set raw once */
	if (s >= 0) {
		if (tcgetattr(s, &t) == 0) {
			cfmakeraw(&t);
			tcsetattr(s, TCSANOW, &t);
		}
		close(s);
	}
	unlink(link);
	if (symlink(p->slave, link) != 0) {
		fprintf(stderr, "llcmux: cannot create %s -> %s: %s\n", link, p->slave, strerror(errno));
		return -1;
	}
	p->open = 0;
	return 0;
}

/* Linux reports POLLHUP on a pty master while no process has the slave open. */
static int slave_is_open(const pty_t* p)
{
	struct pollfd q;
	q.fd = p->master;
	q.events = 0;
	q.revents = 0;
	poll(&q, 1, 0);
	return (q.revents & POLLHUP) == 0;
}

static void write_nb(pty_t* p, const char* data, size_t n, unsigned long* drops)
{
	ssize_t w;
	if (!p->open || n == 0) {
		return;
	}
	w = write(p->master, data, n);
	if (w != (ssize_t)n) {
		(*drops)++;
	}
}

static void pulse_dtr(void)
{
	int bit = TIOCM_DTR;
	struct timespec ts = { 0, 100000000L };
	if (hw < 0) {
		return;
	}
	ioctl(hw, TIOCMBIC, &bit);
	nanosleep(&ts, NULL);
	ioctl(hw, TIOCMBIS, &bit);
	st.dtr_pulses++;
}

/* ------------------------------------------------------ line handling */
static void on_line(void* ctx, const char* line, size_t len)
{
	char out[PE_LINE_MAX + 32];
	int n;
	(void)ctx;
	n = snprintf(out, sizeof(out), "@%llu %.*s\n", (unsigned long long)now_ns, (int)len, line);
	if (n <= 0) {
		return;
	}
	if (len >= 4 && memcmp(line, "RAW:", 4) == 0) {
		st.raw_lines++;
		if (last_raw_ns != 0) {
			pe_hist_add(&raw_int, (double)(now_ns - last_raw_ns) * 1e-6);
		}
		last_raw_ns = now_ns;
		write_nb(&pb, out, (size_t)n, &st.drop_b);
	}
	else if (len >= 4 && memcmp(line, "TLM:", 4) == 0) {
		st.tlm_lines++;
		write_nb(&pb, out, (size_t)n, &st.drop_b);
	}
	else {
		st.other_lines++;
	}
	if (rec >= 0) {
		pe_log_write(rec, out, (size_t)n);
	}
}

/* Bytes for pty A: everything except whole lines that start with "RAW:".
 * Only the first 4 bytes of a line are held back (< 0.35 ms at 115200). */
static size_t filter_a(const char* in, size_t n, char* out)
{
	size_t i, o = 0;
	for (i = 0; i < n; i++) {
		char c = in[i];
		switch (fstate) {
		case F_HOLD:
			if (c == '\n') {
				memcpy(out + o, hold, (size_t)hold_len);
				o += (size_t)hold_len;
				out[o++] = c;
				hold_len = 0;
				break;
			}
			hold[hold_len++] = c;
			if (hold_len == 4) {
				if (memcmp(hold, "RAW:", 4) == 0) {
					fstate = F_DROP;
					st.raw_filtered++;
				}
				else {
					memcpy(out + o, hold, 4);
					o += 4;
					fstate = F_PASS;
				}
				hold_len = 0;
			}
			break;
		case F_PASS:
			out[o++] = c;
			if (c == '\n') {
				fstate = F_HOLD;
			}
			break;
		case F_DROP:
			if (c == '\n') {
				fstate = F_HOLD;
			}
			break;
		}
	}
	return o;
}

static void print_stats(void)
{
	fprintf(stderr,
		"llcmux: hw %s | in %llu B, to A %llu B, cmds %llu B | RAW %lu (filtered %lu) TLM %lu other %lu | "
		"RAW interval p50 %.1f p99 %.1f max %.1f ms (%.1f Hz) | drops A %lu B %lu | A %s, B %s | "
		"reopen %lu, DTR %lu, long lines %lu\n",
		hw >= 0 ? "up" : "DOWN", st.bytes_in, st.bytes_a, st.bytes_cmd,
		st.raw_lines, st.raw_filtered, st.tlm_lines, st.other_lines,
		pe_hist_percentile(&raw_int, 50), pe_hist_percentile(&raw_int, 99), raw_int.max_ms,
		raw_int.count ? 1000.0 * raw_int.count / raw_int.sum_ms : 0.0,
		st.drop_a, st.drop_b, pa.open ? "open" : "closed", pb.open ? "open" : "closed",
		st.hw_reopen, st.dtr_pulses, (unsigned long)lb.n_overflow);
}

static void set_rt(void)
{
	struct sched_param sp;
	cpu_set_t cs;
	sp.sched_priority = prio;
	if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
		fprintf(stderr, "llcmux: warning - SCHED_FIFO %d not available (not root?)\n", prio);
	}
	if (cpu >= 0) {
		CPU_ZERO(&cs);
		CPU_SET(cpu, &cs);
		sched_setaffinity(0, sizeof(cs), &cs);
	}
}

static void usage(void)
{
	fprintf(stderr, "usage: llcmux [--hw DEV] [--baud N] [--pty-link PATH] [--raw-link PATH]\n"
		"              [--record-dir DIR] [--no-dtr-reset] [--prio N] [--cpu N] [--stats-every S]\n");
}

int main(int argc, char** argv)
{
	int i;
	uint64_t next_open = 0, next_stats;
	char rundir[512];

	for (i = 1; i < argc; i++) {
		const char* a = argv[i];
		const char* v = i + 1 < argc ? argv[i + 1] : NULL;
		if (strcmp(a, "--hw") == 0 && v) { hw_path = v; i++; }
		else if (strcmp(a, "--baud") == 0 && v) { baud = atoi(v); i++; }
		else if (strcmp(a, "--pty-link") == 0 && v) { pty_link = v; i++; }
		else if (strcmp(a, "--raw-link") == 0 && v) { raw_link = v; i++; }
		else if (strcmp(a, "--record-dir") == 0 && v) { record_dir = v; i++; }
		else if (strcmp(a, "--no-dtr-reset") == 0) { dtr_reset = 0; }
		else if (strcmp(a, "--prio") == 0 && v) { prio = atoi(v); i++; }
		else if (strcmp(a, "--cpu") == 0 && v) { cpu = atoi(v); i++; }
		else if (strcmp(a, "--stats-every") == 0 && v) { stats_every = atoi(v); i++; }
		else { usage(); return 2; }
	}

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	signal(SIGPIPE, SIG_IGN);
	set_rt();
	mkdir("/run/olympus", 0755);
	if (make_pty(&pa, pty_link) != 0 || make_pty(&pb, raw_link) != 0) {
		fprintf(stderr, "llcmux: cannot create the pseudo-terminals\n");
		return 1;
	}
	if (record_dir != NULL) {
		if (pe_log_start(record_dir, rundir, sizeof(rundir)) == 0) {
			rec = pe_log_open("llc_record.txt");
			fprintf(stderr, "llcmux: recording to %s/llc_record.txt\n", rundir);
		}
	}
	pe_line_buf_reset(&lb);
	pe_hist_reset(&raw_int);
	fprintf(stderr, "llcmux: %s <-> %s (olympus_hlc, RAW filtered) and -> %s (pose app)\n",
		hw_path, pty_link, raw_link);
	next_stats = pe_now_ns() + (uint64_t)stats_every * 1000000000ull;

	while (!stop) {
		struct pollfd q[3];
		int nq = 0, ia = -1, ih = -1, r;
		now_ns = pe_now_ns();

		if (hw < 0 && now_ns >= next_open) {
			hw = open_hw();
			if (hw >= 0) {
				fprintf(stderr, "llcmux: opened %s\n", hw_path);
				pe_line_buf_reset(&lb);
				fstate = F_HOLD;
				hold_len = 0;
				st.hw_reopen++;
			}
			else {
				next_open = now_ns + 500000000ull;
			}
		}

		/* consumers appearing / disappearing */
		{
			int a_now = slave_is_open(&pa), b_now = slave_is_open(&pb);
			if (a_now && !pa.open) {
				fprintf(stderr, "llcmux: %s opened by a client\n", pty_link);
				if (dtr_reset) {
					pulse_dtr();   /* the Arduino resets, as when the real port is opened */
				}
			}
			pa.open = a_now;
			pb.open = b_now;
		}

		if (hw >= 0) {
			q[nq].fd = hw;
			q[nq].events = POLLIN;
			ih = nq++;
		}
		if (pa.open) {
			q[nq].fd = pa.master;
			q[nq].events = POLLIN;
			ia = nq++;
		}
		r = poll(q, (nfds_t)nq, 200);
		if (r < 0) {
			continue;
		}
		now_ns = pe_now_ns();

		if (ih >= 0 && q[ih].revents) {
			char buf[512], outa[512];
			ssize_t n = read(hw, buf, sizeof(buf));
			if (n > 0) {
				size_t na;
				st.bytes_in += (unsigned long long)n;
				na = filter_a(buf, (size_t)n, outa);
				st.bytes_a += na;
				write_nb(&pa, outa, na, &st.drop_a);
				pe_line_buf_feed(&lb, buf, (size_t)n, on_line, NULL);
			}
			else if (n == 0 || (errno != EAGAIN && errno != EINTR)) {
				fprintf(stderr, "llcmux: %s lost, reopening\n", hw_path);
				close(hw);
				hw = -1;
				next_open = now_ns + 500000000ull;
			}
		}
		if (ia >= 0 && (q[ia].revents & POLLIN)) {
			char buf[256];
			ssize_t n = read(pa.master, buf, sizeof(buf));
			if (n > 0 && hw >= 0) {
				ssize_t w = write(hw, buf, (size_t)n);   /* commands from olympus_hlc */
				if (w > 0) {
					st.bytes_cmd += (unsigned long long)w;
				}
			}
		}
		if (stats_every > 0 && now_ns >= next_stats) {
			next_stats = now_ns + (uint64_t)stats_every * 1000000000ull;
			print_stats();
		}
	}

	print_stats();
	unlink(pty_link);
	unlink(raw_link);
	pe_log_stop();
	return 0;
}
