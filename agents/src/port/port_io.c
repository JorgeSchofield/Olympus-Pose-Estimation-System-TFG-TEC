/*
 * port_io.c - Time, byte streams, UDP and process helpers for Linux.
 */
#include "port.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

typedef char pe_udp_addr_fits[(sizeof(struct sockaddr_in) <= sizeof(((pe_udp_dest_t*)0)->addr)) ? 1 : -1];

uint64_t pe_now_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000u + (uint64_t)ts.tv_nsec;
}

static speed_t baud_const(int baud)
{
	switch (baud) {
	case 4800: return B4800;
	case 9600: return B9600;
	case 19200: return B19200;
	case 38400: return B38400;
	case 57600: return B57600;
	case 230400: return B230400;
	default: return B115200;
	}
}

int pe_stream_open(const char* path, int baud)
{
	struct stat st;
	int fd;
	if (stat(path, &st) != 0) {
		return -1;
	}
	if (S_ISREG(st.st_mode)) {
		return open(path, O_RDONLY | O_CLOEXEC);
	}
	fd = open(path, O_RDWR | O_NOCTTY | O_CLOEXEC);
	if (fd < 0) {
		return -1;
	}
	if (isatty(fd)) {
		struct termios t;
		if (tcgetattr(fd, &t) == 0) {
			cfmakeraw(&t);
			t.c_cflag |= CLOCAL | CREAD;
			t.c_cflag &= ~(tcflag_t)(CSTOPB | PARENB | CRTSCTS);
			/* read() is only called after poll() reports data; VMIN 1 keeps
			 * a return of 0 meaning end of stream (hang-up), not "nothing". */
			t.c_cc[VMIN] = 1;
			t.c_cc[VTIME] = 0;
			cfsetispeed(&t, baud_const(baud));
			cfsetospeed(&t, baud_const(baud));
			tcsetattr(fd, TCSANOW, &t);
		}
	}
	return fd;
}

int pe_stream_wait(int fd, int timeout_ms)
{
	struct pollfd p;
	int r;
	p.fd = fd;
	p.events = POLLIN;
	p.revents = 0;
	r = poll(&p, 1, timeout_ms);
	if (r < 0) {
		return errno == EINTR ? 0 : -1;
	}
	if (r == 0) {
		return 0;
	}
	if (p.revents & POLLIN) {
		return 1;
	}
	return -1;   /* POLLHUP / POLLERR without data */
}

long pe_stream_read(int fd, char* buf, size_t n)
{
	ssize_t r;
	do {
		r = read(fd, buf, n);
	} while (r < 0 && errno == EINTR);
	if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
		return -2;   /* nothing right now */
	}
	return (long)r;
}

void pe_stream_close(int fd)
{
	if (fd >= 0) {
		close(fd);
	}
}

int pe_udp_sender(pe_udp_dest_t* d, const char* host, int port)
{
	struct sockaddr_in a;
	memset(d, 0, sizeof(*d));
	d->fd = -1;
	memset(&a, 0, sizeof(a));
	a.sin_family = AF_INET;
	a.sin_port = htons((uint16_t)port);
	if (inet_pton(AF_INET, host, &a.sin_addr) != 1) {
		return -1;
	}
	d->fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
	if (d->fd < 0) {
		return -1;
	}
	memcpy(d->addr, &a, sizeof(a));
	return 0;
}

int pe_udp_send(const pe_udp_dest_t* d, const void* data, size_t n)
{
	if (d->fd < 0) {
		return -1;
	}
	return sendto(d->fd, data, n, MSG_DONTWAIT, (const struct sockaddr*)d->addr,
		sizeof(struct sockaddr_in)) == (ssize_t)n ? 0 : -1;
}

int pe_udp_listener(int port)
{
	struct sockaddr_in a;
	int fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
	if (fd < 0) {
		return -1;
	}
	memset(&a, 0, sizeof(a));
	a.sin_family = AF_INET;
	a.sin_port = htons((uint16_t)port);
	a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	if (bind(fd, (const struct sockaddr*)&a, sizeof(a)) != 0) {
		close(fd);
		return -1;
	}
	return fd;
}

long pe_udp_recv(int fd, void* buf, size_t n, pe_udp_dest_t* from)
{
	struct sockaddr_in a;
	socklen_t al = sizeof(a);
	ssize_t r = recvfrom(fd, buf, n, MSG_DONTWAIT, (struct sockaddr*)&a, &al);
	if (r <= 0) {
		return 0;
	}
	if (from != NULL) {
		from->fd = fd;
		memcpy(from->addr, &a, sizeof(a));
	}
	return (long)r;
}

int pe_udp_reply(int fd, const pe_udp_dest_t* to, const void* data, size_t n)
{
	return sendto(fd, data, n, MSG_DONTWAIT, (const struct sockaddr*)to->addr,
		sizeof(struct sockaddr_in)) == (ssize_t)n ? 0 : -1;
}

int pe_lock_memory(void)
{
	return mlockall(MCL_CURRENT | MCL_FUTURE);
}

long pe_rss_kb(void)
{
	long pages_total = 0, pages_resident = -1;
	FILE* f = fopen("/proc/self/statm", "r");
	if (f == NULL) {
		return -1;
	}
	if (fscanf(f, "%ld %ld", &pages_total, &pages_resident) != 2) {
		pages_resident = -1;
	}
	fclose(f);
	return pages_resident < 0 ? -1 : pages_resident * (sysconf(_SC_PAGESIZE) / 1024);
}

double pe_cpu_seconds(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
	return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}
