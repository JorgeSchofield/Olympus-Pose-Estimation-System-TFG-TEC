#include <CMAES.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>

// MAES_Queue: single-slot blocking queue, the pthreads equivalent of the
// FreeRTOS call used everywhere in this library, xQueueCreate(1, sizeof(MsgObj)).
// Both send and receive can block up to a timeout (in milliseconds), or
// forever when timeout_ms == MAES_MAX_DELAY, or return immediately when
// timeout_ms == 0 - matching xQueueSend/xQueueReceive semantics.
struct MAES_Queue {
	pthread_mutex_t lock;
	pthread_cond_t not_empty;
	pthread_cond_t not_full;
	MsgObj slot;
	bool full;
};

// Builds an absolute deadline usable by pthread_cond_timedwait from a
// relative timeout in milliseconds. The condition variables are bound to
// CLOCK_MONOTONIC (see MAES_QueueCreate), so the deadline must use the same
// clock. CLOCK_REALTIME would jump when the wall clock is set at boot (the
// RPi 5 has no RTC battery by default) or by NTP, stretching or cutting short
// every pending timeout.
static void deadline_from_timeout(struct timespec* deadline, MAESTickType_t timeout_ms) {
	clock_gettime(CLOCK_MONOTONIC, deadline);
	deadline->tv_sec += timeout_ms / 1000;
	deadline->tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
	if (deadline->tv_nsec >= 1000000000L) {
		deadline->tv_sec += 1;
		deadline->tv_nsec -= 1000000000L;
	}
}

MAES_Queue* MAES_QueueCreate(void) {
	MAES_Queue* q = (MAES_Queue*)malloc(sizeof(MAES_Queue));
	if (q == NULL) {
		return NULL;
	}
	pthread_mutex_init(&q->lock, NULL);
	pthread_condattr_t attr;
	pthread_condattr_init(&attr);
	pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
	pthread_cond_init(&q->not_empty, &attr);
	pthread_cond_init(&q->not_full, &attr);
	pthread_condattr_destroy(&attr);
	q->full = false;
	return q;
}

// Cleanup handler used so a cancelled thread (see kill_agent in
// Agent_Platform.c) never leaves the mailbox mutex locked behind it.
static void unlock_mutex_cleanup(void* arg) {
	pthread_mutex_unlock((pthread_mutex_t*)arg);
}

bool MAES_QueueSend(MAES_Queue* q, const MsgObj* msg, MAESTickType_t timeout_ms) {
	if (q == NULL) {
		return false;
	}

	struct timespec deadline;
	bool has_deadline = (timeout_ms != MAES_MAX_DELAY);
	if (has_deadline) {
		deadline_from_timeout(&deadline, timeout_ms);
	}

	bool ok = true;

	// NOTE: pthread_cleanup_push/pop expand (on glibc) to an opening/closing
	// brace pair, so any variable used both inside and after this block -
	// like ok - must be declared before push, not between push and pop.
	pthread_mutex_lock(&q->lock);
	pthread_cleanup_push(unlock_mutex_cleanup, &q->lock);

	while (q->full) {
		if (timeout_ms == 0) {
			ok = false;
			break;
		}
		else if (has_deadline) {
			if (pthread_cond_timedwait(&q->not_full, &q->lock, &deadline) == ETIMEDOUT) {
				ok = false;
				break;
			}
		}
		else {
			pthread_cond_wait(&q->not_full, &q->lock);
		}
	}

	if (ok) {
		q->slot = *msg;
		q->full = true;
		pthread_cond_signal(&q->not_empty);
	}

	pthread_cleanup_pop(1); // unlocks q->lock
	return ok;
}

bool MAES_QueueReceive(MAES_Queue* q, MsgObj* out, MAESTickType_t timeout_ms) {
	if (q == NULL) {
		return false;
	}

	struct timespec deadline;
	bool has_deadline = (timeout_ms != MAES_MAX_DELAY);
	if (has_deadline) {
		deadline_from_timeout(&deadline, timeout_ms);
	}

	bool ok = true;

	pthread_mutex_lock(&q->lock);
	pthread_cleanup_push(unlock_mutex_cleanup, &q->lock);

	while (!q->full) {
		if (timeout_ms == 0) {
			ok = false;
			break;
		}
		else if (has_deadline) {
			if (pthread_cond_timedwait(&q->not_empty, &q->lock, &deadline) == ETIMEDOUT) {
				ok = false;
				break;
			}
		}
		else {
			pthread_cond_wait(&q->not_empty, &q->lock);
		}
	}

	if (ok) {
		*out = q->slot;
		q->full = false;
		pthread_cond_signal(&q->not_full);
	}

	pthread_cleanup_pop(1); // unlocks q->lock
	return ok;
}

void MAES_QueueDelete(MAES_Queue* q) {
	if (q == NULL) {
		return;
	}
	pthread_mutex_destroy(&q->lock);
	pthread_cond_destroy(&q->not_empty);
	pthread_cond_destroy(&q->not_full);
	free(q);
}

Agent_AID MAES_GetCurrentTaskHandle(void) {
	return pthread_self();
}
