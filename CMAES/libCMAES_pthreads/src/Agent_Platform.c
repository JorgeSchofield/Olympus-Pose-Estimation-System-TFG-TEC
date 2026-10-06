#include <CMAES.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sched.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>

/*
 * pthreads backend notes (see the port plan for the full rationale):
 *
 * - Agent lifecycle: a pthread is created immediately by agent_init, but it
 *   blocks at a "start gate" until register_agent/resume_agent opens it -
 *   this reproduces the FreeRTOS "created suspended" semantics using the
 *   same primitive that later implements suspend/resume.
 * - suspend/resume are COOPERATIVE, not preemptive: a suspended agent keeps
 *   running until it hits a checkpoint (MAES_CheckSuspend, called from the
 *   CyclicBehaviour/OneShotBehaviour execute loop and from agent_wait).
 * - kill uses pthread_cancel + pthread_join (deferred cancellation - it only
 *   fires at cancellation points such as the mailbox's condition wait).
 * - AMS/setup-thread identity for admin permission checks is done by
 *   comparing thread ids directly (not by priority equality, as FreeRTOS
 *   does), because real-time priorities are best-effort under Linux and
 *   cannot be relied on as a secure identity check when unprivileged.
 * - Agent_resources.stackSize is kept for API compatibility but not applied;
 *   pthreads' default stack (commonly 8 MiB) is used for every agent.
 */

typedef struct {
	Agent_AID aid;
	pthread_mutex_t lock;
	pthread_cond_t cond;
	bool suspended;      // start gate (before first register) AND pause flag
	AGENT_MODE mode;
	bool in_use;
} MAES_AgentControl;

typedef struct {
	void (*real_fn)(void*);
	void* real_arg;
	MAES_AgentControl* ctrl;
} MAES_TrampolineArgs;

static MAES_AgentControl control_table[AGENT_LIST_SIZE];
static pthread_mutex_t control_table_lock = PTHREAD_MUTEX_INITIALIZER;

static MAES_AgentControl* control_alloc(void) {
	pthread_mutex_lock(&control_table_lock);
	MAES_AgentControl* ctrl = NULL;
	for (int i = 0; i < AGENT_LIST_SIZE; i++) {
		if (!control_table[i].in_use) {
			ctrl = &control_table[i];
			ctrl->in_use = true;
			ctrl->aid = 0;
			ctrl->suspended = true;
			ctrl->mode = SUSPENDED;
			pthread_mutex_init(&ctrl->lock, NULL);
			pthread_cond_init(&ctrl->cond, NULL);
			break;
		}
	}
	pthread_mutex_unlock(&control_table_lock);
	return ctrl;
}

static void control_bind(MAES_AgentControl* ctrl, Agent_AID aid) {
	pthread_mutex_lock(&control_table_lock);
	ctrl->aid = aid;
	pthread_mutex_unlock(&control_table_lock);
}

static MAES_AgentControl* control_find(Agent_AID aid) {
	pthread_mutex_lock(&control_table_lock);
	MAES_AgentControl* found = NULL;
	for (int i = 0; i < AGENT_LIST_SIZE; i++) {
		if (control_table[i].in_use && control_table[i].aid == aid) {
			found = &control_table[i];
			break;
		}
	}
	pthread_mutex_unlock(&control_table_lock);
	return found;
}

static void control_free(MAES_AgentControl* ctrl) {
	if (ctrl == NULL) {
		return;
	}
	pthread_mutex_destroy(&ctrl->lock);
	pthread_cond_destroy(&ctrl->cond);
	pthread_mutex_lock(&control_table_lock);
	ctrl->in_use = false;
	pthread_mutex_unlock(&control_table_lock);
}

static void unlock_mutex_cleanup(void* arg) {
	pthread_mutex_unlock((pthread_mutex_t*)arg);
}

// Shared by the trampoline's start gate and by MAES_CheckSuspend's
// mid-execution checkpoint - both are "block while ctrl->suspended".
static void control_wait_if_suspended(MAES_AgentControl* ctrl) {
	pthread_mutex_lock(&ctrl->lock);
	pthread_cleanup_push(unlock_mutex_cleanup, &ctrl->lock);
	while (ctrl->suspended) {
		pthread_cond_wait(&ctrl->cond, &ctrl->lock);
	}
	ctrl->mode = ACTIVE;
	pthread_cleanup_pop(1); // unlocks ctrl->lock
}

static void control_release(MAES_AgentControl* ctrl) {
	pthread_mutex_lock(&ctrl->lock);
	ctrl->suspended = false;
	ctrl->mode = ACTIVE;
	pthread_cond_broadcast(&ctrl->cond);
	pthread_mutex_unlock(&ctrl->lock);
}

static void control_pause(MAES_AgentControl* ctrl) {
	pthread_mutex_lock(&ctrl->lock);
	ctrl->suspended = true;
	ctrl->mode = SUSPENDED;
	pthread_mutex_unlock(&ctrl->lock);
}

void MAES_CheckSuspend(Agent_AID aid) {
	MAES_AgentControl* ctrl = control_find(aid);
	if (ctrl != NULL) {
		control_wait_if_suspended(ctrl);
	}
}

static void* MAES_pthread_trampoline(void* raw_args) {
	MAES_TrampolineArgs* targs = (MAES_TrampolineArgs*)raw_args;
	void (*real_fn)(void*) = targs->real_fn;
	void* real_arg = targs->real_arg;
	MAES_AgentControl* ctrl = targs->ctrl;
	free(targs);

	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
	pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);

	control_wait_if_suspended(ctrl); // start gate: opened by register_agent

	real_fn(real_arg);
	return NULL;
}

// Best-effort real-time priority mapping. Unprivileged processes on
// Raspberry Pi OS typically lack CAP_SYS_NICE, so failures here are
// reported once and otherwise ignored rather than treated as fatal -
// agent priorities become advisory (SCHED_OTHER) in that case.
static void MAES_SetPriority(Agent_AID aid, int rt_priority) {
	struct sched_param sp;
	sp.sched_priority = rt_priority;
	if (pthread_setschedparam(aid, SCHED_FIFO, &sp) != 0) {
		static bool warned = false;
		if (!warned) {
			fprintf(stderr, "CMAES: warning - insufficient privilege for SCHED_FIFO; "
				"agent priorities are advisory only on this system\n");
			warned = true;
		}
	}
}

// AMS real-time priority. Default 46: below the kernel's threaded IRQ handlers
// (SCHED_FIFO 50 on Linux), so the USB/serial interrupt threads that deliver
// sensor data are never starved by the agent platform. Override with
// MAES_SetAMSPriority() before boot().
static int ams_rt_priority = MAES_DEFAULT_AMS_PRIORITY;

void MAES_SetAMSPriority(int rt_priority) {
	ams_rt_priority = rt_priority;
}

static int MAES_AMSRTPrio(void) {
	int min = sched_get_priority_min(SCHED_FIFO);
	int max = sched_get_priority_max(SCHED_FIFO);
	if (min < 0 || max < 0) {
		return 0;
	}
	int p = ams_rt_priority;
	if (p > max) {
		p = max;
	}
	if (p < min + 1) { // leave at least one level below it for the agents
		p = min + 1;
	}
	return p;
}

// Agents map their ordinal MAES priority onto SCHED_FIFO as min + priority,
// always kept strictly below the AMS.
static int MAES_RTPrioFor(MAESUBaseType_t maesPriority) {
	int min = sched_get_priority_min(SCHED_FIFO);
	if (min < 0) {
		return 0;
	}
	int ceiling = MAES_AMSRTPrio() - 1;
	int p = min + (int)maesPriority;
	if (p > ceiling) {
		p = ceiling;
	}
	return p;
}

// CPU affinity (no FreeRTOS equivalent). cpu < 0 leaves the affinity alone.
bool MAES_SetAffinity(Agent_AID aid, int cpu) {
	if (cpu < 0) {
		return true;
	}
	cpu_set_t set;
	CPU_ZERO(&set);
	CPU_SET(cpu, &set);
	if (pthread_setaffinity_np(aid, sizeof(set), &set) != 0) {
		static bool warned = false;
		if (!warned) {
			fprintf(stderr, "CMAES: warning - could not pin a thread to CPU %d\n", cpu);
			warned = true;
		}
		return false;
	}
	return true;
}

// Milliseconds on CLOCK_MONOTONIC, truncated to 32 bits like an RTOS tick
// counter (wraps after ~49.7 days; use differences, never absolute values).
MAESTickType_t MAES_GetTickCount(void) {
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	return (MAESTickType_t)((uint64_t)now.tv_sec * 1000u + (uint64_t)now.tv_nsec / 1000000u);
}

// Periodic wait on an absolute deadline, the equivalent of FreeRTOS
// vTaskDelayUntil(): sleeps until *last_wake_ms + period_ms and advances
// *last_wake_ms by exactly one period, so the loop period does not drift with
// the time spent working. Unlike vTaskDelayUntil it does not catch up after an
// overrun (or after the agent was suspended): when the deadline has already
// passed it returns false at once and re-anchors *last_wake_ms to now, so a
// late agent never fires a burst of back-to-back iterations.
// Initialise *last_wake_ms with MAES_GetTickCount() before the first call.
bool MAES_DelayUntil(MAESTickType_t* last_wake_ms, MAESTickType_t period_ms) {
	MAES_CheckSuspend(MAES_GetCurrentTaskHandle());

	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	uint64_t now64 = (uint64_t)now.tv_sec * 1000u + (uint64_t)now.tv_nsec / 1000000u;
	MAESTickType_t next = *last_wake_ms + period_ms;
	int32_t delta = (int32_t)(next - (MAESTickType_t)now64); // wrap-safe

	if (delta <= 0) {
		*last_wake_ms = (MAESTickType_t)now64;
		return false;
	}

	uint64_t target64 = now64 + (uint64_t)delta;
	struct timespec target;
	target.tv_sec = (time_t)(target64 / 1000u);
	target.tv_nsec = (long)(target64 % 1000u) * 1000000L;
	while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &target, NULL) == EINTR) {
		// interrupted by a signal: sleep again until the same deadline
	}
	*last_wake_ms = next;
	return true;
}

// Identity checks used for admin-command authorization, replacing the
// FreeRTOS version's uxTaskPriorityGet(...) == configMAX_PRIORITIES - 1.
static bool caller_is_ams(Agent_Platform* platform) {
	return pthread_equal(MAES_GetCurrentTaskHandle(), platform->agentAMS.agent.aid) != 0;
}

static bool caller_is_setup_thread(Agent_Platform* platform) {
	return pthread_equal(MAES_GetCurrentTaskHandle(), platform->setup_thread) != 0;
}

//AMS Task Function: This function initiates the AMS Task, which manages the states of the agents.
//Inputs: Parameters passed into the AMS Task Function.
//Outputs: None.
void AMS_taskFunction(void* pvParameters) {
	AMSparameter* amsParameters = (AMSparameter*)pvParameters;
	USER_DEF_COND* cond = amsParameters->cond;
	Agent_Platform* services = amsParameters->services;
	Agent_Msg msg;
	ConstructorAgent_Msg(&msg, amsParameters->ptr_env);
	msg.Agent_Msg(&msg);
	MAESUBaseType_t error_msg = 0;
	for (;;)
	{
		msg.receive(&msg, MAES_MAX_DELAY);
		if (msg.get_msg_type(&msg) == REQUEST)
		{
			if (strcmp(msg.get_msg_content(&msg), "KILL") == 0)
			{
				if (cond->kill_cond())
				{
					error_msg = services->kill_agent(services, msg.get_target_agent(&msg));
					if (error_msg == NO_ERRORS)
					{
						msg.set_msg_type(&msg, CONFIRM);
					}
					else
					{
						msg.set_msg_type(&msg, REFUSE);
					}
				}
				else
				{
					msg.set_msg_type(&msg, REFUSE);
				}
				msg.send(&msg, msg.get_sender(&msg), 0);
			} //KILL Case

			else if (strcmp(msg.get_msg_content(&msg), "REGISTER") == 0)
			{
				if (cond->register_cond())
				{
					error_msg = services->register_agent(services, msg.get_target_agent(&msg));
					if (error_msg == NO_ERRORS)
					{
						msg.set_msg_type(&msg, CONFIRM);
					}
					else
					{
						msg.set_msg_type(&msg, REFUSE);
					}
				}
				else
				{
					msg.set_msg_type(&msg, REFUSE);
				}
				msg.send(&msg, msg.get_sender(&msg), 0);
			} //REGISTER Case

			else if (strcmp(msg.get_msg_content(&msg), "DEREGISTER") == 0)
			{
				if (cond->deregister_cond())
				{
					error_msg = services->deregister_agent(services, msg.get_target_agent(&msg));
					if (error_msg == NO_ERRORS)
					{
						msg.set_msg_type(&msg, CONFIRM);
					}
					else
					{
						msg.set_msg_type(&msg, REFUSE);
					}
				}
				else
				{
					msg.set_msg_type(&msg, REFUSE);
				}
				msg.send(&msg, msg.get_sender(&msg), 0);
			} //DEREGISTER Case

			else if (strcmp(msg.get_msg_content(&msg), "SUSPEND") == 0)
			{
				if (cond->suspend_cond())
				{
					error_msg = services->suspend_agent(services, msg.get_target_agent(&msg));
					if (error_msg == NO_ERRORS)
					{
						msg.set_msg_type(&msg, CONFIRM);
					}
					else
					{
						msg.set_msg_type(&msg, REFUSE);
					}
				}
				else
				{
					msg.set_msg_type(&msg, REFUSE);
				}
				msg.send(&msg, msg.get_sender(&msg), 0);
			} //SUSPEND Case

			else if (strcmp(msg.get_msg_content(&msg), "RESUME") == 0)
			{
				if (cond->resume_cond())
				{
					error_msg = services->resume_agent(services, msg.get_target_agent(&msg));
					if (error_msg == NO_ERRORS)
					{
						msg.set_msg_type(&msg, CONFIRM);
					}
					else
					{
						msg.set_msg_type(&msg, REFUSE);
					}
				}
				else
				{
					msg.set_msg_type(&msg, REFUSE);
				}
				msg.send(&msg, msg.get_sender(&msg), 0);
			} //RESUME Case

			else if (strcmp(msg.get_msg_content(&msg), "RESTART") == 0)
			{
				if (cond->restart_cond())
				{
					services->restart(services, msg.get_target_agent(&msg));
				}
				else
				{
					msg.set_msg_type(&msg, REFUSE);
				}
				msg.send(&msg, msg.get_sender(&msg), 0);
			} //RESTART Case

			else
			{
				msg.set_msg_type(&msg, NOT_UNDERSTOOD);
				msg.send(&msg, msg.get_sender(&msg), 0);
			}
		} //end if
		else
		{
			msg.set_msg_type(&msg, NOT_UNDERSTOOD);
			msg.send(&msg, msg.get_sender(&msg), 0);
		}
	} // end while
};

//Agent Platform Function: This function sets some of the initial values of the parameters needed in the agent platform.
//Inputs: The Platform instance itself and the platform given name.
//Outputs: None.
void Agent_PlatformFunction(Agent_Platform* platform, const char* name) {
	static AMSparameter parameters;
	platform->parameter = &parameters;
	platform->setup_thread = MAES_GetCurrentTaskHandle();
	ConstructorUSER_DEF_COND(&platform->cond);
	platform->agentAMS.agent.agent_name = name;
	platform->description.AP_name = name;
	platform->description.subscribers = 0;
	platform->ptr_cond = &platform->cond;
	platform->agentAMS.agent.priority = 0;
	for (MAESUBaseType_t i = 0; i < AGENT_LIST_SIZE; i++)
	{
		platform->Agent_Handle[i] = (Agent_AID)NULL;
	}
};

//Agent Platform with Conditions Function: This function sets some of the initial values of the parameters needed in the agent platform. In this functin, some conditions are established by the user.
//Inputs: The Platform instance itself, the platform given name and the conditions given by the user.
//Outputs: None.
void Agent_PlatformWithCondFunction(Agent_Platform* platform, const char* name, USER_DEF_COND* user_cond) {
	platform->setup_thread = MAES_GetCurrentTaskHandle();
	platform->agentAMS.agent.agent_name = name;
	platform->ptr_cond = user_cond;
	platform->description.subscribers = 0;
	for (MAESUBaseType_t i = 0; i < AGENT_LIST_SIZE; i++)
	{
		platform->Agent_Handle[i] = (Agent_AID)NULL;
	}
};

// Internal helper shared by agent_init/agent_initConParam/boot: creates the
// mailbox, the control block (start gate closed) and the pthread itself.
static void spawn_agent_thread(MAESAgent* agent, void (*behaviour)(void*), void* pvParameters) {
	agent->agent.mailbox_handle = MAES_QueueCreate();

	MAES_AgentControl* ctrl = control_alloc();

	MAES_TrampolineArgs* targs = (MAES_TrampolineArgs*)malloc(sizeof(MAES_TrampolineArgs));
	targs->real_fn = behaviour;
	targs->real_arg = pvParameters;
	targs->ctrl = ctrl;

	agent->resources.function = behaviour;
	agent->resources.taskParameters = pvParameters;

	pthread_t tid;
	pthread_create(&tid, NULL, MAES_pthread_trampoline, targs);

	agent->agent.aid = tid;
	control_bind(ctrl, tid);
	env.set_TaskEnv(&env, tid, agent);
};

//Boot Function: This function boots the agent platform. Therefore, it register each agent into the platform itself.
//Inputs: The Platform instance itself.
//Outputs: bool variable indicating if the boot was successful.
bool bootFunction(Agent_Platform* platform) {
	ConstructorAgente(&platform->agentAMS);
	platform->agentAMS.Iniciador(&platform->agentAMS, "AMSAgent", MAES_AMSRTPrio(), 1024);
	platform->parameter->cond = platform->ptr_cond;
	platform->parameter->services = platform;
	platform->parameter->ptr_env = &env;

	spawn_agent_thread(&platform->agentAMS, AMS_taskFunction, (void*)platform->parameter);
	platform->description.AMS_AID = platform->agentAMS.agent.aid;

	if (platform->agentAMS.agent.aid != (Agent_AID)0)
	{
		// Register every agent env-entry created so far, exactly like the
		// FreeRTOS version - this includes the AMS's own entry (index 0),
		// since register_agent is what links an agent's AP field to the
		// AMS and opens its start gate. Without going through this path
		// for itself, the AMS's own agent.AP would stay unset and its
		// replies to REQUESTs would be rejected by isRegisteredFunction.
		sysVar* element;
		MAESUBaseType_t i = 0;
		while (i < AGENT_LIST_SIZE) {
			element = env.getEnv(&env);

			if (element[i].first == (Agent_AID)0)
			{
				break;
			}
			platform->register_agent(platform, element[i].first);
			i++;
		}
		// register_agent() maps every agent's ordinal priority through
		// MAES_RTPrioFor(), which reserves the top slot for the AMS; make
		// sure the AMS itself actually ends up there rather than at
		// whatever MAES_RTPrioFor(agentAMS.agent.priority) computed.
		MAES_SetPriority(platform->agentAMS.agent.aid, MAES_AMSRTPrio());
		return NO_ERRORS;
	}
	else
	{
		/* System_abort */
		return INVALID;
	}
};

//Agent Initiate Function: This function creates a pthread for an agent, held at a start gate until it is registered.
//Inputs: The Platform instance itself, the agent and the agent's behavior.
//Outputs: None.
void agent_initFunction(Agent_Platform* platform, MAESAgent* agent, void (*behaviour)(void*)) {
	spawn_agent_thread(agent, behaviour, NULL);
};

//Agent Initiate with Parameters Function: This function creates a pthread for an agent. Also, it includes input parameters
//Inputs: The Platform instance itself, the agent, the agent's behavior and its input parameters.
//Outputs: None.
void agent_initConParamFunction(Agent_Platform* platform, MAESAgent* agent, void (*behaviour)(void*), void* pvParameters) {
	spawn_agent_thread(agent, behaviour, pvParameters);
};

//Agent Search Function: This function searches for an agent in the platform.
//Inputs: The Platform instance itself and the agent AID.
//Outputs: A bool value that indicates if the agent was found in the platform.
bool agent_searchFunction(Agent_Platform* platform, Agent_AID aid) {
	for (MAESUBaseType_t i = 0; i < platform->description.subscribers; i++)
	{
		if (pthread_equal(platform->Agent_Handle[i], aid))
		{
			return true;
		}
	}
	return false;
};

//Agent Wait Function: This function creates a delay (milliseconds).
//Inputs: The Platform instance itself, the agent and the amount of milliseconds of delay.
//Outputs: None.
void agent_waitFunction(Agent_Platform* platform, MAESTickType_t ticks) {
	MAES_CheckSuspend(MAES_GetCurrentTaskHandle());
	struct timespec ts;
	ts.tv_sec = ticks / 1000;
	ts.tv_nsec = (long)(ticks % 1000) * 1000000L;
	nanosleep(&ts, NULL);
};

//Agent Yield Function: This function yields the processor to another thread.
//Inputs: The Platform instance itself.
//Outputs: None.
void agent_yieldFunction(Agent_Platform* platform) {
	sched_yield();
};

//Get Running Agent Function: This function indicates the agent that is currently executing its task.
//Inputs: The Platform instance itself.
//Outputs: None.
Agent_AID get_running_agentFunction(Agent_Platform* platform) {
	return MAES_GetCurrentTaskHandle();
};

AGENT_MODE get_stateFunction(Agent_Platform* platform, Agent_AID aid) {
	if (platform->agent_search(platform, aid))
	{
		MAES_AgentControl* ctrl = control_find(aid);
		if (ctrl == NULL)
		{
			return NO_MODE;
		}
		return ctrl->mode;
	}
	else
	{
		return NO_MODE;
	}
};

//Get Agent Description Function: This function indicates the description of an specific agent.
//Inputs: The Platform instance itself and the agents AID.
//Outputs: The description of the agent.
Agent_info get_Agent_descriptionFunction(Agent_AID aid) {
	MAESAgent* a = (MAESAgent*)env.get_taskEnv(&env, aid);
	return a->agent;
};

//Get Agent Platform Description Function: This function indicates the platform's description.
//Inputs: The Platform instance itself.
//Outputs: A pointer to the platform's description (the header always declared a
//pointer; returning the struct by value through that pointer type was undefined
//behaviour).
AP_Description* get_AP_descriptionFunction(Agent_Platform* platform) {
	return &platform->description;
};

//Register Agent Function: This function registers an agent into the platform.
//Inputs: The Platform instance itself and the agent's AID.
//Outputs: An error code indicating if registering the agent was successful.
ERROR_CODE register_agentFunction(Agent_Platform* platform, Agent_AID aid) {
	if (aid == (Agent_AID)NULL)
	{
		return HANDLE_NULL;
	}
	else if (caller_is_setup_thread(platform) || caller_is_ams(platform))
	{
		if (!platform->agent_search(platform,aid))
		{
			if (platform->description.subscribers < AGENT_LIST_SIZE)
			{
				MAESAgent* a;
				a = env.get_taskEnv(&env, aid);
				a->agent.AP = platform->agentAMS.agent.aid;
				platform->Agent_Handle[platform->description.subscribers] = aid;
				platform->description.subscribers++;
				MAES_SetPriority(aid, MAES_RTPrioFor(a->agent.priority));
				MAES_AgentControl* ctrl = control_find(aid);
				if (ctrl != NULL)
				{
					control_release(ctrl);
				}
				return NO_ERRORS;
			}
			else
			{
				printf("\nList FUll\n");
				return LIST_FULL;
			}
		}
		else
		{
			printf("\nDuplicated\n");
			return DUPLICATED;
		}
	}
	else
	{
		printf("\nInvalid\n");
		return INVALID;
	}
};

//Deregister Agent Function: This function deregisters an agent into the platform.
//Inputs: The Platform instance itself and the agent's AID.
//Outputs: An error code indicating if deregistering the agent was successful.
ERROR_CODE deregister_agentFunction(Agent_Platform* platform, Agent_AID aid) {
	if (caller_is_ams(platform))
	{
		MAESUBaseType_t i = 0;
		while (i < AGENT_LIST_SIZE)
		{
			if (pthread_equal(platform->Agent_Handle[i], aid))
			{
				MAESAgent* a;
				platform->suspend_agent(platform,aid);
				a = (MAESAgent*)env.get_taskEnv(&env, aid);
				a->agent.AP = (Agent_AID)NULL;

				while (i < AGENT_LIST_SIZE - 1)
				{
					platform->Agent_Handle[i] = platform->Agent_Handle[i + 1];
					i++;
				}
				platform->Agent_Handle[AGENT_LIST_SIZE - 1] = (Agent_AID)NULL;
				platform->description.subscribers--;
				break;
			}
			i++;
		}
		if (i == AGENT_LIST_SIZE)
		{
			return NOT_FOUND;
		}
	}
	return NO_ERRORS;
};

//Kill Agent Function: This function kills an agent into the platform.
//Inputs: The Platform instance itself and the agent's AID.
//Outputs: An error code indicating if killing the agent was successful.
ERROR_CODE kill_agentFunction(Agent_Platform* platform, Agent_AID aid) {
	if (caller_is_ams(platform))
	{
		ERROR_CODE error;
		error = platform->deregister_agent(platform,aid);

		if (error == NO_ERRORS)
		{
			MAESAgent* a;
			Mailbox_Handle m;

			a = (MAESAgent*)env.get_taskEnv(&env, aid);
			m = a->agent.mailbox_handle;

			pthread_cancel(aid);
			pthread_join(aid, NULL);

			MAES_QueueDelete(m);

			MAES_AgentControl* ctrl = control_find(aid);
			control_free(ctrl);

			a->agent.aid = (Agent_AID)NULL;
			env.erase_TaskEnv(&env,aid);
			// No subscribers-- here: deregister_agent() above already removed the
			// agent from Agent_Handle and decremented the count. Decrementing again
			// made agent_search() miss the last registered agent after every kill.
		}
		return error;
	}
	else
	{
		return INVALID;
	}
};

//Suspend Agent Function: This function suspend an agent into the platform.
//Inputs: The Platform instance itself and the agent's AID.
//Outputs: An error code indicating if suspending the agent was successful.
ERROR_CODE suspend_agentFunction(Agent_Platform* platform, Agent_AID aid) {
	if (caller_is_ams(platform))
	{
		if (platform->agent_search(platform,aid))
		{
			MAES_AgentControl* ctrl = control_find(aid);
			if (ctrl != NULL)
			{
				control_pause(ctrl);
			}
			return (NO_ERRORS);
		}
		else
		{
			return NOT_FOUND;
		}
	}
	else
	{
		return INVALID;
	}
};

//Resume Agent Function: This function resumes an agent into the platform.
//Inputs: The Platform instance itself and the agent's AID.
//Outputs: An error code indicating if resuming the agent was successful.
ERROR_CODE resume_agentFunction(Agent_Platform* platform, Agent_AID aid) {
	if (caller_is_ams(platform))
	{
		if (platform->agent_search(platform,aid))
		{
			MAESAgent* a;
			a = (MAESAgent*)env.get_taskEnv(&env, aid);
			MAES_AgentControl* ctrl = control_find(aid);
			if (ctrl != NULL)
			{
				control_release(ctrl);
			}
			MAES_SetPriority(aid, MAES_RTPrioFor(a->agent.priority));
			return NO_ERRORS;
		}
		else
		{
			return NOT_FOUND;
		}
	}
	else
	{
		return INVALID;
	}
};

//Restart Agent Function: This function restarts an agent into the platform.
//Inputs: The Platform instance itself and the agent's AID.
//Outputs: None.
void restartFunction(Agent_Platform* platform, Agent_AID aid) {
	if (caller_is_ams(platform))
	{
		MAESAgent* a;
		a = (MAESAgent*)env.get_taskEnv(&env, aid);
		Mailbox_Handle m;
		// delete Task and Mailbox
		m = a->agent.mailbox_handle;
		pthread_cancel(aid);
		pthread_join(aid, NULL);
		MAES_QueueDelete(m);
		MAES_AgentControl* old_ctrl = control_find(aid);
		control_free(old_ctrl);
		env.erase_TaskEnv(&env,aid);
		// Mailbox and Task, freshly created (starts suspended again)
		spawn_agent_thread(a, a->resources.function, a->resources.taskParameters);
	}
};

//Agent Platform Constructor: This function assigns the class pointers to its corresponding function.
//Inputs: Ponter to the Agent Platform class.
//Outputs: None.
void ConstructorAgent_Platform(Agent_Platform* platform, sysVars* env) {
	platform->ptr_env = env;
	platform->Agent_Platform = &Agent_PlatformFunction;
	platform->Agent_PlatformWithCond = &Agent_PlatformWithCondFunction;
	platform->boot = &bootFunction;
	platform->agent_init = &agent_initFunction;
	platform->agent_initConParam = &agent_initConParamFunction;
	platform->agent_search = &agent_searchFunction;
	platform->agent_wait = &agent_waitFunction;
	platform->agent_yield = &agent_yieldFunction;
	platform->get_running_agent = &get_running_agentFunction;
	platform->get_state = &get_stateFunction;
	platform->get_Agent_description = &get_Agent_descriptionFunction;
	platform->get_AP_description = &get_AP_descriptionFunction;
	platform->register_agent = &register_agentFunction;
	platform->deregister_agent = &deregister_agentFunction;
	platform->kill_agent = &kill_agentFunction;
	platform->suspend_agent = &suspend_agentFunction;
	platform->resume_agent = &resume_agentFunction;
	platform->restart = &restartFunction;
};
