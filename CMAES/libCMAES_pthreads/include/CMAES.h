/***********************************************************
 *   MAES is a framework for Real Time Embedded Systems    *
 *   designed by C. Chan-Zheng at TU Delft.                *
 *   This is a an adaptation written in C of the library   *
 *   FreeMAES which implements the MAES framework.         *
 *   Original FreeRTOS library by Andres Conejo Boza.      *
 *   This is a native POSIX/pthreads backend (Linux),      *
 *   exposing the same public API as libCMAES_FreeRTOS.    *
************************************************************/

#ifndef CMAES_H
#define CMAES_H

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>




	/*******************************************************
	 *            PLATFORM PRIMITIVES (pthreads)           *
	 *******************************************************/

	// NOTE on Agent_AID identity: like the original FreeRTOS library (which
	// compares raw xTaskHandle values with ==), this backend compares raw
	// pthread_t values with ==. That is well-defined on glibc/Linux, where
	// pthread_t is an integral type - the target platform (Raspberry Pi OS).
	// It is NOT portable to platforms where pthread_t is an opaque struct;
	// a future improvement would route every comparison through
	// pthread_equal() instead.

#define MAESTaskHandle_t pthread_t
#define MAESQueueHandle_t struct MAES_Queue*
#define MAESTickType_t uint32_t                 // milliseconds, not RTOS ticks
typedef void (*MAESTaskFunction_t)(void*);
#define MAESUBaseType_t unsigned int
#define MAESBaseType_t int

#define MAES_MAX_DELAY UINT32_MAX                // "block forever", like portMAX_DELAY

#define Agent_AID MAESTaskHandle_t		             // Agent ID
#define Mailbox_Handle MAESQueueHandle_t            // Agent's Mailbox Handle
#define AGENT_LIST_SIZE 64					        // Maximum Agents per platform
#define MAX_RECEIVERS AGENT_LIST_SIZE - 1        	// Maximum receivers available for any agent
#define BEHAVIOUR_LIST_SIZE 8				        //
#define ORGANIZATIONS_SIZE 16				         // Maximum Members for org




	/*******************************************************
	 *                        ENUMS                        *
	 *******************************************************/

	typedef enum
	{
		ACCEPT_PROPOSAL,
		AGREE,
		CANCEL,
		CFP,
		CONFIRM,
		DISCONFIRM,
		FAILURE,
		INFORM,
		INFORM_IF,
		INFORM_REF,
		NOT_UNDERSTOOD,
		PROPAGATE,
		PROPOSE,
		QUERY_IF,
		QUERY_REF,
		REFUSE,
		REJECT_PROPOSAL,
		REQUEST,
		REQUEST_WHEN,
		REQUEST_WHENEVER,
		SUBSCRIBE,
		NO_RESPONSE,
	}MSG_TYPE;

	typedef enum
	{
		NO_ERRORS,
		FOUND,
		HANDLE_NULL,
		LIST_FULL,
		DUPLICATED,
		NOT_FOUND,
		TIMEOUT,
		INVALID,
		NOT_REGISTERED,
	}ERROR_CODE;

	typedef enum
	{
		ACTIVE,
		SUSPENDED,
		WAITING,
		TERMINATED,
		NO_MODE,
	}AGENT_MODE;

	typedef enum
	{
		OWNER,
		ADMIN,
		MEMBER,
		NON_MEMBER,
	}ORG_AFFILIATION;

	typedef enum
	{
		MODERATOR,
		PARTICIPANT,
		VISITOR,
		NONE,
	}ORG_ROLE;

	typedef enum
	{
		HIERARCHY,
		TEAM,
	}ORG_TYPE;

	/*******************************************************
	 *                     DEFINITIONS                     *
	 *******************************************************/

	 // Organization Information
	typedef struct
	{
		ORG_TYPE org_type;
		MAESUBaseType_t members_num;
		MAESUBaseType_t banned_num;
		Agent_AID members[AGENT_LIST_SIZE];
		Agent_AID banned[AGENT_LIST_SIZE];
		Agent_AID owner;
		Agent_AID admin;
		Agent_AID moderator;
	} org_info;

	// Agent Information
	typedef struct Agent_info Agent_info;
	struct Agent_info
	{
		Agent_AID aid;
		Mailbox_Handle mailbox_handle;
		const char* agent_name;
		MAESUBaseType_t priority;
		Agent_AID AP;
		org_info* org;
		ORG_AFFILIATION affiliation;
		ORG_ROLE role;
	};

	// Agent Resources
	typedef struct Agent_resources Agent_resources;
	struct Agent_resources
	{
		uint16_t stackSize;
		void (*function)(void*);
		void* taskParameters;
	};

	// Message
	typedef struct
	{
		Agent_AID sender_agent;
		Agent_AID target_agent;
		MSG_TYPE type;
		char* content;
	} MsgObj;

	// AP Description
	typedef struct
	{
		Agent_AID AMS_AID;
		const char* AP_name;
		MAESUBaseType_t subscribers;
	} AP_Description;


	/****************************************************** *
	 *                       CLASSES                       *
	 *******************************************************/

	// Class: MAESAgent//

	typedef struct MAESAgent MAESAgent;
	struct MAESAgent {

		//Attributes
		Agent_info agent;
		Agent_resources resources;

		//Methods
		void* (*Iniciador)(MAESAgent*,const char* ,MAESUBaseType_t ,uint16_t );
		// NOTE: declared Agent_AID* in the original FreeRTOS header, but the
		// implementation always returned a bare Agent_AID - harmless there
		// only because xTaskHandle is itself a pointer typedef. On pthreads,
		// Agent_AID is pthread_t (not a pointer), so that mismatch would be
		// a genuine int-from-pointer-function bug; declared correctly here.
		Agent_AID (*AID)(MAESAgent *);
	};

	//Class: sysVars//

	typedef struct {
		Agent_AID first;
		MAESAgent* second;
	}sysVar;

	typedef struct sysVars sysVars;
	struct sysVars {

		//Attributes
		sysVar environment[AGENT_LIST_SIZE];

		//Methods
		MAESAgent* (* get_taskEnv)(sysVars*,Agent_AID aid);
		void* (*set_TaskEnv)(sysVars*, Agent_AID aid, MAESAgent* agent_ptr);
		void* (* erase_TaskEnv)(sysVars*,Agent_AID aid);
		sysVar* (*getEnv)(sysVars*);
	};

	extern sysVars env;

//Class: Agent Message//

	typedef struct Agent_Msg Agent_Msg;
	struct Agent_Msg {

		//Attributes
		MsgObj msg;
		sysVars* ptr_env;
		Agent_AID receivers[MAX_RECEIVERS];
		MAESUBaseType_t subscribers;
		Agent_AID caller;

		//Methods
		bool (* isRegistered)(Agent_Msg*,Agent_AID aid);
		// NOTE: the original FreeRTOS header declared this as returning
		// Mailbox_Handle* (pointer-to-handle), but every call site actually
		// used the return value as a plain Mailbox_Handle - harmless there
		// only because xQueueHandle is itself a pointer typedef. Declared
		// correctly here as Mailbox_Handle to match real usage.
		Mailbox_Handle (* get_mailbox)(Agent_Msg*,Agent_AID aid);
		void* (*Agent_Msg)(Agent_Msg*);
		ERROR_CODE (*add_receiver)(Agent_Msg*,Agent_AID aid_receiver);
		ERROR_CODE (*remove_receiver)(Agent_Msg*,Agent_AID aid_receiver);
		void* (*clear_all_receiver)(Agent_Msg*);
		void* (*refresh_list)(Agent_Msg*);
		MSG_TYPE (*receive)(Agent_Msg*,MAESTickType_t timeout);
		ERROR_CODE (*send)(Agent_Msg*,Agent_AID aid_receiver, MAESTickType_t timeout);
		ERROR_CODE (*send0)(Agent_Msg*);
		void* (*set_msg_type)(Agent_Msg*,MSG_TYPE type);
		void* (*set_msg_content)(Agent_Msg*,char* body);
		MsgObj* (*get_msg)(Agent_Msg*);
		MSG_TYPE (*get_msg_type)(Agent_Msg*);
		char* (*get_msg_content)(Agent_Msg*);
		Agent_AID (*get_sender)(Agent_Msg*);
		Agent_AID (*get_target_agent)(Agent_Msg*);
		ERROR_CODE (*registration)(Agent_Msg*,Agent_AID target_agent);
		ERROR_CODE (*deregistration)(Agent_Msg*,Agent_AID target_agent);
		ERROR_CODE (*suspend)(Agent_Msg*,Agent_AID target_agent);
		ERROR_CODE (*resume)(Agent_Msg*,Agent_AID target_agent);
		ERROR_CODE (*kill)(Agent_Msg*,Agent_AID target_agent);
		ERROR_CODE (*restart)(Agent_Msg*);
	};

	//Class: User Defined Conditions//

	typedef struct USER_DEF_COND USER_DEF_COND;
	struct USER_DEF_COND {

		//Methods
		bool (*register_cond)();
		bool (*deregister_cond)();
		bool (*suspend_cond)();
		bool (*kill_cond)();
		bool (*resume_cond)();
		bool (*restart_cond)();
	};

// Class: Agent Platform//

	typedef struct AMSparameter AMSparameter;
	typedef struct Agent_Platform Agent_Platform;
	struct Agent_Platform {

		//Attributes
		sysVars* ptr_env;
		MAESAgent agentAMS;
		Agent_AID Agent_Handle[AGENT_LIST_SIZE];
		AP_Description description;
		USER_DEF_COND cond;
		USER_DEF_COND* ptr_cond;
		AMSparameter* parameter;
		Agent_AID setup_thread; // pthreads backend only: thread that built the platform (see Agent_Platform.c)

		//Methods
		void* (*Agent_Platform)(Agent_Platform* platform,const char* name);
		void* (*Agent_PlatformWithCond)(Agent_Platform* platform, const char* name,
			USER_DEF_COND* user_cond);
		bool (*boot)(Agent_Platform* platform);
		void* (*agent_init)(Agent_Platform* platform, MAESAgent* agent, void* behaviour);
		void* (*agent_initConParam)(Agent_Platform* platform, MAESAgent* agent,
			void behaviour(void* pvParameters), void* pvParameters);
		bool (*agent_search)(Agent_Platform* platform, Agent_AID aid);
		void* (* agent_wait)(Agent_Platform* platform, MAESTickType_t ticks);
		void* (*agent_yield)(Agent_Platform* platform);
		Agent_AID (*get_running_agent)(Agent_Platform* platform);
		AGENT_MODE (*get_state)(Agent_Platform* platform, Agent_AID aid);
		Agent_info (*get_Agent_description)(Agent_AID aid);
		AP_Description* (*get_AP_description)(Agent_Platform* platform);
		ERROR_CODE (*register_agent)(Agent_Platform* platform, Agent_AID aid);
		ERROR_CODE (*deregister_agent)(Agent_Platform* platform, Agent_AID aid);
		ERROR_CODE (*kill_agent)(Agent_Platform* platform, Agent_AID aid);
		ERROR_CODE (*suspend_agent)(Agent_Platform* platform, Agent_AID aid);
		ERROR_CODE (*resume_agent)(Agent_Platform* platform, Agent_AID aid);
		void* (*restart)(Agent_Platform* platform, Agent_AID aid);
	};

	extern Agent_Platform Platform;

	//Class: AMS Task
	struct AMSparameter{
		sysVars* ptr_env;
		Agent_Platform* services;
		USER_DEF_COND* cond;
	};




	//Class: CyclicBehaviour//

	typedef struct CyclicBehaviour CyclicBehaviour;
	struct CyclicBehaviour {

		//Attributes
		void* taskParameters;
		Agent_Msg* msg;

		//Methods
		void* (* action)(CyclicBehaviour* Behaviour, void* pvParameters);
		void* (*setup)(CyclicBehaviour* Behaviour, void* pvParameters);
		bool (*done)(CyclicBehaviour* Behaviour, void* pvParameters);
		bool (*failure_detection)(CyclicBehaviour* Behaviour, void* pvParameters);
		void* (*failure_identification)(CyclicBehaviour* Behaviour, void* pvParameters);
		void* (*failure_recovery)(CyclicBehaviour* Behaviour, void* pvParameters);
		void* (*execute)(CyclicBehaviour* Behaviour, void* pvParameters);
	};


	//Class: OneShotBehaviour//

	typedef struct OneShotBehaviour OneShotBehaviour;
	struct OneShotBehaviour {

		//Attributes
		void* taskParameters;
		Agent_Msg* msg;

		//Methods
		void* (*action)(OneShotBehaviour* Behaviour, void* pvParameters);
		void* (*setup)(OneShotBehaviour* Behaviour, void* pvParameters);
		bool (*done)(OneShotBehaviour* Behaviour, void* pvParameters);
		bool (*failure_detection)(OneShotBehaviour* Behaviour, void* pvParameters);
		void* (*failure_identification)(OneShotBehaviour* Behaviour, void* pvParameters);
		void* (*failure_recovery)(OneShotBehaviour* Behaviour, void* pvParameters);
		void* (*execute)(OneShotBehaviour* Behaviour, void* pvParameters);
	};

	//Class: Agent Organization//

	typedef struct Agent_Organization Agent_Organization;
	struct Agent_Organization {

		//Attributes
		sysVars* ptr_env;
		org_info description;

		//Methods
		void* (*Agent_Organization)(Agent_Organization* Organization,ORG_TYPE organization_type);
		ERROR_CODE (*create)(Agent_Organization* Organization);
		ERROR_CODE (*destroy)(Agent_Organization* Organization);
		ERROR_CODE (*isMember)(Agent_Organization* Organization,Agent_AID aid);
		ERROR_CODE (*isBanned)(Agent_Organization* Organization, Agent_AID aid);
		ERROR_CODE (*change_owner)(Agent_Organization* Organization, Agent_AID aid);
		ERROR_CODE (*set_admin)(Agent_Organization* Organization, Agent_AID aid);
		ERROR_CODE (*set_moderator)(Agent_Organization* Organization, Agent_AID aid);
		ERROR_CODE (*add_agent)(Agent_Organization* Organization, Agent_AID aid);
		ERROR_CODE (*kick_agent)(Agent_Organization* Organization, Agent_AID aid);
		ERROR_CODE (*ban_agent)(Agent_Organization* Organization, Agent_AID aid);
		ERROR_CODE (*remove_ban)(Agent_Organization* Organization, Agent_AID aid);
		void* (*clear_ban_list)(Agent_Organization* Organization);
		ERROR_CODE (*set_participan)(Agent_Organization* Organization, Agent_AID aid);
		ERROR_CODE (*set_visitor)(Agent_Organization* Organization, Agent_AID aid);
		ORG_TYPE (*get_org_type)(Agent_Organization* Organization);
		org_info (*get_info)(Agent_Organization* Organization);
		MAESUBaseType_t (*get_size)(Agent_Organization* Organization);
		MSG_TYPE (*invite)(Agent_Organization* Organization, Agent_Msg msg,
			MAESUBaseType_t password, Agent_AID target_agent, MAESUBaseType_t timeout);
	};


	void ConstructorAgente(MAESAgent* agente);
	void ConstructorSysVars(sysVars* Vars);
	void ConstructorAgent_Msg(Agent_Msg* Message, sysVars* env);
	void ConstructorAgent_Platform(Agent_Platform* platform, sysVars* env);
	void ConstructorCyclicBehaviour(CyclicBehaviour* Behaviour);
	void ConstructorOneShotBehaviour(OneShotBehaviour* Behaviour);
	void ConstructorAgent_Organization(Agent_Organization* Organization, sysVars* env);


	/*******************************************************
	 *      PTHREADS BACKEND SUPPORT API (new, no          *
	 *      FreeRTOS equivalent - see MAES_Queue.c and      *
	 *      Agent_Platform.c)                               *
	 *******************************************************/

	// Single-slot blocking queue, equivalent to xQueueCreate(1, sizeof(MsgObj))
	typedef struct MAES_Queue MAES_Queue;

	MAES_Queue* MAES_QueueCreate(void);
	bool MAES_QueueSend(MAES_Queue* q, const MsgObj* msg, MAESTickType_t timeout_ms);
	bool MAES_QueueReceive(MAES_Queue* q, MsgObj* out, MAESTickType_t timeout_ms);
	void MAES_QueueDelete(MAES_Queue* q);

	// Current-thread identity, replaces xTaskGetCurrentTaskHandle()
	Agent_AID MAES_GetCurrentTaskHandle(void);

	// Cooperative suspend checkpoint: blocks the calling agent if it has been
	// suspended, until resumed. Called from CyclicBehaviour/OneShotBehaviour
	// execute() loops and from agent_wait(). See the plan's note on suspend
	// being cooperative (not preemptive) under pthreads.
	void MAES_CheckSuspend(Agent_AID aid);

#endif // CMAES_H
