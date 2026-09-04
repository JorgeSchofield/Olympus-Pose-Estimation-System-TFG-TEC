// Smoke test for the pthreads CMAES backend: a straight port of
// libCMAES_FreeRTOS/src/sender_reciever.c onto libCMAES_pthreads. Two
// agents, one Cyclic "write" behaviour sending a message every second, one
// Cyclic "read" behaviour blocking on receive - validates agent creation,
// registration, mailbox send/receive and the AMS request/response path
// end-to-end before building anything more elaborate on top.

#include <CMAES.h>
#include <stdio.h>

MAESAgent sender, receiver;
Agent_Platform AP;
sysVars env;
CyclicBehaviour writingBehaviour, readingBehaviour;
Agent_Msg msg_writing, msg_reading;


//Functions related to the writing Behaviour//

void writingsetup(CyclicBehaviour* Behaviour, void* pvParameters) {
	Behaviour->msg->Agent_Msg(Behaviour->msg);
	Behaviour->msg->add_receiver(Behaviour->msg,receiver.AID(&receiver));
};

void writingaction(CyclicBehaviour* Behaviour, void* pvParameters) {
	printf("***Sending Message***\n");
	Behaviour->msg->send0(Behaviour->msg);
	AP.agent_wait(&AP, 1000);
};

//Write Wrapper:
void write_wrapper(void* pvParameters) {
	writingBehaviour.msg = &msg_writing;
	writingBehaviour.setup = &writingsetup;
	writingBehaviour.action = &writingaction;
	writingBehaviour.execute(&writingBehaviour,&pvParameters);
};


//Functions related to the reading Behaviour

void readingaction(CyclicBehaviour* Behaviour, void* pvParameters) {
	Behaviour->msg->Agent_Msg(Behaviour->msg);
	Behaviour->msg->receive(Behaviour->msg,MAES_MAX_DELAY);
	printf("***Message Received: Hello world***\n\n");
};

//Read Wrapper:
void read_wrapper(void* pvParameters) {
	readingBehaviour.msg = &msg_reading;
	readingBehaviour.action = &readingaction;
	readingBehaviour.execute(&readingBehaviour,&pvParameters);
};


//Main
int main(void) {
	printf("------Sender Receiver APP (pthreads backend)------ \n");
	//Constructors for each initialized class

	ConstructorAgente(&sender);
	ConstructorAgente(&receiver);
	ConstructorSysVars(&env);
	ConstructorAgent_Platform(&AP, &env);
	ConstructorAgent_Msg(&msg_writing, &env);
	ConstructorAgent_Msg(&msg_reading, &env);
	ConstructorCyclicBehaviour(&writingBehaviour);
	ConstructorCyclicBehaviour(&readingBehaviour);

	//Initializing the Agents and the Platform.
	sender.Iniciador(&sender, "Agent Sender", 1, 512);
	receiver.Iniciador(&receiver, "Agent Receiver", 3, 512);
	AP.Agent_Platform(&AP, "sender_receiver_platform");

	//Registering the Agents and their respective behaviour into the Platform
	AP.agent_init(&AP,&sender, &write_wrapper);
	AP.agent_init(&AP,&receiver, &read_wrapper);
	AP.boot(&AP);
	printf("CMAES booted successfully \n");
	printf("Initiating APP\n\n");

	// Unlike the FreeRTOS demo (which hands control to vTaskStartScheduler
	// and never returns), agent threads here are already running once
	// boot() returns. Block the main thread on the AMS, which loops
	// forever servicing platform requests.
	pthread_join(AP.description.AMS_AID, NULL);

	return 0;
};
