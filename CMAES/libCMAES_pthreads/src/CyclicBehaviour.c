#include <CMAES.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

//Cyclic Action Function: This function is a wrapper that contains the core action of the agent's behavior. This function must be redefined in the main application.
//Inputs: The behavior instance itself and parameters (in case they are needed).
//Outputs: None.
void actionFunction(CyclicBehaviour* Behaviour, void* pvParameters) {
	// Empty
};

//Cyclic Setup Function: This function is a wrapper that contains some lines of code that must be executed before the action function is called. 
//Inputs: The behavior instance itself and parameters (in case they are needed).
//Outputs: None.
void setupFunction(CyclicBehaviour* Behaviour, void* pvParameters) {
	// Empty
};

//Cyclic Done Function: This function determines if the behavior will be executed multiple times. In the Cyclic case, this will always be false, meaning it will be executed indefinitely.  
//Inputs: The behavior instance itself and parameters (in case they are needed).
//Outputs: True bool meaning the behavior will be executed indefinitely.
bool doneFunction(CyclicBehaviour* Behaviour, void* pvParameters) {
	return false;
};

//Cyclic Failure Detection Function: This function is a wrapper that can be used to develop an algorithm for failure detection.  
//Inputs: The behavior instance itself and parameters (in case they are needed).
//Outputs: Bool value indicating if a failure was detected.
bool failure_detectionFunction(CyclicBehaviour* Behaviour, void* pvParameters) {
	return false;
};

//Cyclic Failure Identification Function: This function is a wrapper that can be used to develop an algorithm for failure identification.  
//Inputs: The behavior instance itself and parameters (in case they are needed).
//Outputs: None.
void failure_identificationFunction(CyclicBehaviour* Behaviour, void* pvParameters) {
	// Empty
};

//Cyclic Failure Recovery Function: This function is a wrapper that can be used to develop an algorithm for failure recovery.  
//Inputs: The behavior instance itself and parameters (in case they are needed).
//Outputs: None.
void failure_recoveryFunction(CyclicBehaviour* Behaviour, void* pvParameters) {
	// Empty
};

//Cyclic Execute Function: This function is a wrapper that determines the agent's behavior. This is the function that must be called from the main application.  
//Inputs: The behavior instance itself and parameters (in case they are needed).
//Outputs: None.
void executeFunction(CyclicBehaviour* Behaviour, void* pvParameters) {
	Behaviour->setup(Behaviour, pvParameters);
	do
	{
		// pthreads backend: suspend_agent() is cooperative, not preemptive
		// (see Agent_Platform.c) - this is where a suspended agent actually
		// blocks, at the top of every iteration.
		MAES_CheckSuspend(MAES_GetCurrentTaskHandle());
		Behaviour->action(Behaviour, pvParameters);
		if (Behaviour->failure_detection(Behaviour, pvParameters))
		{
			Behaviour->failure_identification(Behaviour, pvParameters);
			Behaviour->failure_recovery(Behaviour, pvParameters);
		}
	} while (!Behaviour->done(Behaviour, pvParameters));
};

//Cyclic Behaviour Constructor: This function assigns the class pointers to its corresponding function.
//Inputs: Ponter to the Cyclic Behaviour class.
//Outputs: None.
void ConstructorCyclicBehaviour(CyclicBehaviour* Behaviour) {
	Behaviour->action = &actionFunction;
	Behaviour->setup = &setupFunction;
	Behaviour->done = &doneFunction;
	Behaviour->failure_detection = &failure_detectionFunction;
	Behaviour->failure_identification = &failure_detectionFunction;
	Behaviour->failure_recovery = &failure_recoveryFunction;
	Behaviour->execute = &executeFunction;
};