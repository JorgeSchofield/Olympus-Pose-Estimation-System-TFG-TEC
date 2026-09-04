#include <CMAES.h>
#include <stdio.h>
#include <stdlib.h>


//AID Function: It returns the Agent ID of the Agent given.
//Inputs: Pointer to an Agent
//Output: Pointer to the Agent's ID.
Agent_AID AID0(MAESAgent* MAESagent) {
	return MAESagent->agent.aid;
};

//Agent Inicialization: The function sets the default value for each parameter.
//Inputs: Pointer to an Agent, String that contains the name you want to give to the Agent, priority and the size of the stack
//Outputs: None
void MAESAgent1(MAESAgent* MAESagent, const char* name, MAESUBaseType_t pri, uint16_t sizeStack) {
	MAESagent->agent.aid = (Agent_AID)0;
	MAESagent->agent.mailbox_handle = NULL;
	MAESagent->agent.agent_name = name;
	MAESagent->agent.priority = pri;
	MAESagent->resources.stackSize = sizeStack;
	MAESagent->agent.AP = (Agent_AID)0;
	MAESagent->agent.org = NULL;
	MAESagent->agent.affiliation = NON_MEMBER;
	MAESagent->agent.role = VISITOR;
};

//Agent Constructor: This function assigns the class pointers to its corresponding function.
//Inputs: Pointer to the Agent.
//Outputs: None.
void ConstructorAgente(MAESAgent* agente) {
	agente->Iniciador = &MAESAgent1;
	agente->AID = &AID0;
}
