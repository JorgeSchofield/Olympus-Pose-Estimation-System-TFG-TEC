#include <CMAES.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>


//Agent Organization Function: This function sets some of the initial values of the parameters needed in the agent organization.
//Inputs: The organization instance itself and the organization type.
//Outputs: None.
void Agent_OrganizationFunction(Agent_Organization* Organization, ORG_TYPE organization_type) {
	Organization->description.org_type = organization_type;
	Organization->description.members_num = 0;
	Organization->description.banned_num = 0;

	for (MAESUBaseType_t i = 0; i < AGENT_LIST_SIZE; i++)
	{
		Organization->description.members[i] = (Agent_AID)NULL;
		Organization->description.banned[i] = (Agent_AID)NULL;
	}
};

//Create Function: This function creates an organization with some initial parameters.
//Inputs: The organization instance itself.
//Outputs: An error code indicating if creating the organization was successful.
ERROR_CODE createFunction(Agent_Organization* Organization) {
	if (Organization->description.owner == (Agent_AID)NULL)
	{
		MAESAgent* agent = (MAESAgent*)Organization->ptr_env->get_taskEnv(Organization->ptr_env,MAES_GetCurrentTaskHandle());

		if (agent->agent.AP != (Agent_AID)NULL)
		{
			agent->agent.role = PARTICIPANT;
			agent->agent.affiliation = OWNER;
			agent->agent.org = &Organization->description;
			Organization->description.owner = MAES_GetCurrentTaskHandle();
			Organization->description.members[Organization->description.members_num] = Organization->description.owner;
			Organization->description.members_num++;
			return NO_ERRORS;
		}
		else
		{
			return NOT_REGISTERED;
		}
	}
	else
	{
		return INVALID;
	}
};

//Destroy Function: This function destroys an organization.
//Inputs: The organization instance itself.
//Outputs: An error code indicating if destroying the organization was successful.
ERROR_CODE destroyFunction(Agent_Organization* Organization) {
	if (pthread_equal(Organization->description.owner, MAES_GetCurrentTaskHandle()))
	{
		MAESAgent* agent;
		for (MAESUBaseType_t i = 0; i < Organization->description.members_num; i++)
		{
			agent = (MAESAgent*)Organization->ptr_env->get_taskEnv(Organization->ptr_env,Organization->description.members[i]);
			agent->agent.org = NULL;
			agent->agent.affiliation = NON_MEMBER;
			agent->agent.role = NONE;
			Organization->description.members[i] = (Agent_AID)NULL;
		}
		for (MAESUBaseType_t i = 0; i < Organization->description.banned_num; i++)
		{
			Organization->description.banned[i] = (Agent_AID)NULL;
		}
		return NO_ERRORS;
	}

	else
	{
		return INVALID;
	}
};

//Is Member Function: This function searches for an agent in the organization.
//Inputs: The organization instance itself and the agent AID.
//Outputs: An error code indicating if the agent was found in the organization.
ERROR_CODE isMemberFunction(Agent_Organization* Organization, Agent_AID aid) {
	for (MAESUBaseType_t i = 0; i < Organization->description.members_num; i++)
	{
		if (pthread_equal(Organization->description.members[i], aid))
		{
			return FOUND;
		}
	}
	return NOT_FOUND;
};

//Is Banned Function: This function searches for an agent in the organization.
//Inputs: The organization instance itself and the agent AID.
//Outputs: An error code indicating if the agent was banned from the organization.
ERROR_CODE isBannedFunction(Agent_Organization* Organization, Agent_AID aid) {
	for (MAESUBaseType_t i = 0; i < Organization->description.banned_num; i++)
	{
		if (pthread_equal(Organization->description.banned[i], aid))
		{
			return FOUND;
		}
	}
	return NOT_FOUND;
};

//Change Owner Function: This function changes the organization's owner.
//Inputs: The organization instance itself and the agent AID.
//Outputs: An error code indicating if the organization's owner has been changed.
ERROR_CODE change_ownerFunction(Agent_Organization* Organization, Agent_AID aid) {
	if (pthread_equal(Organization->description.owner, MAES_GetCurrentTaskHandle()) && Organization->isMember(Organization,aid) == FOUND)
	{
		MAESAgent* agent = (MAESAgent*)Organization->ptr_env->get_taskEnv(Organization->ptr_env, Organization->description.owner);
		agent->agent.affiliation = MEMBER;
		agent->agent.role = VISITOR;
		agent = (MAESAgent*)Organization->ptr_env->get_taskEnv(Organization->ptr_env, aid);
		agent->agent.affiliation = OWNER;
		agent->agent.role = PARTICIPANT;
		Organization->description.owner = aid;
		return NO_ERRORS;
	}
	else
	{
		return INVALID;
	}
};

//Set Admin Function: This function sets the organization's administrator.
//Inputs: The organization instance itself and the agent AID.
//Outputs: An error code indicating if the agent AID has been set as the organization's administrator.
ERROR_CODE set_adminFunction(Agent_Organization* Organization, Agent_AID aid) {
	if (pthread_equal(Organization->description.owner, MAES_GetCurrentTaskHandle()) && Organization->isMember(Organization,aid) == FOUND)
	{
		MAESAgent* agent = (MAESAgent*)Organization->ptr_env->get_taskEnv(Organization->ptr_env, aid);
		if (agent->agent.affiliation != OWNER)
			agent->agent.affiliation = ADMIN;
		agent->agent.role = PARTICIPANT;
		Organization->description.admin = aid;
		return NO_ERRORS;
	}
	else
	{
		return INVALID;
	}
};

//Set Moderator Function: This function sets the organization's moderator.
//Inputs: The organization instance itself and the agent AID.
//Outputs: An error code indicating if the agent AID has been set as the organization's moderator.
ERROR_CODE set_moderatorFunction(Agent_Organization* Organization, Agent_AID aid) {
	if (pthread_equal(Organization->description.owner, MAES_GetCurrentTaskHandle()) && Organization->isMember(Organization,aid) == FOUND)
	{
		MAESAgent* agent = (MAESAgent*)Organization->ptr_env->get_taskEnv(Organization->ptr_env, aid);
		agent->agent.role = MODERATOR;
		Organization->description.moderator = aid;
		return NO_ERRORS;
	}
	else
	{
		return INVALID;
	}
};

//Add Agent Function: This function adds an agent into the organization.
//Inputs: The organization instance itself and the agent AID.
//Outputs: An error code indicating if the agent was added successfully into the organization.
ERROR_CODE add_agentFunction(Agent_Organization* Organization, Agent_AID aid) {
	MAESAgent* agent = (MAESAgent*)Organization->ptr_env->get_taskEnv(Organization->ptr_env, aid);

	if (Organization->description.members_num == AGENT_LIST_SIZE)
		return LIST_FULL;
	else if (agent->agent.org != NULL || Organization->isBanned(Organization,aid) == FOUND || agent->agent.AP == (Agent_AID)NULL)
	{
		return INVALID;
	}
	else if (Organization->isMember(Organization,aid) == FOUND)
	{
		return DUPLICATED;
	}
	else if (pthread_equal(Organization->description.owner, MAES_GetCurrentTaskHandle()) || pthread_equal(Organization->description.admin, MAES_GetCurrentTaskHandle()))
	{
		Organization->description.members[Organization->description.members_num] = aid;
		Organization->description.members_num++;
		agent->agent.affiliation = MEMBER;
		agent->agent.role = PARTICIPANT;
		agent->agent.org = &Organization->description;
		return NO_ERRORS;
	}
	else
	{
		return INVALID;
	}
};

//Kick Agent Function: This function kicks an agent from the organization.
//Inputs: The organization instance itself and the agent AID.
//Outputs: An error code indicating if the agent was successfully kicked from the organization.
ERROR_CODE kick_agentFunction(Agent_Organization* Organization, Agent_AID aid) {
	if (pthread_equal(aid, MAES_GetCurrentTaskHandle()))
		return INVALID;
	else if (pthread_equal(Organization->description.owner, MAES_GetCurrentTaskHandle()) || pthread_equal(Organization->description.admin, MAES_GetCurrentTaskHandle()))
	{
		MAESUBaseType_t i = 0;
		MAESAgent* agent = (MAESAgent*)Organization->ptr_env->get_taskEnv(Organization->ptr_env, aid);
		while (i < AGENT_LIST_SIZE)
		{
			if (pthread_equal(Organization->description.members[i], aid))
			{
				while (i < AGENT_LIST_SIZE - 1)
				{
					Organization->description.members[i] = Organization->description.members[i + 1];
					i++;
				}
				Organization->description.members[AGENT_LIST_SIZE - 1] = (Agent_AID)NULL;
				Organization->description.members_num--;
				agent->agent.role = NONE;
				agent->agent.affiliation = NON_MEMBER;
				agent->agent.org = NULL;
				break;
			}
			i++;
		}
		if (i == AGENT_LIST_SIZE)
		{
			return NOT_FOUND;
		}

		else
		{
			return NO_ERRORS;
		}
	}

	else
	{
		return INVALID;
	}
};

//Ban Agent Function: This function banns an agent from the organization.
//Inputs: The organization instance itself and the agent AID.
//Outputs: An error code indicating if the agent was successfully banned from the organization.
ERROR_CODE ban_agentFunction(Agent_Organization* Organization, Agent_AID aid) {
	if (Organization->description.banned_num == AGENT_LIST_SIZE)
	{
		return LIST_FULL;
	}

	else if (Organization->isBanned(Organization,aid) == FOUND)
	{
		return DUPLICATED;
	}

	else if (pthread_equal(Organization->description.owner, MAES_GetCurrentTaskHandle()) || pthread_equal(Organization->description.admin, MAES_GetCurrentTaskHandle()))
	{
		Organization->description.banned[Organization->description.banned_num] = aid;
		Organization->description.banned_num++;
		if (Organization->isMember(Organization,aid) == FOUND)
			Organization->kick_agent(Organization,aid);
		return NO_ERRORS;
	}

	else
	{
		return INVALID;
	}
};

//Remove Ban Function: This function removes the ban status of an agent in the organization.
//Inputs: The organization instance itself and the agent AID.
//Outputs: An error code indicating if the agent was successfully unbanned from the organization.
ERROR_CODE remove_banFunction(Agent_Organization* Organization, Agent_AID aid) {
	if (pthread_equal(Organization->description.owner, MAES_GetCurrentTaskHandle()) || pthread_equal(Organization->description.admin, MAES_GetCurrentTaskHandle()))
	{
		MAESUBaseType_t i = 0;
		while (i < AGENT_LIST_SIZE)
		{
			if (pthread_equal(Organization->description.banned[i], aid))
			{
				while (i < AGENT_LIST_SIZE - 1)
				{
					Organization->description.banned[i] = Organization->description.banned[i + 1];
					i++;
				}
				Organization->description.banned[AGENT_LIST_SIZE - 1] = (Agent_AID)NULL;
				Organization->description.banned_num--;
				break;
			}
			i++;
		}

		if (i == AGENT_LIST_SIZE)
		{
			return NOT_FOUND;
		}

		else
		{
			return NO_ERRORS;
		}
	}
	else
	{
		return INVALID;
	}
};

//Clear Ban List Function: This function clears the list of banned agents in the organization.
//Inputs: The organization instance itself.
//Outputs: None.
void clear_ban_listFunction(Agent_Organization* Organization) {
	if (pthread_equal(Organization->description.owner, MAES_GetCurrentTaskHandle()) || pthread_equal(Organization->description.admin, MAES_GetCurrentTaskHandle()))
	{
		for (MAESUBaseType_t i = 0; i < AGENT_LIST_SIZE; i++)
		{
			Organization->description.banned[i] = (Agent_AID)NULL;
		}
	}
};

//Set Participant Function: This function sets the participant role to an agent in the organization.
//Inputs: The organization instance itself and the agent AID.
//Outputs: An error code indicating if the agent's participant role was set successfully.
ERROR_CODE set_participanFunction(Agent_Organization* Organization, Agent_AID aid) {
	if ((pthread_equal(Organization->description.owner, MAES_GetCurrentTaskHandle()) || pthread_equal(Organization->description.moderator, MAES_GetCurrentTaskHandle())) && Organization->isMember(Organization,aid) == FOUND)
	{
		MAESAgent* agent = (MAESAgent*)Organization->ptr_env->get_taskEnv(Organization->ptr_env, aid);
		agent->agent.role = PARTICIPANT;
		return NO_ERRORS;
	}
	else
	{
		return INVALID;
	}
};

//Set Visitor Function: This function sets the visitor role to an agent in the organization.
//Inputs: The organization instance itself and the agent AID.
//Outputs: An error code indicating if the agent's visitor role was set successfully.
ERROR_CODE set_visitorFunction(Agent_Organization* Organization, Agent_AID aid) {
	if ((pthread_equal(Organization->description.owner, MAES_GetCurrentTaskHandle()) || pthread_equal(Organization->description.moderator, MAES_GetCurrentTaskHandle())) && Organization->isMember(Organization,aid) == FOUND)
	{
		MAESAgent* agent = (MAESAgent*)Organization->ptr_env->get_taskEnv(Organization->ptr_env, aid);
		agent->agent.role = VISITOR;
		return NO_ERRORS;
	}
	else
	{
		return INVALID;
	}
};

//Get Organization Type Function: This function returns the organization's type.
//Inputs: The organization instance itself.
//Outputs: The organization's type itself.
ORG_TYPE get_org_typeFunction(Agent_Organization* Organization) {
	return Organization->description.org_type;
};

//Get Information Function: This function returns the organization's information stored in its description.
//Inputs: The organization instance itself.
//Outputs: The organization's description.
org_info get_infoFunction(Agent_Organization* Organization) {
	return Organization->description;
};

//Get Size Function: This function returns the organization's size.
//Inputs: The organization instance itself.
//Outputs: The amount of agents that can be registered in the organization.
MAESUBaseType_t get_sizeFunction(Agent_Organization* Organization) {
	return Organization->description.members_num;
};

//Invite Function: This function invites an agent into the organization and, if he accepts, the agent is added into the organization.
//Inputs: The organization instance itself, the message used to invite the agent, the organization's password, the target agent and the timeout.
//Outputs: The message type, indicating if the agent accepted or denied the invitation.
MSG_TYPE inviteFunction(Agent_Organization* Organization, Agent_Msg msg, MAESUBaseType_t password, Agent_AID target_agent, MAESUBaseType_t timeout) {
	MAESAgent* caller = (MAESAgent*)Organization->ptr_env->get_taskEnv(Organization->ptr_env, MAES_GetCurrentTaskHandle());

	if (caller->agent.affiliation == OWNER || caller->agent.affiliation == ADMIN)
	{
		msg.set_msg_type(&msg,PROPOSE);
		msg.set_msg_content(&msg,(char*)"Join Organization");
		msg.send(&msg,target_agent, timeout);

		msg.receive(&msg,timeout);

		if (msg.get_msg_type(&msg) == ACCEPT_PROPOSAL)
		{
			Organization->add_agent(Organization,target_agent);
		}
	}

	else
	{
		msg.set_msg_type(&msg,REFUSE);
	}
	return msg.get_msg_type(&msg);
};


//Agent Organization Constructor: This function assigns the class pointers to its corresponding function.
//Inputs: Ponter to the Agent Organization class.
//Outputs: None.
void ConstructorAgent_Organization(Agent_Organization* Organization, sysVars* env) {
	Organization->ptr_env=env;
	Organization->Agent_Organization = &Agent_OrganizationFunction;
	Organization->create = &createFunction;
	Organization->destroy = &destroyFunction;
	Organization->isMember = &isMemberFunction;
	Organization->isBanned = &isBannedFunction;
	Organization->change_owner = &change_ownerFunction;
	Organization->set_admin = &set_adminFunction;
	Organization->set_moderator = &set_moderatorFunction;
	Organization->add_agent = &add_agentFunction;
	Organization->kick_agent = &kick_agentFunction;
	Organization->ban_agent = &ban_agentFunction;
	Organization->remove_ban = &remove_banFunction;
	Organization->clear_ban_list = &clear_ban_listFunction;
	Organization->set_participan = &set_participanFunction;
	Organization->set_visitor = &set_visitorFunction;
	Organization->get_org_type = &get_org_typeFunction;
	Organization->get_info = &get_infoFunction;
	Organization->get_size = &get_sizeFunction;
	Organization->invite = &inviteFunction;
};
