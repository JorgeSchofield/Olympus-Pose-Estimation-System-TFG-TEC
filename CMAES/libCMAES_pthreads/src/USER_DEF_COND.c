#include <CMAES.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

//Register Condition Function: Enables or disables the possibility of registering an agent. 
//Inputs: None.
//Outputs: True value indicating that the condition is enabled or false value indicating that the condition is disabled.
bool register_condFunction()
{
    return true;
}

//Kill Condition Function: Enables or disables the possibility of killing an agent. 
//Inputs: None.
//Outputs: True value indicating that the condition is enabled or false value indicating that the condition is disabled.
bool kill_condFunction()
{
    return true;
}

//Deregister Condition Function: Enables or disables the possibility of deregistering an agent. 
//Inputs: None.
//Outputs: True value indicating that the condition is enabled or false value indicating that the condition is disabled.
bool deregister_condFunction()
{
    return true;
}

//Suspend Condition Function: Enables or disables the possibility of suspending an agent. 
//Inputs: None.
//Outputs: True value indicating that the condition is enabled or false value indicating that the condition is disabled.
bool suspend_condFunction()
{
    return true;
}

//Resume Condition Function: Enables or disables the possibility of resuming an agent. 
//Inputs: None.
//Outputs: True value indicating that the condition is enabled or false value indicating that the condition is disabled.
bool resume_condFunction()
{
    return true;
}

//Restart Condition Function: Enables or disables the possibility of restarting an agent. 
//Inputs: None.
//Outputs: True value indicating that the condition is enabled or false value indicating that the condition is disabled.
bool restart_condFunction()
{
    return true;
}

//USER_DEF_COND Constructor: This function assigns the class pointers to its corresponding function.
//Inputs: Ponter to the USER_DEF_COND class.
//Outputs: None.
void ConstructorUSER_DEF_COND(USER_DEF_COND* cond) {
    cond->register_cond = &register_condFunction;
    cond->kill_cond = &kill_condFunction;
    cond->deregister_cond = &deregister_condFunction;
    cond->suspend_cond = &suspend_condFunction;
    cond->resume_cond = &resume_condFunction;
    cond->restart_cond = &restart_condFunction;
};