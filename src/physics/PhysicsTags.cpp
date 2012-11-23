#include "PhysicsTags.h"

PhysicsTags::PhysicsTags() :
	mask( 0 ),
	owner( 0 ),
	pid( -1 )
{
	
}

/*
================================
PhysicsTags::pid_lt

Sort physics objects by their PID.
================================
*/
bool PhysicsTags::pid_lt( PhysicsTags* a, PhysicsTags* b )
{
	return a->pid < b->pid;
}
