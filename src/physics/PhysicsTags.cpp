#include "PhysicsTags.h"

PhysicsTags::PhysicsTags() :
	pid( -1 ),
	mask( 0 ),
	expire_enable( false ),
	owner( 0 )
{
	
}

/*
================================
PhysicsTags::expired

In lieu of a good data structure for PhysicsState::destroy,
we're just using an "expired" flag.
================================
*/
bool PhysicsTags::expired() const
{
	return expire_enable;
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
