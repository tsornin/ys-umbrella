#include "PhysicsTags.h"

PhysicsTags::PhysicsTags() :
	expire_enable( false ),
	pid( -1 ),
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
