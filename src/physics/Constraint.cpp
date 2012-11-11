#include "Constraint.h"

Constraint::Constraint( Rigid* a, Rigid* b ) :
	PhysicsGraph < Rigid, Constraint >::Edge( a, b ),
	// Warm starting
	lambda(0)
{
	
}
