#include "Constraint.h"

Constraint::Constraint( Rigid* a, Rigid* b ) :
	// Vertices
	a(a), b(b),
	// Warm starting
	lambda(0)
{
	
}
