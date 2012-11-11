#include "Angular.h"
#include "Distance.h" // for superclass PhysicsGraph < Distance, Angular >::Edge

/*
================================
Angular::Angular
================================
*/
Angular::Angular( Distance* m, Distance* n ) :
	PhysicsGraph < Distance, Angular >::Edge( m, n )
{
	// TODO: Super sketchy constructor.
}
