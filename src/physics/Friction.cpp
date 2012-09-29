#include "Friction.h"

/*
================================
Friction::Friction
================================
*/
Friction::Friction( Rigid* a, Rigid* b ) : Constraint( a, b )
{
	
}

/*
================================
Friction::eval
================================
*/
Scalar Friction::eval()
{
	return 0;
}

/*
================================
Friction::jacobian
================================
*/
std::pair < Vec3, Vec3 > Friction::jacobian()
{
	return std::pair < Vec3, Vec3 >(
		- Vec3( tangent, (p - a->getPosition()) ^ tangent ),
		  Vec3( tangent, (p - b->getPosition()) ^ tangent ) );
}

/*
================================
Friction::bias
================================
*/
Scalar Friction::bias( Scalar jv )
{
	return 0;
}

/*
================================
Friction::bounds
================================
*/
std::pair < Scalar, Scalar > Friction::bounds()
{
	// Friction mixing
	Scalar k = geometric_mean( a->getFriction(), b->getFriction() );

	// Bound by last frame's lambda (TODO: problem?)
	Scalar l = k * normal_lambda;

	return std::pair < Scalar, Scalar >( -l, l );
}
