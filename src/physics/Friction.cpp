#include "Friction.h"
#include "Rigid.h"

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
Scalar Friction::eval() const
{
	// No position stabilization for Friction
	return 0;
}

/*
================================
Friction::jacobian
================================
*/
std::pair < Vec3, Vec3 > Friction::jacobian() const
{
	return std::pair < Vec3, Vec3 >(
		- Vec3( tangent, (p - a->position) ^ tangent ),
		  Vec3( tangent, (p - b->position) ^ tangent ) );
}

/*
================================
Friction::bias
================================
*/
Scalar Friction::bias( Scalar jv ) const
{
	// No position stabilization for Friction
	return 0;
}

/*
================================
Friction::bounds
================================
*/
std::pair < Scalar, Scalar > Friction::bounds() const
{
	// Bound friction using the cached normal force
	// (doesn't seem to be a problem)
	Scalar k = mix_friction();
	Scalar l = k * normal_lambda;

	return std::pair < Scalar, Scalar >( -l, l );
}

/*
================================
Friction::mix_friction
================================
*/
Scalar Friction::mix_friction() const
{
	return Friction::mix_friction( a->friction, b->friction );
}

/*
================================
Friction::mix_friction
================================
*/
Scalar Friction::mix_friction( Scalar k1, Scalar k2 )
{
	return std::max( k1, k2 );
}
