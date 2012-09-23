#include "MouseConstraint.h"

/*
================================
MouseConstraint::MouseConstraint
================================
*/
MouseConstraint::MouseConstraint( Rigid* a, Rigid* b ) : Constraint( a, b )
{
	lambda = 0;
}

/*
================================
MouseConstraint::eval
================================
*/
Scalar MouseConstraint::eval()
{
	return (b->world( b_local ) - a_world).length();
}

/*
================================
MouseConstraint::jacobian
================================
*/
std::pair < Vec3, Vec3 > MouseConstraint::jacobian()
{
	Vec2 axis = b->world( b_local ) - a_world;

	if ( axis.length2() < 0.1 ) {
		return std::pair < Vec3, Vec3 >( Vec3(1), Vec3(1) );
	}

	Vec2 n = axis.unit();
	Vec2 r = a_world - b->getPosition();

	return std::pair < Vec3, Vec3 >( Vec3(1), Vec3( n, r^n ) );
}

/*
================================
MouseConstraint::bias
================================
*/
Scalar MouseConstraint::bias( Scalar jv )
{
	return -eval() * 1.0;
}

/*
================================
MouseConstraint::bounds
================================
*/
std::pair < Scalar, Scalar > MouseConstraint::bounds()
{
	return std::pair < Scalar, Scalar >( -b->getMass() * 5, 0 );
}
