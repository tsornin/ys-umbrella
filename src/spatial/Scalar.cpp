#include "Scalar.h"
#include <cmath> // for std::sqrt, std::fabs

static const Scalar DEGS_PER_RAD = 57.295779513082320876798154814105;
Scalar to_degrees( Scalar rad )
	{ return rad * DEGS_PER_RAD; }

static const Scalar RADS_PER_DEG = 0.01745329251994329576923690768489;
Scalar to_radians( Scalar deg )
	{ return deg * RADS_PER_DEG; }

Scalar geometric_mean( Scalar a, Scalar b )
	{ return std::sqrt( a * b ); }

Scalar arithmetic_mean( Scalar a, Scalar b )
	{ return ( a + b ) * 0.5; }

static const Scalar EQUALS_EPSILON = 1e-4;
Scalar equals( Scalar a, Scalar b ) {
	return std::fabs( a - b ) <= EQUALS_EPSILON * std::fabs( a );
}
