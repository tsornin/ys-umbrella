#ifndef SPATIAL_SCALAR_H
#define SPATIAL_SCALAR_H

#include <cmath>
#include <cfloat> // for DBL_MAX, FLT_MAX
#include <algorithm> // for std::min, std::max

/*
================================
Precision typedef
================================
*/
// #define SCALAR_USE_DOUBLE_PRECISION
#ifdef SCALAR_USE_DOUBLE_PRECISION
	typedef double Scalar;
	#define SCALAR_BIG 1e100
	#define SCALAR_MAX DBL_MAX
#else
	typedef float Scalar;
	#define SCALAR_BIG 1e18f
	#define SCALAR_MAX FLT_MAX
#endif

/*
================================
Constants
================================
*/
const Scalar PI = 3.1415926535897932384626433832795;

/*
================================
Functions
================================
*/
Scalar to_degrees( Scalar rad );
Scalar to_radians( Scalar deg );

Scalar geometric_mean( Scalar a, Scalar b );
Scalar arithmetic_mean( Scalar a, Scalar b );

Scalar equals( Scalar a, Scalar b );

template< typename T >
bool bound( T& n, const T& a, const T& b ) {
	const T n_prev = n;
	n = std::max( n, a );
	n = std::min( n, b );
	return ( n == n_prev );
}

template < typename T >
void clamp( T& n, const T& a, const T& b ) {
	if ( n < a ) n = a;
	if ( b < n ) n = b;
}

#endif
