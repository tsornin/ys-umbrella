#include "Random.h"
#include <cstdlib> // for std::rand
#include <cmath> // for std::sqrt

/*
================================
irand
================================
*/
int irand( int n ) {
	return ( std::rand() % n );
}

/*
================================
brand
================================
*/
bool brand() {
	return irand( 2 );
}

/*
================================
frand
================================
*/
double frand() {
	static double rand_max_inverse = 1.0 / (double) RAND_MAX;
	return ( (double) std::rand() ) * rand_max_inverse;
}

/*
================================
grand

See java.util.Random.nextGaussian
================================
*/
bool haveNextNextGaussian = false;
double nextNextGaussian = 0.0;
double grand() {
	if ( haveNextNextGaussian ) {
		haveNextNextGaussian = false;
		return nextNextGaussian;
	}
	else {
		double v1, v2, s;
		do {
			v1 = 2 * frand() - 1; // between -1.0 and 1.0
			v2 = 2 * frand() - 1; // between -1.0 and 1.0
			s = v1*v1 + v2*v2;
		} while ( s >= 1 || s == 0 );
		double multiplier = std::sqrt( -2 * std::log(s)/s );
		nextNextGaussian = v2 * multiplier;
		haveNextNextGaussian = true;
		return (v1 * multiplier);
	}
}
