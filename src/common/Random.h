#ifndef COMMON_RANDOM_H
#define COMMON_RANDOM_H

/*
================================
Global random functions.
================================
*/

// Returns a random integer in [ 0, n ).
int irand( int n );

// Returns true and false equally often.
bool brand();

// Returns uniformly distributed doubles in [ 0.0, 1.0 ].
double frand();

// Returns Gaussian distributed doubles with mean 0 and standard deviation 1.
double grand();

#endif
