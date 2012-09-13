#ifndef PHYSICS_CONSTANTS_H
#define PHYSICS_CONSTANTS_H

#include "spatial/Scalar.h"

/*
================================
Physics constants.
================================
*/

// 20% decay per second @ 60 fps: 0.996288
// 20% decay per second @ 30 fps: 0.992589
// 80% decay per second @ 30 fps: 0.947766
// 80% decay per second @ 60 fps: 0.973533
const Scalar STANDARD_LINEAR_DAMPING = 0.973533;
const Scalar STANDARD_ANGULAR_DAMPING = 0.973533;

// Default collision properties
const Scalar STANDARD_MASS		= 1.0;
const Scalar STANDARD_RADIUS	= 1.0;
const Scalar STANDARD_MOMENT	= 1.0; // Moment is mass * radius * radius
const Scalar STANDARD_BOUNCE	= 0.0;
const Scalar STANDARD_DENSITY	= 1.0;
const Scalar STANDARD_FRICTION	= 0.0;

const Scalar GRAVITY_LO		= 0.3;
const Scalar GRAVITY_MED	= 0.6;
const Scalar GRAVITY_HI		= 0.9;

#endif
