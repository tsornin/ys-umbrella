#ifndef PHYSICS_CONSTRAINT_H
#define PHYSICS_CONSTRAINT_H

#include "PhysicsTags.h"
#include <set>
#include "spatial/Vec3.h"
#include "Rigid.h"

class Constraint : public PhysicsTags
{
protected:
	Constraint( Rigid* a, Rigid* b );
	virtual ~Constraint() {}

	virtual std::pair < Vec3, Vec3 > jacobian() = 0;

	friend class PhysicsState;

protected: // Members
	// Vertices
	Rigid
		*a, // source vertex (reference)
		*b; // target vertex (incident)

private: // Physics engine graph data
};

#endif
