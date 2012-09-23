#ifndef PHYSICS_CONSTRAINT_H
#define PHYSICS_CONSTRAINT_H

#include "PhysicsTags.h"
#include <set>
#include "spatial/Vec3.h"
#include "Rigid.h"
#include "graphics/Renderer.h"

class Constraint : public PhysicsTags
{
protected:
	Constraint( Rigid* a, Rigid* b );
public:
	virtual ~Constraint() {}

	virtual Scalar eval() = 0;
	virtual std::pair < Vec3, Vec3 > jacobian() = 0;
	virtual Scalar bias( Scalar jv ) = 0;
	virtual std::pair < Scalar, Scalar > bounds() = 0;

	virtual void draw( Renderer& rd ) {}

	friend class PhysicsState;
	friend class Renderer;

protected: // Members
	// Vertices
	Rigid
		*a, // source vertex (reference)
		*b; // target vertex (incident)

	// Warm starting
	Scalar lambda;
};

#endif
