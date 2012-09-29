#ifndef PHYSICS_CONSTRAINT_H
#define PHYSICS_CONSTRAINT_H

#include "PhysicsTags.h"
#include <set>
#include "spatial/Vec3.h"
#include "Rigid.h"
#include "graphics/Renderer.h"

class Constraint : public PhysicsTags
{
protected: // Lifecycle
	Constraint( Rigid* a, Rigid* b );
	virtual ~Constraint() {}

	friend class PhysicsState;
	template < typename T > friend struct Expire;

public: // Constraint
	virtual Scalar eval() const = 0;
	virtual std::pair < Vec3, Vec3 > jacobian() const = 0;
	virtual Scalar bias( Scalar jv ) const = 0;
	virtual std::pair < Scalar, Scalar > bounds() const = 0;

	virtual void draw( Renderer& rd ) const {}
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
