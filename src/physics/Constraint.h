#ifndef PHYSICS_CONSTRAINT_H
#define PHYSICS_CONSTRAINT_H

#include "PhysicsTags.h"
#include "PhysicsGraph.h"
#include "spatial/Vec3.h"
#include "graphics/Renderer.h"

class Rigid;

/*
================================
???
================================
*/
class Constraint :
	public PhysicsTags,
	public PhysicsGraph < Rigid, Constraint >::Edge
{
protected: // Lifecycle
	Constraint( Rigid* a, Rigid* b );
	Constraint( const Constraint& ) = delete;
	Constraint& operator = ( const Constraint& ) = delete;
	virtual ~Constraint() {}

public: // "Entity" functions
	virtual void draw( Renderer& rd ) const {}

public: // Constraint functions
	virtual Scalar eval() const = 0;
	virtual std::pair < Vec3, Vec3 > jacobian() const = 0;
	virtual Scalar bias( Scalar jv ) const = 0;
	virtual std::pair < Scalar, Scalar > bounds() const = 0;

protected: // Members
	// Warm starting
	Scalar lambda;

	friend class PhysicsState;
	template < typename T > friend struct Expire;
	friend class Renderer;
};

#endif
