#ifndef PHYSICS_MOUSE_CONSTRAINT_H
#define PHYSICS_MOUSE_CONSTRAINT_H

#include "Constraint.h"

class MouseConstraint : public Constraint
{
private: // Lifecycle
	MouseConstraint( Rigid* a, Rigid* b );
public:
	virtual ~MouseConstraint() {}
	friend class PhysicsState;

public: // Constraint
	virtual Scalar eval();
	virtual std::pair < Vec3, Vec3 > jacobian();
	virtual Scalar bias( Scalar jv );
	virtual std::pair < Scalar, Scalar > bounds();

	virtual void draw( Renderer& rd ) { rd.drawMouseConstraint( *this ); }
	friend class Renderer;

public: // Mouse constraint
	Vec2 a_world;
	Vec2 b_local;
};

#endif
