#ifndef PHYSICS_CONTACT_H
#define PHYSICS_CONTACT_H

#include "Constraint.h" // superclass Constraint
#include <string> // TODO: see std::hash < Identifier >

/*
================================
A key that uniquely identifies a Rigid body feature.
================================
*/
struct FeatureKey
{
public:
	int pid; // global ID of the Rigid body in the physics engine
	int cid; // index of the Convex shape in the Rigid body
	int fid; // index of the feature in the Convex shape (a vertex or an edge)

	friend bool operator == ( const FeatureKey& fk1, const FeatureKey& fk2 );
};

/*
================================
A key that uniquely identifies a Contact between two Rigid body features.

Used to match new contacts to old contacts.
================================
*/
struct ContactKey
{
public:
	FeatureKey a; // the reference feature (an edge)
	FeatureKey b; // the incident feature (a vertex)

	// For std::unordered_map < ContactKey, Contact* >
	friend bool operator == ( const ContactKey& ck1, const ContactKey& ck2 );
};

// For std::unordered_map < ContactKey, Contact* >
namespace std {
	template <>
	struct hash < ContactKey >
	{
		size_t operator () ( const ContactKey& x ) const {
			// TODO: Not sure if this is standard user-type hashing procedure.
			string s( (char*) &x, sizeof( ContactKey ) );
			return hash < string > () ( s );
		}
	};
}

/*
================================
Contact constraint.

Represents a single point of contact
between collision shapes of two Rigid bodies.

Instances of this class are managed by the physics engine.
Contact constraints are transient and should not be used by other classes.
================================
*/
class Contact : public Constraint
{
public: // Constraint
	Contact( Rigid* a, Rigid* b );
	virtual ~Contact() {}

	virtual Scalar eval();
	virtual std::pair < Vec3, Vec3 > jacobian();
	virtual std::pair < Scalar, Scalar > bounds();

	friend class PhysicsState;

public: // Members
	Vec2 normal; // Points away from body A
	Scalar overlap; // Positive when penetrating
	Vec2 a_p;
	Vec2 b_p;

	// Contact caching
	ContactKey key;
};

#endif
