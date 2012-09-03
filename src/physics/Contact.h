#ifndef PHYSICS_CONTACT_H
#define PHYSICS_CONTACT_H

#include "Constraint.h" // superclass Constraint
#include <string> // TODO: see std::hash < Identifier >

/*
================================
Contact constraint identifier.

This identifier is unique for contact points up to their varying positions,
and is used to match new contacts to old (previous step) contacts.

Contact constraints are formed between two Rigid bodies.
We use three IDs for each Rigid body to uniquely identify a contact:
	pid:	the ID of the Rigid body in the physics engine
	cid:	the index of the Convex shape in the Rigid body
	fid:	the index of the feature involved in the collision
The feature ID indices either a vertex or an edge.
By convention, body A is the reference (edge) shape,
and body B is the incident (vertex) shape.
================================
*/
struct Identifier
{
public:
	int a_pid;
	int a_cid;
	int a_fid;

	int b_pid;
	int b_cid;
	int b_fid;

	// For std::unordered_map < Identifier, Contact* >
	friend bool operator == ( const Identifier& id1, const Identifier& id2 ) {
		return
			id1.a_pid == id2.a_pid &&
			id1.a_cid == id2.a_cid &&
			id1.a_fid == id2.a_fid &&
			id1.b_pid == id2.b_pid &&
			id1.b_cid == id2.b_cid &&
			id1.b_fid == id2.b_fid;
	}
};

// For std::unordered_map < Identifier, Contact* >
namespace std {
	template <> struct hash < Identifier > {
		size_t operator() ( const Identifier& x ) const {
			// TODO: Is this standard user-type hashing procedure?
			string s( (char*) &x, sizeof( Identifier ) );
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

public: // Contact
	Identifier key();

public: // Members
	Vec2 normal; // Points away from body A
	Scalar overlap; // Positive when penetrating

	Vec2 a_p;
	int a_cid;
	int a_fid;

	Vec2 b_p;
	int b_cid;
	int b_fid;
};

#endif
