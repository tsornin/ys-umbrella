#ifndef PHYSICS_CONTACT_H
#define PHYSICS_CONTACT_H

class Rigid;

struct Contact
{
public: // Constraint common
	Rigid* a;
	Rigid* b;

public: // Contact specific
	Vec2 normal; // Points away from body A
	Scalar overlap; // Positive when penetrating
	Vec2 a_p;
	Vec2 b_p;
};

#endif
