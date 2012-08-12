#ifndef PHYSICS_CONTACT_H
#define PHYSICS_CONTACT_H

class Rigid;

struct Contact
{
public:
	Vec2 normal; // Points away from body A
	Scalar overlap; // Positive when penetrating

	// TODO: We don't need the Rigid bodies here, do we?
	Rigid* a;
	int a_i;
	Vec2 a_p;

	Rigid* b;
	int b_i;
	Vec2 b_p;
};

#endif
