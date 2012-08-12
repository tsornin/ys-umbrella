#ifndef PHYSICS_CONTACT_H
#define PHYSICS_CONTACT_H

struct Contact
{
public:
	// Points away from body 1
	Vec2 normal;

	int b1_index;
	Vec2 b1_where;

	int b2_index;
	Vec2 b2_where;
};

#endif
