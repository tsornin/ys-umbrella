#include "Contact.h"

/*
================================
Contact::Contact
================================
*/
Contact::Contact( Rigid* a, Rigid* b ) : Constraint( a, b )
{
	
}

/*
================================
Contact::jacobian
================================
*/
std::pair < Vec3, Vec3 > Contact::jacobian()
{
	return std::pair < Vec3, Vec3 >(
		- Vec3( normal, (a_p - a->getPosition()) ^ normal ),
		  Vec3( normal, (b_p - b->getPosition()) ^ normal ) );
}
