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
Contact::eval
================================
*/
Scalar Contact::eval()
{
	return -overlap;
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

/*
================================
Contact::bounds
================================
*/
std::pair < Scalar, Scalar > Contact::bounds()
{
	return std::pair < Scalar, Scalar >( 0, SCALAR_MAX );
}

Identifier Contact::key()
{
	Identifier ret;

		ret.a_pid = a->pid;
		ret.a_cid = a_cid;
		ret.a_fid = a_fid;

		ret.b_pid = b->pid;
		ret.b_cid = b_cid;
		ret.b_fid = b_fid;

	return ret;
}
