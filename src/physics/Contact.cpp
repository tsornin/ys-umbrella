#include "Contact.h"

bool operator == ( const FeatureKey& fk1, const FeatureKey& fk2 ) {
	return
		fk1.pid == fk2.pid &&
		fk1.cid == fk2.cid &&
		fk1.fid == fk2.fid;
}

bool operator == ( const ContactKey& ck1, const ContactKey& ck2 ) {
	return (ck1.a == ck2.a) && (ck1.b == ck2.b);
}

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
