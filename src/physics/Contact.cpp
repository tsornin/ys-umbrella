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
Contact::~Contact
================================
*/
Contact::~Contact()
{
	delete ft;
}

/*
================================
Contact::eval
================================
*/
Scalar Contact::eval() const
{
	return -overlap;
}

/*
================================
Contact::jacobian
================================
*/
std::pair < Vec3, Vec3 > Contact::jacobian() const
{
	return std::pair < Vec3, Vec3 >(
		- Vec3( normal, (a_p - a->getPosition()) ^ normal ),
		  Vec3( normal, (b_p - b->getPosition()) ^ normal ) );
}

/*
================================
Contact::bias
================================
*/
Scalar Contact::bias( Scalar jv ) const
{
	Scalar ret = 0;

	// Restitution
	static const Scalar PHYSICS_BOUNCE_THRESHOLD = 2.0;
	// Restitution threshold
	if ( std::fabs( jv ) > PHYSICS_BOUNCE_THRESHOLD ) {
		Scalar e = mix_restitution();
		ret += -jv * e;
	}

	// Position stabilization
	static const Scalar PHYSICS_SLOP = 0.1;
	static const Scalar PHYSICS_BIAS = 0.1;

	Scalar error = -eval() - PHYSICS_SLOP;
	if ( error > 0 ) {
		ret += error * PHYSICS_BIAS;
	}

	return ret;
}

/*
================================
Contact::bounds
================================
*/
std::pair < Scalar, Scalar > Contact::bounds() const
{
	return std::pair < Scalar, Scalar >( 0, SCALAR_MAX );
}

/*
================================
Contact::local_lambda

Returns the reaction force if this Contact were independent.
This isn't directly used by the solver;
it's used to bound the normal force for newly created Friction contacts
(and since we're doing that, we might as well use it for warm starting too).
================================
*/
Scalar Contact::local_lambda() const
{
	auto J = jacobian();
	Scalar jv =
		J.first.dot( a->getVelocityState() ) +
		J.second.dot( b->getVelocityState() );

	Scalar jmjt =
		J.first.prod( J.first ).dot( a->getInverseMass() ) +
		J.second.prod( J.second ).dot( b->getInverseMass() );

	Scalar e = mix_restitution();

	return -(1+e) * jv / jmjt;
}

/*
================================
Contact::mix_restitution
================================
*/
Scalar Contact::mix_restitution() const
{
	return std::max( a->getBounce(), b->getBounce() );
}
