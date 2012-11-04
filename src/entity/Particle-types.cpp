#include "Particle-types.h"

Flame::Flame( EntityState& es ) : Particle( es )
{
	eu->setBounce( 0 );
	eu->setGravity( Vec2( 0, GRAVITY_LO / 2 ) );
	eu->mask = 1;
}

Flames::Flames( EntityState& es ) : Particles < Flame >( es )
{

}

Fire::Fire( EntityState& es ) : Emitter < Flame >( es )
{
	target = es.flames;

	position_jitter = Vec2( 8.0, 8.0 );
	velocity_base = Vec2( 0.0, 0.0 );
	velocity_jitter = Vec2( 0.0, 0.0 );
	follow = Vec2( 0.1, 0.1 );

	duration_base = 32;
	duration_jitter = 0;

	frequency_base = -1;
	frequency_jitter = 3;

	gaussian = true;
}

Fires::Fires( EntityState& es ) : Particles < Fire >( es )
{

}
