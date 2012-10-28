#include "Particle-types.h"

Flame::Flame( EntityState& es ) : Particle( es )
{
	//eu->setGravity( Vec2( 0, 10 ) );
}

Flames::Flames( EntityState& es ) : Particles < Flame >( es )
{

}

Fire::Fire( EntityState& es ) : Emitter < Flame >( es )
{
	target = es.flames;

	// spawn at center
	position_jitter = Vec2( 0.0, 0.0 );
	velocity_base = Vec2( 0.0, 0.0 );
	velocity_jitter = Vec2( 5.0, 5.0 );
	follow = Vec2( 1.0, 1.0 );

	duration_base = 30;
	duration_jitter = 10;

	frequency_base = 2;
	frequency_jitter = 1;

	gaussian = true;
}

Fires::Fires( EntityState& es ) : Particles < Fire >( es )
{

}
