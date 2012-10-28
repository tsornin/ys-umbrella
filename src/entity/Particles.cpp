#include "Particles.h"

Particle::Particle( EntityState& es ) : Entity( es ),
	eu( es.createEuler() ),
	frames_elapsed( 0 ),
	life_remaining( 0 )
{
	
}

Particle::~Particle()
{
	es.destroyEuler( eu );
}

bool Particle::expired() const
{
	return life_remaining <= 0;
}

void Particle::update()
{
	frames_elapsed++;
	life_remaining--;
}

AABB Particle::getAABB()
{
	return eu->getAABB();
}

/*
================================
EmitterProperties::normalRandom
================================
*/
double EmitterProperties::normalRandom() const
{
	return gaussian ? grand() : frand() - 0.5;
}
