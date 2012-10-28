#ifndef ENTITY_PARTICLE_TYPES
#define ENTITY_PARTICLE_TYPES

#include "Particles.h"

class Flame : public Particle
{
public:
	Flame( EntityState& es );
};

class Flames : public Particles < Flame >
{
public:
	Flames( EntityState& es );
	virtual void draw( Renderer& rd ) {}
};

class Fire : public Emitter < Flame >
{
public:
	Fire( EntityState& es );
};

class Fires : public Particles < Fire >
{
public:
	Fires( EntityState& es );
	virtual void draw( Renderer& rd ) {}
};

#endif
