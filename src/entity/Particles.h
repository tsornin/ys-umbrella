#ifndef ENTITY_PARTICLES_H
#define ENTITY_PARTICLES_H

#include "Entity.h" // superclass Entity
#include <set>
#include <vector>
#include <algorithm>
#include "common/Random.h"

/*
================================
Particle
================================
*/
class Particle : public Entity
{
public: // Life cycle
	Particle( EntityState& es );
	virtual ~Particle();

	bool expired() const;

public: // Entity functions
	virtual void update();
	virtual AABB getAABB();

public: // Members
	Euler* eu;
	int frames_elapsed;
	int life_remaining;
};


/*
================================
Particles
================================
*/
template < typename T >
class Particles : public Entity
{
public: // Life cycle
	Particles( EntityState& es );
	virtual ~Particles();

public: // Entity
	virtual void update();
	virtual AABB getAABB();
	virtual void draw( Renderer& rd ) {}

public: // Particles
	T* createParticle();
	void destroyParticle( T* );

public: // Members
	std::vector < T* > ts;
	std::set < T* > user_ts;
};

template < typename T >
struct Expire {
	bool operator () ( T* t ) {
		if ( t->expired() ) {
			delete t;
			return true;
		}
		return false;
	}
};

template < typename T >
Particles < T >::Particles( EntityState& es ) : Entity( es )
{
	
}

template < typename T >
Particles < T >::~Particles()
{
	for ( T* t : ts ) delete t;
}

template < typename T >
void Particles < T >::update()
{
	ts.erase( std::remove_if( ts.begin(), ts.end(), Expire < T >() ), ts.end() );

	for ( T* t : ts ) t->update();
	for ( T* t : user_ts ) t->update();
}

template < typename T >
AABB Particles < T >::getAABB()
{
	return AABB(0);
}

template < typename T >
T* Particles < T >::createParticle()
{
	T* t = new T( es );
	// t->eid = es.nextEID();
	user_ts.insert( t );
	return t;
}

template < typename T >
void Particles < T >::destroyParticle( T* t )
{
	user_ts.remove( t );
	delete t;
}


/*
================================
EmitterProperties
================================
*/
struct EmitterProperties
{
	// Offset spawn position
	// (base position comes from Euler* Particle::eu)
	Vec2 position_jitter;

	// Following speed multiplier
	Vec2 follow;

	// Base + offset spawn velocity
	Vec2 velocity_base, velocity_jitter;

	// Base + offest duration (frames)
	int duration_base, duration_jitter;

	// Base + offset particles per frame
	int frequency_base, frequency_jitter;

	// Uniform distribution if false.
	bool gaussian;

	double normalRandom() const;
};


/*
================================
Emitter
================================
*/
template < typename T >
class Emitter : public Particle, protected EmitterProperties
{
public: // Lifecycle
	Emitter( EntityState& es );
	virtual ~Emitter() {}

public: // Entity
	virtual void input( const InputSet& is );
	virtual void update();
	virtual AABB getAABB() const;

protected: // Members
	Particles < T >* target;
	EmitterProperties ep;
	bool active;
};

template < typename T >
Emitter < T >::Emitter( EntityState& es ) : Particle( es ),
	active( true )
{
	
}

template < typename T >
void Emitter < T >::input( const InputSet& is )
{
	if ( is.rising( IK_A ) ) active = !active;
	if ( is.falling( IK_B ) ) active = false;
	if ( is.get( IK_B ) ) active = true;

	InputSet dummy = is;
	dummy.put( IK_A, false );
	dummy.put( IK_B, false );
	eu->input( dummy );
}

template < typename T >
AABB Emitter < T >::getAABB() const
{
	AABB box = eu->getAABB();
	Vec2 pj = position_jitter * (gaussian ? 2.0 : 0.5);
	box.min -= pj;
	box.max += pj;
	return box;
}

template < typename T >
void Emitter < T >::update()
{
	Particle::update();

	if ( active ) {
		int quantity = frequency_base + irand( frequency_jitter + 1 );
		for ( int m = 0; m < quantity; ++m ) {
			T* t = new T( es );
			target->ts.push_back( t );

			t->frames_elapsed = 0;
			t->life_remaining = duration_base + irand( duration_jitter + 1 );

			Vec2 position_base = this->eu->getPosition();

			t->eu->setPosition( position_base + Vec2::prod(
				Vec2( normalRandom(), normalRandom() ), position_jitter ) );

			t->eu->setVelocity( velocity_base + Vec2::prod(
				Vec2( normalRandom(), normalRandom() ), velocity_jitter ) );

			// Following
			t->eu->addVelocity( Vec2::prod( this->eu->getVelocity(), follow ) );

			// "Temporal anti-aliasing" (an integration over a random timestep)
			t->eu->addPosition( t->eu->getVelocity() * frand() );
		}
	}
}

#endif
