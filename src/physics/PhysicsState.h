#ifndef PHYSICS_STATE_H
#define PHYSICS_STATE_H

#include <list>
#include <vector>
#include "game/BlankState.h" // superclass BlankState
#include "Euler.h"
#include "Verlet.h"
#include "Distance.h"
#include "Angular.h"

typedef unsigned int EulerType;
typedef unsigned int VerletType;
typedef unsigned int DistanceType;

typedef std::pair <
	std::vector < Verlet* >,	
	std::vector < Distance* > > VerletGraph;

/*
================================
Physics engine.

Superclass for any State that uses physics objects.
Manages (creates, destroys, and provides) physics objects.
================================
*/
class PhysicsState : public BlankState
{
public: // State implementation
	virtual ~PhysicsState() {}

	virtual void init( Engine* game );
	virtual void cleanup();

	virtual void input( Engine* game );
	virtual void update( Engine* game );
	virtual void draw( Engine* game );

	virtual void setCaption( std::ostringstream& buffer );

public: // Singleton pattern
	static PhysicsState* Instance();
protected:
	PhysicsState() {}

public: // Physics engine
	Euler* createEuler( EulerType et = 0 );
	void destroyEuler( Euler* eu );

	Verlet* createVerlet( VerletType vt = 0 );
	void destroyVerlet( Verlet* vl );

	Distance* createDistance( Verlet* a, Verlet* b, DistanceType dt = 0 );
	void destroyDistance( Distance* dc );

	Angular* createAngular( Distance* m, Distance* n );
	void destroyAngular( Angular* ac );

	VerletGraph connected( Verlet* root );

private: // Physics timestep
	void expire();
	void integrate();
		void integrate_velocity();
			void apply_gravity_forces();
			void apply_wind_forces();
		void integrate_position();
	void find_connected_components();
	void relax_connected_components();
		VerletGraph mark_connected( Verlet* root );
	void detect_collisions();
		void clear_collision_data();

	int nextPID();

private: // Members
	// (next) Physics ID
	int next_pid;

	typedef std::pair < EulerType, Euler* > TEuler;
	std::list < TEuler > eus;

	typedef std::pair < VerletType, Verlet* > TVerlet;
	std::list < TVerlet > vls;

	typedef std::pair < DistanceType, Distance* > TDistance;
	std::list < TDistance > dcs;

	std::list < Angular* > acs;

	// Connected-components analysis of Verlet-Distance graph
	bool dirty_connected_components;
	std::vector < VerletGraph > connected_components;
};

#endif
