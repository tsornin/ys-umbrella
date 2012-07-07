#ifndef PHYSICS_STATE_H
#define PHYSICS_STATE_H

#include <list>
#include "game/BlankState.h" // superclass BlankState

class Euler;
enum EulerType : unsigned int;

class Verlet;
enum VerletType : unsigned int;

/*
================================
Physics Engine.

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
	Euler* createEuler( EulerType et );
	void destroyEuler( Euler* eu );

	Verlet* createVerlet( VerletType vt );
	void destroyVerlet( Verlet* vl );

private: // Physics timestep
	void expire();
	void integrate();
		void integrate_velocity();
			void apply_gravity_forces();
			void apply_wind_forces();
		void integrate_position();

	int nextPID();

private: // Members
	// (next) Physics ID
	int next_pid;

	typedef std::pair < EulerType, Euler* > TEuler;
	std::list < TEuler > eus;

	typedef std::pair < VerletType, Verlet* > TVerlet;
	std::list < TVerlet > vls;

	bool dirty_connected_components;
};

#endif
