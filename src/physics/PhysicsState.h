#ifndef PHYSICS_STATE_H
#define PHYSICS_STATE_H

#include <list>
#include <vector>
#include "game/BlankState.h" // superclass BlankState
#include "common/MeshOBJ.h"
#include "Constants.h"
#include "Euler.h"
#include "Rigid.h"
#include "Verlet.h"
#include "Distance.h"
#include "Angular.h"
#include "Contact.h"

typedef std::pair <
	std::vector < Verlet* >,
	std::vector < Distance* > > VerletGraph;

typedef std::pair <
	std::vector < Rigid* >,
	std::vector < Constraint* > > RigidGraph;

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
	Rigid* createRigid( const MeshOBJ& obj, RigidType rt = 0 );
	void destroyRigid( Rigid* rg );

	Contact* createContact( Rigid* a, Rigid* b );
	void destroyContact( Contact* ct );
	
	Euler* createEuler( EulerType et = 0 );
	void destroyEuler( Euler* eu );

	Verlet* createVerlet( VerletType vt = 0 );
	void destroyVerlet( Verlet* vl );

	Distance* createDistance( Verlet* a, Verlet* b, DistanceType dt = 0 );
	void destroyDistance( Distance* dc );

	Angular* createAngular( Distance* m, Distance* n );
	void destroyAngular( Angular* ac );

	Verlet* nearestVerlet( const Vec2& p, Scalar r );

private: // Physics timestep
	void step();
		// TODO: index reset
		void expire();

		void find_verlet_islands();
			VerletGraph mark_connected( Verlet* root );

		void detect_collisions();
			void clear_collision_data();
			void transform_convex();
			void detect_rigid_collisions();

		void find_rigid_islands();
			RigidGraph mark_connected( Rigid* root );

		// TODO: this is the same as "apply_contact_forces for rigid bodies"
		void relax_verlet_islands();

		void integrate();
			void integrate_velocity();
				void apply_gravity_forces();
				void apply_wind_forces();
				void apply_contact_forces();
					void solve_rigid_islands();
					void solve_rigid_island( RigidGraph& rgg );
			void integrate_position();

	int nextPID();



private: // Members
	// (next) Physics ID
	int next_pid;

	// Rigid bodies
	std::vector < Rigid* > rgs;
	std::vector < Contact* > cts;
	std::vector < RigidGraph > rigid_islands;
	std::vector < std::pair < Rigid*, Convex > > rigid_shapes;

	// Euler particles
	std::list < Euler* > eus;

	// Verlet particles
	std::list < Verlet* > vls;
	std::list < Distance* > dcs;
	std::list < Angular* > acs;
	// Connected-components analysis of Verlet-Distance graph
	bool dirty_verlet_islands;
	std::vector < VerletGraph > verlet_islands;
};

#endif
