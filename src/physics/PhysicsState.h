#ifndef PHYSICS_STATE_H
#define PHYSICS_STATE_H

#include <list>
#include <vector>
#include <unordered_map>
#include "game/BlankState.h" // superclass BlankState
#include "common/MeshOBJ.h"
#include "Constants.h"
#include "PhysicsTags.h"
#include "PhysicsGraph.h"
#include "Euler.h"
#include "Rigid.h"
#include "Verlet.h"
#include "Distance.h"
#include "Angular.h"
#include "Friction.h"
#include "Contact.h"

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

public: // Physics engine - lifecycle
	Rigid* createRigid( const MeshOBJ& obj );
	Rigid* createRigid();
	void destroyRigid( Rigid* rg );

	Friction* createFriction( Rigid* a, Rigid* b );
	void destroyFriction( Friction* ft );

	Contact* createContact( Rigid* a, Rigid* b );
	void destroyContact( Contact* ct );

	Euler* createEuler();
	void destroyEuler( Euler* eu );

	Verlet* createVerlet();
	void destroyVerlet( Verlet* vl );

	Distance* createDistance( Verlet* a, Verlet* b );
	void destroyDistance( Distance* dc );

	Angular* createAngular( Distance* m, Distance* n );
	void destroyAngular( Angular* ac );

public: // Physics engine - stuff
	Verlet* nearestVerlet( const Vec2& p, Scalar r );
	Rigid* nearestRigid( const Vec2& p );

	// RigidIsland island( Rigid* rg );
	// VerletIsland island( Verlet* vl );

private: // Physics timestep
	typedef std::pair < Rigid*, int > ConvexTag;
	typedef PhysicsGraph < Rigid, Constraint >::Island RigidIsland;
	typedef PhysicsGraph < Verlet, Distance >::Island VerletIsland;

	void step();
		void expire();
		void clear_collision_data();

		void rigid_step();
			void rigid_transform_convex();
			void rigid_index_contacts();
			void rigid_detect_rigid();
				void rigid_caltrops(
					ConvexTag& ta, Convex& a,
					ConvexTag& tb, Convex& b );
			void rigid_expire_contacts();
			void rigid_find_islands();
				// RigidGraph mark_connected( Rigid* root );
			void rigid_integrate();
				void rigid_integrate_velocity();
					void rigid_apply_gravity_forces();
					void rigid_apply_wind_forces();
					void rigid_solve_islands();
						void rigid_solve_island( RigidIsland& rgi );
				void rigid_integrate_position();

		void euler_step();
			void euler_detect_rigid();
			void euler_integrate();
				void euler_integrate_velocity();
					void euler_apply_gravity_forces();
					void euler_apply_wind_forces();
				void euler_integrate_position();

		void verlet_step();
			void verlet_find_islands();
				// VerletGraph mark_connected( Verlet* root );
			void verlet_detect_rigid();
			void verlet_integrate();
				void verlet_solve_islands();
					void verlet_solve_island( VerletIsland& vli );
				void verlet_integrate_position();

	int nextPID();

private: // Members
	// (next) Physics ID
	int next_pid;

	// Rigid bodies
	std::vector < Rigid* > rgs;
	std::vector < Contact* > contacts;
	std::vector < Constraint* > cts;
	std::vector < PhysicsGraph < Rigid, Constraint >::Island > rigid_islands;

	std::vector < std::pair < ConvexTag, Convex > > rigid_shapes;
	std::unordered_map < ContactKey, Contact* > contact_cache;

	// Euler particles
	std::list < Euler* > eus;

	// Verlet particles
	std::vector < Verlet* > vls;
	std::vector < Distance* > dcs;
	std::vector < Angular* > acs;
	std::vector < PhysicsGraph < Verlet, Distance >::Island > verlet_islands;
	bool dirty_verlet_islands;

// protected:
	// Rigid* anchor;
};

#endif
