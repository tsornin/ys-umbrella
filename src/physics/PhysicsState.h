#ifndef PHYSICS_STATE_H
#define PHYSICS_STATE_H

#include <list>
#include <vector>
#include "game/BlankState.h" // superclass BlankState
#include "Constants.h"
#include "Euler.h"
#include "Rigid.h"
#include "Verlet.h"
#include "Distance.h"
#include "Angular.h"
#include "Contact.h"
#include "common/MeshOBJ.h"

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
	Rigid* createRigid( const MeshOBJ& obj, RigidType rt = 0 );
	void destroyRigid( Rigid* rg );

	Euler* createEuler( EulerType et = 0 );
	void destroyEuler( Euler* eu );

	Verlet* createVerlet( VerletType vt = 0 );
	void destroyVerlet( Verlet* vl );

	Distance* createDistance( Verlet* a, Verlet* b, DistanceType dt = 0 );
	void destroyDistance( Distance* dc );

	Angular* createAngular( Distance* m, Distance* n );
	void destroyAngular( Angular* ac );

	VerletGraph connected( Verlet* root );

	Verlet* nearestVerlet( const Vec2& p, Scalar r );

private: // Physics timestep
	void step();
		void expire();
		void find_verlet_islands();
			VerletGraph mark_connected( Verlet* root );
		void detect_collisions();
			void clear_collision_data();
			void transform_convex();
			void detect_rigid_collisions();

			// Think about adding eu-rg and vl-rg later.
				// run dser and dsvr here.

			// integrate velocity
				// apply external forces (wind, gravity)
					// V2bar = V1 + M-1 F_ext
				// apply contact forces
					// Form J
					// Form M
					// Form Lambda_bounds
					// C' = J V2bar
					// Eta = beta C + C'
					// B = M-1 Jt
					// Solve Lambda: J B Lambda = Eta
					// Apply contact forces:
					// V2 = V2bar + M-1 Jt Lambda
			// integrate position
				// X2 = X1 + V2
	// RENDER

		void relax_verlet_islands();
		void integrate();
			void integrate_velocity();
				void apply_gravity_forces();
				void apply_wind_forces();
				void apply_contact_forces();
			void integrate_position();

	int nextPID();



private: // Members
	// (next) Physics ID
	int next_pid;

	// Rigid bodies
	std::vector < Rigid* > rgs;
	std::multimap < int, Convex > rigid_shapes; // maps rg->gid to shapes
	std::vector < Contact > rigid_contacts;

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
