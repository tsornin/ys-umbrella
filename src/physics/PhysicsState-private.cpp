#include "PhysicsState.h"
#include <queue>
#include <algorithm> // for std::remove_if
#include "spatial/PD_BruteForce.h"
#include "spatial/RD_BruteForce.h"

// TODO: cleanup sat-caltrops
#include "spatial/Segment.h"
#include "spatial/Ray.h"

/*
================================
PhysicsState::step

Steps the simulation once.
================================
*/
void PhysicsState::step()
{
	expire();
	clear_collision_data();

	rigid_step();
	euler_step();
	verlet_step();
}

/*
================================
PhysicsState::expire

Removes and deletes expired physics objects.

TODO: use std::vector for all physics object collections
================================
*/
template < typename T >
struct Expire {
	bool operator() ( T& t ) {
		if ( t->expired() ) {
			delete t;
			return true;
		}
		return false;
	}
};

void PhysicsState::expire()
{
	rgs.erase( std::remove_if( rgs.begin(), rgs.end(), Expire < Rigid* >() ), rgs.end() );

	eus.remove_if( Expire < Euler* >() );

	vls.remove_if( Expire < Verlet* >() );
	dcs.remove_if( Expire < Distance* >() );
	acs.remove_if( Expire < Angular* >() );
}

/*
================================
PhysicsState::clear_collision_data

Clears all collision data from the last frame.
================================
*/
void PhysicsState::clear_collision_data()
{
	rigid_shapes.clear();

	// TODO: contact caching (?)
	// for ( Contact* ct : cts ) destroyContact( ct );
	// cts.clear();

	// Index contacts by contact identifier
	contact_cache.clear();
	for ( Contact* ct : cts ) {
		// contact_cache[ ct->key() ] = ct;
		contact_cache.insert( std::pair < Identifier, Contact* >( ct->key(), ct ) );
	}
	// Mark all contacts as expired
	for ( Contact* ct : cts ) {
		ct->expire_enable = true;
	}

	rigid_islands.clear();
	verlet_islands.clear();
}

/*
================================
PhysicsState::rigid_step

Steps the Rigid body simulation.
================================
*/
void PhysicsState::rigid_step()
{
	rigid_transform_convex();
	rigid_detect_rigid();
	rigid_find_islands();
	rigid_integrate();
}

/*
================================
PhysicsState::rigid_transform_convex

Makes copies of all Rigid-owned Convex shapes,
transformed into world space and tagged with their owners.
This happens every frame; there are no deletion problems.

TODO: A more sophisticated on-demand transforming scheme?
Is it possible to broad-phase before transforming?

TODO: Masking. Skip zero masks (is the mask in the rigid body, or the shape?)
================================
*/
void PhysicsState::rigid_transform_convex()
{
	for ( Rigid* rg : rgs ) {
		int n = rg->shapes.size();
		for ( int i = 0; i < n; ++i ) {
			Convex xf( rg->shapes[i] );
			xf.transform( rg->position, rg->angle );
			rigid_shapes.push_back( std::pair < ConvexTag, Convex >(
				ConvexTag( rg, i ), xf ) );
		}
	}
}

/*
================================
PhysicsState::rigid_detect_rigid

Detects all collisions between Rigid bodies.

TODO: sat-caltrops cleanup
================================
*/
void PhysicsState::rigid_detect_rigid()
{
	RD_BruteForce < int > rd; // The integer indexes rigid_shapes
	for ( unsigned int i = 0; i < rigid_shapes.size(); ++i ) {
		Rigid* rg = rigid_shapes[i].first.first;
		int cid = rigid_shapes[i].first.second;
		Convex& pg = rigid_shapes[i].second;

		AABB box = pg.getAABB();
		box.fatten( 2.0 );

		// Broadphase happens here
		for ( int j : rd.query( box ) ) {
			Rigid* rg2 = rigid_shapes[j].first.first;
			int cid2 = rigid_shapes[j].first.second;
			Convex& pg2 = rigid_shapes[j].second;

			// Avoid self-collision
			if ( rg == rg2 ) continue;

			if ( rg->linear_enable == false && rg->angular_enable == false
				&& rg2->linear_enable == false && rg2->angular_enable == false )
				continue;

			// TODO: Masking happens here

			// Bodies and shapes: (rg,pg) and (rg2,pg2).

			// THE SEPARATING AXIS THEOREM ALGORITHM
			auto sat = Convex::sat( pg, pg2 );
			if ( ! sat.first ) continue;
			Vec2 correction = sat.second * 2.0;

			// THE CALTROPS ALGORITHM
			Convex& a = pg;
			Convex& b = pg2;
			int an = a.points.size();
			int bn = b.points.size();

			correction = -correction;

			for ( int j = 0; j < bn; ++j ) {
				int h = j-1; if ( h < 0 ) h += bn;
				Vec2 vn = correction.unit();
				Vec2& v = b.points[j];

				Scalar caltrop_length = correction.length();

				// This is the caltrop
				Ray r( v - vn * caltrop_length, vn );

				// The key point of caltrops (fixing triangles)
				// is that v_b doesn't need to be in A to be detected.

				// Fire the ray at each segment.
				for ( int i = 0; i < an; ++i ) {
					Vec2& p = a.points[i];
					Vec2& q = a.points[ (i+1) % an ];
					Vec2& n = a.normals[i];

					if ( vn * n > 0 ) continue;
					Segment s( p, q );
					auto rxs = r.intersects( s );
					if ( rxs.first ) {
						if ( rxs.second > caltrop_length ) break;
						// See if s "contains" v.
						if ( Wall( p, n.lperp() ).contains( v ) ) break;
						if ( Wall( q, n.rperp() ).contains( v ) ) break;
						// Yes, s "contains" v.
						Wall w( p, n );

						Identifier key;
							key.a_pid = rg->pid;
							key.a_cid = cid;
							key.a_fid = i;
							key.b_pid = rg2->pid;
							key.b_cid = cid2;
							key.b_fid = j;
						// TODO: doing a unordered_map "double-find" out of laziness
						if ( contact_cache.find( key ) != contact_cache.end() ) {
						// if ( false ) {
							// Old contact; update it.
							Contact* ct = contact_cache[ key ];
							ct->overlap = - w.distance( v );
							ct->normal = w.normal;
							ct->a_p = w.nearest( v );
							ct->b_p = v;

							ct->expire_enable = false;
						}
						else {
							// New contact; cool story, bro.
							Contact* ct = createContact( rg, rg2 );
								ct->overlap = - w.distance( v );
								ct->normal = w.normal;

								ct->a_p = w.nearest( v );
								ct->a_cid = cid;
								ct->a_fid = i;

								ct->b_p = v;
								ct->b_cid = cid2;
								ct->b_fid = j;
						}

						break; // Only one intersection.
					}
				}
			}

			correction = -correction;

			for ( int j = 0; j < an; ++j ) {
				int h = j-1; if ( h < 0 ) h += an;
				Vec2 vn = correction.unit();
				Vec2& v = a.points[j];

				Scalar caltrop_length = correction.length();

				// This is the caltrop
				Ray r( v - vn * caltrop_length, vn );

				// The key point of caltrops (fixing triangles)
				// is that v_a doesn't need to be in B to be detected.

				// Fire the ray at each segment.
				for ( int i = 0; i < bn; ++i ) {
					Vec2& p = b.points[i];
					Vec2& q = b.points[ (i+1) % bn ];
					Vec2& n = b.normals[i];

					if ( vn * n > 0 ) continue;
					Segment s( p, q );
					auto rxs = r.intersects( s );
					if ( rxs.first ) {
						if ( rxs.second > caltrop_length ) break;
						// See if s "contains" v.
						if ( Wall( p, n.lperp() ).contains( v ) ) break;
						if ( Wall( q, n.rperp() ).contains( v ) ) break;
						// Yes, s "contains" v.
						Wall w( p, n );

						Identifier key;
							key.a_pid = rg2->pid;
							key.a_cid = cid2;
							key.a_fid = i;
							key.b_pid = rg->pid;
							key.b_cid = cid;
							key.b_fid = j;
						if ( contact_cache.find( key ) != contact_cache.end() ) {
						// if ( false ) {
							// Old contact; update it.
							Contact* ct = contact_cache[ key ];
							ct->overlap = - w.distance( v );
							ct->normal = w.normal;
							ct->a_p = w.nearest( v );
							ct->b_p = v;

							ct->expire_enable = false;
						}
						else {
							// New contact; cool story, bro.
							Contact* ct = createContact( rg2, rg );
								ct->overlap = - w.distance( v );
								ct->normal = w.normal;

								ct->a_p = w.nearest( v );
								ct->a_cid = cid2;
								ct->a_fid = i;

								ct->b_p = v;
								ct->b_cid = cid;
								ct->b_fid = j;
						}

						break; // Only one intersection.
					}
				}
			}
		}

		rd.insert( box, i );
	}


	// When creating the contact cache, we marked all Contacts as expired.
	// During detection, we look for persistent contacts and re-mark them as not expired.
	// After detection, all expired Contacts are deleted.
	for ( Contact* ct : cts ) {
		if ( ct->expired() ) destroyContact( ct );
	}
	cts.erase( std::remove_if( cts.begin(), cts.end(), Expire < Contact* >() ), cts.end() );
}

/*
================================
PhysicsState::rigid_find_islands

Finds all connected components of the
{ Rigid body, Constraint } graph.

This is a slightly modified CC algorithm:
	1. Edge-less vertices are not considered components.
	2. "Frozen" vertices belong to as many components as they have edges.

Invariant: no Rigid bodies are marked
================================
*/
void PhysicsState::rigid_find_islands()
{
	rigid_islands.clear();

	// Pre-processing: ignore edge-less and frozen vertices
	for ( Rigid* rg : rgs ) {
		if ( rg->edges.empty() ) rg->marked = true;
		if ( !rg->linear_enable && !rg->angular_enable ) rg->marked = true;
	}

	// Undirected connected components algorithm
	for ( Rigid* rg : rgs ) {
		if ( rg->marked ) continue;
		rigid_islands.push_back( mark_connected( rg ) );
	}

	// Post-condition
	// for ( Rigid* rg : rgs ) assert( rg->marked );

	// Invariant
	for ( Rigid* rg : rgs ) rg->marked = false;
}

/*
================================
PhysicsState::mark_connected

Returns all Rigid bodies and Constraints in
the connected component of the specified Rigid body.

This is a slightly modified CC algorithm (see find_rigid_islands):
We never call mark_connected on frozen or edge-less vertices, and
frozen vertices belong to multiple components.

Post-condition: all Verlet particles returned are marked
================================
*/
RigidGraph PhysicsState::mark_connected( Rigid* root )
{
	RigidGraph ret;
	std::vector < Rigid* >& rgs = ret.first;
	std::vector < Constraint* >& cts = ret.second;

	// Breadth-first search
	std::queue < Rigid*, std::list < Rigid* > > unseen;
	unseen.push( root );
	rgs.push_back( root );
	root->marked = true;

	while ( ! unseen.empty() ) {
		Rigid* v = unseen.front();
		unseen.pop();

		for ( Constraint* ct : v->edges ) {
			// assert( ct->a == v || ct->b == v );
			Rigid* w = ( ct->a == v ) ? ct->b : ct->a;

			// Include and mark, but do not expand, frozen nodes.
			// We add frozen nodes even though they are marked:
			// this means that we consider frozen nodes with multiple
			// neighbors to belong to multiple connected components.
			if ( !w->linear_enable && !w->angular_enable ) {
				rgs.push_back( w );
			}
			// Normal BFS on non-frozen nodes.
			else if ( ! w->marked ) {
				unseen.push( w );
				rgs.push_back( w );
				w->marked = true;
			}

			// This finds all edges twice
			cts.push_back( ct );
		}
	}

	// Remove duplicate edges
	std::sort( cts.begin(), cts.end() );
	cts.erase( std::unique( cts.begin(), cts.end() ), cts.end() );

	// Post-condition
	// for ( Rigid* rg : rgs ) assert( rg->marked );

	return ret;
}

/*
================================
PhysicsState::rigid_integrate
================================
*/
void PhysicsState::rigid_integrate()
{
	rigid_integrate_velocity();
	rigid_integrate_position();
}

/*
================================
PhysicsState::rigid_integrate_velocity
================================
*/
void PhysicsState::rigid_integrate_velocity()
{
	// External forces
	rigid_apply_gravity_forces();
	rigid_apply_wind_forces();

	// Constraint forces
	rigid_solve_islands();
}

/*
================================
PhysicsState::rigid_apply_gravity_forces

TODO: design a gravity system
	global gravity
	per-body gravity multiplier
	gravity regions with PS::gravity( Vec2 )
================================
*/
void PhysicsState::rigid_apply_gravity_forces()
{
	for ( Rigid* rg : rgs ) {
		rg->addVelocity( rg->gravity );
	}
}

/*
================================
PhysicsState::rigid_apply_wind_forces

TODO: design a wind system
	global drag
	vector wind
	per-body wind multiplier
	coarse real-time fluid wind
================================
*/
void PhysicsState::rigid_apply_wind_forces()
{
	for ( Rigid* rg : rgs ) {
		rg->velocity *= rg->linear_damping;
		rg->angular_velocity *= rg->angular_damping;
	}
}

/*
================================
PhysicsState::rigid_solve_islands

Computes and applies constraint forces for each Rigid island.
================================
*/
void PhysicsState::rigid_solve_islands()
{
	for ( RigidGraph& rgg : rigid_islands ) {
		rigid_solve_island( rgg );
	}
}

/*
================================
PhysicsState::rigid_solve_island

Computes and applies constraint forces for the specified Rigid island.
PDF: Interactive Dynamics (Catto 2005)

Invariant: local_id is -1

TODO: Vec3 Rigid::get_velocity_state. set_velocity_state
================================
*/
void PhysicsState::rigid_solve_island( RigidGraph& rgg )
{
	// This shadows this->rgs and this->cts.
	std::vector < Rigid* >& rgs = rgg.first;
	std::vector < Constraint* >& cts = rgg.second;

	// TODO: Reproducibility of simulation
	// Discovered a bug where the island solver gave different but plausible
	// results on every simulation reset. The culprit was std::set. The
	// island solver's use of std::set in find_rigid_islands caused the
	// list of physics objects to come back in an unknown (up to the
	// allocator's pointer values) order. This causes a difference in the
	// simulation because the solver is iterative.
	// 
	// Maybe we should re-think using raw pointers?
	std::sort( rgs.begin(), rgs.end(), PhysicsTags::pid_lt );
	std::sort( cts.begin(), cts.end(), PhysicsTags::pid_lt );

	int n = rgs.size();
	int s = cts.size();

	// Set local_id
	for ( int i = 0; i < n; ++i ) {
		rgs[i]->local_id = i;
	}

	// Velocity vector, V
	auto V = std::vector < Vec3 >( n );
	for ( int i = 0; i < n; ++i ) {
		Rigid* rg = rgs[i];
		V[i] = Vec3( rg->velocity, rg->angular_velocity );
	}

	// Inverse mass matrix, M
	auto M = std::vector < Vec3 >( n );
	for ( int i = 0; i < n; ++i ) {
		Rigid* rg = rgs[i];
		M[i] = Vec3(
			Vec2( rg->linear_enable ? 1.0 / rg->mass : 0 ),
			rg->angular_enable ? 1.0 / rg->moment : 0 );
	}

	// Jacobian matrix, J
	auto J_sp = std::vector < std::pair < Vec3, Vec3 > >( s );
	auto J_map = std::vector < std::pair < int, int > >( s );
	for ( int i = 0; i < s; ++i ) {
		Constraint* ct = cts[i];
		J_sp[i] = ct->jacobian();
		J_map[i] = std::pair < int, int >(
			ct->a->local_id,
			ct->b->local_id );
	}

	// Constraint velocity vector, H (eta)
	auto H = std::vector < Scalar >( s );
	for ( int i = 0; i < s; ++i ) {
		H[i] =
			J_sp[i].first.dot( V[ J_map[i].first ] ) +
			J_sp[i].second.dot( V[ J_map[i].second ] );

		H[i] = -H[i];

		// TODO: Move these into Constants.h
		static const Scalar PHYSICS_SLOP = 0.01;
		static const Scalar PHYSICS_BIAS = 0.1;

		// TODO: Is physics slop part of the error?
		Scalar error = -cts[i]->eval() - PHYSICS_SLOP;
		if ( error < 0 ) error = 0;
		H[i] += error * PHYSICS_BIAS;
	}

	// B = M J
	auto B_sp = std::vector < std::pair < Vec3, Vec3 > >( s );
	for ( int i = 0; i < s; ++i ) {
		B_sp[i].first = J_sp[i].first.prod( M[ J_map[i].first ] );
		B_sp[i].second = J_sp[i].second.prod( M[ J_map[i].second ] );
	}

	// Constraint force vector, L
	auto a = std::vector < Vec3 >( n );
	auto d = std::vector < Scalar >( s );
	auto L = std::vector < Scalar >( s );
	auto L_0 = std::vector < Scalar >( s );
	// Warm starting
	for ( int i = 0; i < s; ++i ) {
		L_0[i] = cts[i]->lambda;
	}
	L = L_0;
	// Initialize a = B L
	for ( int i = 0; i < s; ++i ) {
		a[ J_map[i].first ] += B_sp[i].first * L[i]; // scale
		a[ J_map[i].second ] += B_sp[i].second * L[i]; // scale
	}
	// Initialize diagonal
	for ( int i = 0; i < s; ++i ) {
		d[i] =
			B_sp[i].first.dot( J_sp[i].first ) +
			B_sp[i].second.dot( J_sp[i].second );
	}
	// Estimate the number of iterations needed
	int m = (int) std::ceil( std::sqrt( s*2 ) );
	// Solve for L with Projected Gauss-Seidel
	for ( int j = 0; j < m; ++j ) {
		for ( int i = 0; i < s; ++i ) {
			int b1 = J_map[i].first;
			int b2 = J_map[i].second;
			Scalar delta = ( H[i] - (
				J_sp[i].first.dot( a[b1] ) +
				J_sp[i].second.dot( a[b2] ) ) ) / d[i];
			L_0[i] = L[i];
			Scalar tmp =  L_0[i] + delta;
			auto bounds = cts[i]->bounds();
			clamp( tmp, bounds.first, bounds.second );
			L[i] = tmp;
			delta = L[i] - L_0[i];
			a[b1] += B_sp[i].first * delta; // scale
			a[b2] += B_sp[i].second * delta; // scale
		}
	}
	// Store new lambdas
	for ( int i = 0; i < s; ++i ) {
		cts[i]->lambda = L[i];
	}

	// Compute F_c = Jt L
	auto F = std::vector < Vec3 >( n );
	for ( int i = 0; i < s; ++i ) {
		F[ J_map[i].first ] += J_sp[i].first * L[i]; // scale
		F[ J_map[i].second ] += J_sp[i].second * L[i]; // scale
	}

	// Update V += R_c, R_c = M F_c
	for ( int i = 0; i < n; ++i ) {
		V[i] += F[i].prod( M[i] );
	}

	// Set new velocity
	for ( int i = 0; i < n; ++i ) {
		Rigid* rg = rgs[i];
		rg->setVelocity( Vec2( V[i].x, V[i].y ) );
		rg->setAngularVelocity( V[i].z );
	}

	// Clear local_id
	for ( int i = 0; i < n; ++i ) {
		rgs[i]->local_id = -1;
	}
}

/*
================================
PhysicsState::rigid_integrate_position
================================
*/
void PhysicsState::rigid_integrate_position()
{
	for ( Rigid* rg : rgs ) {
		rg->update();
	}
}

/*
================================
PhysicsState::euler_step

Steps the Euler particle simulation.
================================
*/
void PhysicsState::euler_step()
{
	euler_detect_rigid();
	euler_integrate();
}

/*
================================
PhysicsState::euler_detect_rigid

Detects and resolves all collisions between Euler particles and Rigid bodies.
================================
*/
void PhysicsState::euler_detect_rigid()
{
	// TODO: euler_detect_rigid
}

/*
================================
PhysicsState::euler_integrate
================================
*/
void PhysicsState::euler_integrate()
{
	euler_integrate_velocity();
	euler_integrate_position();
}

/*
================================
PhysicsState::euler_integrate_velocity
================================
*/
void PhysicsState::euler_integrate_velocity()
{
	euler_apply_gravity_forces();
	euler_apply_wind_forces();
}

/*
================================
PhysicsState::euler_apply_gravity_forces
================================
*/
void PhysicsState::euler_apply_gravity_forces()
{
	for ( Euler* eu : eus ) {
		eu->addVelocity( eu->gravity );
	}
}

/*
================================
PhysicsState::euler_apply_wind_forces
================================
*/
void PhysicsState::euler_apply_wind_forces()
{
	// TODO: Euler damping is still in Euler::update
}

/*
================================
PhysicsState::euler_integrate_position
================================
*/
void PhysicsState::euler_integrate_position()
{
	for ( Euler* eu : eus ) {
		eu->update();
	}
}

/*
================================
PhysicsState::verlet_step
================================
*/
void PhysicsState::verlet_step()
{
	verlet_find_islands();
	verlet_detect_rigid();
	verlet_integrate();
}

/*
================================
PhysicsState::verlet_find_islands

Finds all connected components of the
{ Verlet particle, Distance constraint } graph.

This is a slightly modified CC algorithm:
	1. Edge-less vertices are not considered components.
	2. "Frozen" vertices belong to as many components as they have edges.

Invariant: no Verlet particles are marked
================================
*/
void PhysicsState::verlet_find_islands()
{
	// TODO: any freeze toggle should also set this.
	// if ( ! dirty_verlet_islands ) return;

	verlet_islands.clear();

	// Pre-processing: reset all component ID tags
	for ( Verlet* vl : vls ) {
		vl->component_id = -1;
	}

	// Pre-processing: ignore edge-less and frozen vertices
	for ( Verlet* vl : vls ) {
		if ( vl->edges.empty() ) vl->marked = true;
		if ( ! vl->linear_enable ) vl->marked = true;
	}

	// Undirected connected components algorithm
	for ( Verlet* vl : vls ) {
		if ( vl->marked ) continue;
		verlet_islands.push_back( mark_connected( vl ) );
	}

	// Tag every Verlet with its component ID
	int n = verlet_islands.size();
	for ( int i = 0; i < n; ++i ) {
		VerletGraph& vlg = verlet_islands[i];
		for ( Verlet* vl : vlg.first ) vl->component_id = i;
	}

	// Post-condition
	// for ( Verlet* vl : vls ) assert( vl->marked );

	// Invariant
	for ( Verlet* vl : vls ) vl->marked = false;

	dirty_verlet_islands = false;
}

/*
================================
PhysicsState::mark_connected

Returns all Verlet particles and Distance constraints in
the connected component of the specified Verlet particle.

This is a slightly modified CC algorithm (see find_verlet_islands):
We never call mark_connected on frozen or edge-less vertices, and
frozen vertices belong to multiple components.

Post-condition: all Verlet particles returned are marked
================================
*/
VerletGraph PhysicsState::mark_connected( Verlet* root )
{
	VerletGraph ret;
	std::vector < Verlet* >& vls = ret.first;
	std::vector < Distance* >& dcs = ret.second;

	// Breadth-first search
	std::queue < Verlet*, std::list < Verlet* > > unseen;
	unseen.push( root );
	vls.push_back( root );
	root->marked = true;

	while ( ! unseen.empty() ) {
		Verlet* v = unseen.front();
		unseen.pop();

		for ( Distance* dc : v->edges ) {
			// assert( dc->a == v || dc->b == v );
			Verlet* w = ( dc->a == v ) ? dc->b : dc->a;

			// Include and mark, but do not expand, frozen nodes.
			// We add frozen nodes even though they are marked:
			// this means that we consider frozen nodes with multiple
			// neighbors to belong to multiple connected components.
			if ( ! w->linear_enable ) {
				vls.push_back( w );
			}
			// Normal BFS on non-frozen nodes.
			else if ( ! w->marked ) {
				unseen.push( w );
				vls.push_back( w );
				w->marked = true;
			}

			// This finds all edges twice
			dcs.push_back( dc );
		}
	}

	// Remove duplicate edges
	std::sort( dcs.begin(), dcs.end() );
	dcs.erase( std::unique( dcs.begin(), dcs.end() ), dcs.end() );

	// Post-condition
	// for ( Verlet* vl : vls ) assert( vl->marked );

	return ret;
}

/*
================================
PhysicsState::verlet_detect_rigid
================================
*/
void PhysicsState::verlet_detect_rigid()
{
	// TODO: verlet_detect_rigid
}

/*
================================
PhysicsState::verlet_integrate
================================
*/
void PhysicsState::verlet_integrate()
{
	// TODO: this should be in a function
	for ( Verlet* vl : vls ) {
		vl->addPosition( vl->gravity );
	}

	verlet_solve_islands();
	verlet_integrate_position();
}

/*
================================
PhysicsState::verlet_solve_islands
================================
*/
void PhysicsState::verlet_solve_islands()
{
	for ( VerletGraph& vlg : verlet_islands ) {
		verlet_solve_island( vlg );
	}
}

/*
================================
PhysicsState::verlet_solve_island
================================
*/
void PhysicsState::verlet_solve_island( VerletGraph& vlg )
{
	// std::vector < Verlet* >& vls = vlg.first;
	std::vector < Distance* >& dcs = vlg.second;

	// Estimate the number of iterations needed
	int m = (int) std::ceil( std::sqrt( dcs.size() ) );

	// TODO: Relax distance constraints with wall contacts.

	for ( int i = 0; i < m; ++i ) {
		for ( Distance* dc : dcs ) {
			dc->apply();
		}
	}
}

/*
================================
PhysicsState::verlet_integrate_position
================================
*/
void PhysicsState::verlet_integrate_position()
{
	// TODO: Verlet damping is still in Verlet::update
	for ( Verlet* vl : vls ) {
		vl->update();
	}
}


/*
================================
PhysicsState::nextPID
================================
*/
int PhysicsState::nextPID()
{
	return next_pid++;
}
