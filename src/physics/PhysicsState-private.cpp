#include "PhysicsState.h"
#include <queue>
#include <algorithm> // for std::remove_if
#include "spatial/PD_BruteForce.h"
#include "spatial/RD_BruteForce.h"

// TODO: these are for sat-caltrops.
#include "spatial/Segment.h"
#include "spatial/Ray.h"

/*
================================
PhysicsState::step

Steps the simulation.
================================
*/
void PhysicsState::step()
{
	integrate_position();

	expire();
	find_verlet_islands();
	detect_collisions();
	find_rigid_islands();
	relax_verlet_islands();

	// TODO: integrate is split so we can see stuff
	// integrate();
	integrate_velocity();
}

/*
================================
Expire

Functor for std::remove_if,
called from PhysicsState::expire
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

/*
================================
PhysicsState::expire

Removes and deletes expired physics objects.
================================
*/
void PhysicsState::expire()
{
	// TODO: indices?
	rgs.erase( std::remove_if( rgs.begin(), rgs.end(), Expire < Rigid* >() ), rgs.end() );
	// TODO: contacts are "collision data" and get removed in collision-data.
	// cts.remove_if( Expire < Contact* >() );

	eus.remove_if( Expire < Euler* >() );

	vls.remove_if( Expire < Verlet* >() );
	dcs.remove_if( Expire < Distance* >() );
	acs.remove_if( Expire < Angular* >() );
}

/*
================================
PhysicsState::find_verlet_islands

Finds all connected components of the
{ Verlet particle, Distance constraint } graph.

This is a slightly modified CC algorithm:
	1. Edge-less vertices are not considered components.
	2. "Frozen" vertices belong to as many components as they have edges.

Invariant: no Verlet particles are marked
================================
*/
void PhysicsState::find_verlet_islands()
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
		VerletGraph& vg = verlet_islands[i];
		for ( Verlet* vl : vg.first ) vl->component_id = i;
	}

	// for ( Verlet* vl : vls ) assert( vl->marked );

	// Post-condition
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
			// this means that we consider frozen nodes with multiple neighbors
			// to belong to multiple connected components.
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

	// for ( Verlet* vl : vls ) assert( vl->marked );

	return ret;
}

/*
================================
PhysicsState::detect_collisions
================================
*/
void PhysicsState::detect_collisions()
{
	clear_collision_data();
	transform_convex();
	detect_rigid_collisions();
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

	// TODO: contact caching goes here (?)
	// TODO: contact is deleted right here. Problem?
	for ( Contact* ct : cts ) destroyContact( ct );
	cts.clear();

	// TODO: why not clear rigid and verlet islands here?
	rigid_islands.clear();
	verlet_islands.clear();

	// verlet_wall_contacts.clear();
}

/*
================================
PhysicsState::transform_convex

Makes copies of all Rigid-owned Convex shapes,
transformed into world space and tagged with owners.
This happens every frame; there are no deletion problems.

TODO: A more sophisticated on-demand transforming scheme?
================================
*/
void PhysicsState::transform_convex()
{
	for ( Rigid* rg : rgs ) {
		// Skip non-colliding Rigid bodies
		if ( rg->mask == 0 ) continue;

		for ( const Convex& pg : rg->shapes ) {
			Convex t( pg );
			t.transform( rg->position, rg->angle );
			rigid_shapes.push_back( std::pair < Rigid*, Convex >( rg, t ) );
		}
	}
}

/*
================================
PhysicsState::detect_rigid_collisions
================================
*/
void PhysicsState::detect_rigid_collisions()
{
	RD_BruteForce < int > rd; // The integer indexes rigid_shapes
	for ( unsigned int i = 0; i < rigid_shapes.size(); ++i ) {
		Rigid* rg = rigid_shapes[i].first;
		Convex& pg = rigid_shapes[i].second;

		AABB box = pg.getAABB();
		box.fatten( 2.0 );

		// Broadphase happens here
		for ( int j : rd.query( box ) ) {
			Rigid* rg2 = rigid_shapes[j].first;
			Convex& pg2 = rigid_shapes[j].second;

			// Avoid self-collision
			if ( rg == rg2 ) continue;

			if ( rg->linear_enable == false
				&& rg->angular_enable == false
				&& rg2->linear_enable == false
				&& rg2->angular_enable == false ) continue;

			// Masking happens here

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

			for ( int i = 0; i < bn; ++i ) {
				int h = i-1; if ( h < 0 ) h += bn;
				Vec2 vn = correction.unit();
				Vec2& v = b.points[i];

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
						Contact* ct = createContact( rg, rg2 );
							ct->overlap = - w.distance( v );
							ct->normal = w.normal;
							ct->a_p = w.nearest( v );
							ct->b_p = v;
						break; // Only one intersection.
					}
				}
			}

			correction = -correction;

			for ( int i = 0; i < an; ++i ) {
				int h = i-1; if ( h < 0 ) h += an;
				Vec2 vn = correction.unit();
				Vec2& v = a.points[i];

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
						Contact* ct = createContact( rg2, rg );
							ct->overlap = - w.distance( v );
							ct->normal = w.normal;
							ct->a_p = w.nearest( v );
							ct->b_p = v;
						break; // Only one intersection.
					}
				}
			}
		}

		rd.insert( box, i );
	}
}

/*
================================
PhysicsState::find_rigid_islands

Finds all connected components of the
{ Rigid body, Constraint } graph.

This is a slightly modified CC algorithm:
	1. Edge-less vertices are not considered components.
	2. "Frozen" vertices belong to as many components as they have edges.

Invariant: no Rigid bodies are marked
================================
*/
void PhysicsState::find_rigid_islands()
{
	rigid_islands.clear();

	// Pre-processing: reset all component ID tags
	// for ( Rigid* rg : rgs ) {
	// 	rg->component_id = -1;
	// }

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

	// TODO: why do we need this for Rigid?
	// int n = rigid_islands.size();
	// for ( int i = 0; i < n; ++i ) {
	// 	RigidGraph& ig = rigid_islands[i];
	// 	for ( Rigid* rg : ig.first ) rg->component_id = i;
	// }

	// for ( Rigid* rg : rgs ) assert( rg->marked );

	// Post-condition
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
			// this means that we consider frozen nodes with multiple neighbors
			// to belong to multiple connected components.
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

	// for ( Rigid* rg : rgs ) assert( rg->marked );

	return ret;
}

/*
================================
PhysicsState::relax_verlet_islands

Applies relaxation to each connected component of Verlet particles.

Repeatedly applies distance constraints between Verlet particles,
converging toward the solution and implicitly modifying velocity.
================================
*/
void PhysicsState::relax_verlet_islands()
{
	for ( VerletGraph vg : verlet_islands ) {
		// std::vector < Verlet* >& vls = vg.first;
		std::vector < Distance* >& dcs = vg.second;

		// Estimate the number of iterations needed
		int m = (int) std::ceil( std::sqrt( dcs.size() ) );

		// Relax distance constraints with wall contacts
		for ( int i = 0; i < m; ++i ) {
			for ( Distance* dc : dcs ) {
				dc->apply();
			}

			// TODO: equal_range can be moved outside one for block.
			// Every component has at least one vertex
			// int id = vls.front()->component_id;
			// auto range = verlet_wall_contacts.equal_range( id );
			// for ( auto it = range.first; it != range.second; ++it ) {
			// 	// Seems like this works best with biased Convex::contains
			// 	WallContact& wc = it->second;
			// 	Verlet* vl = wc.first;
			// 	Wall& w = wc.second;

			// 	auto ret = w.contains( vl->getPosition() );
			// 	if ( ret.first ) vl->addPosition( ret.second );
			// }
		}
	}
}

/*
================================
PhysicsState::integrate

Applies forces to velocity and velocity to position, creating collisions.
================================
*/
void PhysicsState::integrate()
{
	integrate_velocity();
	integrate_position();
}

/*
================================
PhysicsState::integrate_velocity

Applies forces (gravity, wind) to velocities
in preparation for the position update.
================================
*/
void PhysicsState::integrate_velocity()
{
	apply_gravity_forces();
	apply_wind_forces();
	apply_contact_forces();
}

/*
================================
PhysicsState::apply_gravity_forces

TODO:
design a gravity system
	0. global gravity
	1. per-particle gravity
	2. getGravityAt - gravity regions
stop manually applying gravity at Entity level
================================
*/
void PhysicsState::apply_gravity_forces()
{
	for ( Rigid* rg : rgs ) rg->addVelocity( rg->gravity );

	for ( Euler* eu : eus ) eu->addVelocity( eu->gravity );

	for ( Verlet* vl : vls ) vl->addPosition( vl->gravity );
}

/*
================================
PhysicsState::apply_wind_forces

TODO:
design a wind system
	global drag
	vector wind
	fluid wind (coarse realtime)
	fluid wind (fine precomputed "aura)
remove stokes resistance at physics object level
================================
*/
void PhysicsState::apply_wind_forces()
{
	// Apply wind to bodies using PhysicsState::getWindAt.
	//velocity = Vec2::interp( velocity, wind(position), linear_damping )

	// TODO: Wind forces to Rigid are a bit different.
	// 2. Add forces per projected Convex?
	for ( Rigid* rg : rgs ) {
		rg->velocity *= rg->linear_damping;
		rg->angular_velocity *= rg->angular_damping;
	}
}

/*
================================
PhysicsState::apply_contact_forces

Interactive Dynamics (Catto 2005)
================================
*/
void PhysicsState::apply_contact_forces()
{
	solve_rigid_islands();
}

/*
================================
PhysicsState::solve_rigid_islands
================================
*/
void PhysicsState::solve_rigid_islands()
{
	for ( RigidGraph& rgg : rigid_islands ) {
		solve_rigid_island( rgg );
	}
}

/*
================================
PhysicsState::solve_rigid_island

Computes and applies constraint forces for an island of Rigid bodies.

PDF: Interactive Dynamics (Catto 2005)

TODO:
Consider storing Rigid position and velocity as Vec3 to begin with.
================================
*/
void PhysicsState::solve_rigid_island( RigidGraph& rgg )
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
		static const Scalar PHYSICS_SLOP = 0.1;
		static const Scalar PHYSICS_BIAS = 0.5;

		// TODO: Is physics slop part of the error?
		// Contact* ct = (Contact*) cts[i];
		Scalar error = -cts[i]->eval() - PHYSICS_SLOP;
		// Scalar error = ct->overlap - PHYSICS_SLOP;
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
	// TODO: a = B L with warm starting
	auto a = std::vector < Vec3 >( n );
	auto d = std::vector < Scalar >( s );
	auto L = std::vector < Scalar >( s );
	auto L_0 = std::vector < Scalar >( s );
	// Initialize diagonal
	for ( int i = 0; i < s; ++i ) {
		d[i] =
			B_sp[i].first.dot( J_sp[i].first ) +
			B_sp[i].second.dot( J_sp[i].second );
	}
	// Estimate the number of iterations needed
	int m = (int) std::ceil( std::sqrt( s*2 ) );
	// Solve for L
	// TODO: Store the results of virtual Constraint::bounds
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
PhysicsState::integrate_position

Applies velocities to positions, possibly creating collisions.
================================
*/
void PhysicsState::integrate_position()
{
	for ( Rigid* rg : rgs ) rg->update();
	for ( Euler* eu : eus ) eu->update();
	for ( Verlet* vl : vls ) vl->update();
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
