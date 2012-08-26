#include "PhysicsState.h"
#include <queue>
#include <algorithm> // for std::remove_if
#include "spatial/PD_BruteForce.h"
#include "spatial/RD_BruteForce.h"
#include "spatial/Vec3.h" // for solver // TODO: move
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
	relax_verlet_islands();
	//integrate();
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
	// This invalidates old indices.
	rgs.erase( std::remove_if( rgs.begin(), rgs.end(), Expire < Rigid* >() ), rgs.end() );
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

	// Pre-processing
	for ( Verlet* vl : vls ) {
		// Ignore edge-less points
		if ( vl->edges.empty() ) vl->marked = true;

		// Ignore frozen points
		if ( ! vl->linear_enable ) vl->marked = true;

		// Reset all component ID tags
		vl->component_id = -1;
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

	// Start the BFS
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
	rigid_contacts.clear();
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
	// TODO: island indexing
	// Constraint just stores Rigid*.
	/*
		PS::expire
		frame-id set. indexes rgs
		PS::detect_rigid_collisions
		PS::find_rigid_islands
		each island sets component-id and per-island-id.
	*/
	for ( unsigned int i = 0; i < rgs.size(); ++i ) {
		// Store the local ID in the Rigid body,
		// so that Constraint can just store Rigid*.
		rgs[i]->local_id = i;
	}

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
			bool swap; Vec2 p; Wall w;
			if ( ! Convex::sat( pg, pg2, swap, p, w ) ) continue;

			// Correction to B
			Vec2 correction( p, w.nearest( p ) );
			correction *= 2.0;
			if ( swap ) correction = -correction;

			// THE CALTROPS ALGORITHM
			Convex& a = pg;
			Convex& b = pg2;
			int an = a.points.size();
			int bn = b.points.size();

			correction = -correction;

			for ( int i = 0; i < bn; ++i ) {
				int h = i-1; if ( h < 0 ) h += bn;
				// Vec2& m = b.normals[h];
				// Vec2& n = b.normals[i];
				// Vec2 vn = (m+n).unit(); // This breaks digons
				Vec2 vn = correction.unit();
				Vec2& v = b.points[i];

				// Scalar caltrop_length = 50.0; // This is bullshit
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
						Contact c;
							c.a = rg;
							c.b = rg2;
							c.overlap = - w.distance( v );
							c.normal = w.normal;
							c.a_p = w.nearest( v );
							c.b_p = v;
						rigid_contacts.push_back( c );
						break; // Only one intersection.
					}
				}
			}

			correction = -correction;

			for ( int i = 0; i < an; ++i ) {
				int h = i-1; if ( h < 0 ) h += an;
				// Vec2& m = a.normals[h];
				// Vec2& n = a.normals[i];
				// Vec2 vn = (m+n).unit(); // This breaks digons
				Vec2 vn = correction.unit();
				Vec2& v = a.points[i];

				// Scalar caltrop_length = 50.0; // This is bullshit
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
						Contact c;
							c.a = rg2;
							c.b = rg;
							c.overlap = - w.distance( v );
							c.normal = w.normal;
							c.a_p = w.nearest( v );
							c.b_p = v;
						rigid_contacts.push_back( c );
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
	fluid wind (fine precomputed "aura
)remove stokes resistance at physics object level
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
	int s = rigid_contacts.size();
	int n = rgs.size();

	// Compute the Jacobian matrix, J
	auto J_sp_row = std::vector < Vec3 >( s );
	auto J_sp = std::vector < std::vector < Vec3 > >( 2, J_sp_row );

	auto J_map_row = std::vector < int >( s );
	auto J_map = std::vector < std::vector < int > >( 2, J_map_row );

	for ( int i = 0; i < s; ++i ) {
		Contact& c = rigid_contacts[i];

		J_sp[0][i] = Vec3( -c.normal, -(c.a_p - c.a->position)^c.normal );
		J_sp[1][i] = Vec3(  c.normal,  (c.b_p - c.b->position)^c.normal );

		J_map[0][i] = c.a->local_id;
		J_map[1][i] = c.b->local_id;
	}

	// Consider changing the layout of Rigid:
	// Vec3 position, Vec3 velocity; // z is angular

	// Compute the velocity vector, V
	// V = map position_state rgs
	auto V = std::vector < Vec3 >( n );
	for ( int i = 0; i < n; ++i ) {
		Rigid* rg = rgs[i];
		V[i] = Vec3( rg->velocity, rg->angular_velocity );
	}

	// Compute the inverse mass matrix, M
	auto M = std::vector < Vec3 >( n );
	for ( int i = 0; i < n; ++i ) {
		Rigid* rg = rgs[i];
		M[i] = Vec3(
			Vec2( rg->linear_enable ? 1.0 / rg->mass : 0 ),
			rg->angular_enable ? 1.0 / rg->moment : 0 );
	}

	// Compute the constraint velocity vector, H
	// TODO: H is eta in the paper, Eta = bias + J V2bar
	auto H = std::vector < Scalar >( s );
	for ( int i = 0; i < s; ++i ) {
		H[i] =
			J_sp[0][i].dot( V[ J_map[0][i] ] ) +
			J_sp[1][i].dot( V[ J_map[1][i] ] );

		H[i] = -H[i];

		static const Scalar PHYSICS_SLOP = 0.1;
		static const Scalar PHYSICS_BIAS = 0.5;

		Scalar error = rigid_contacts[i].overlap - PHYSICS_SLOP;
		error = error > 0 ? error : 0;
		H[i] += PHYSICS_BIAS * error;
	}

	// Compute B = M J
	auto B_sp = std::vector < std::vector < Vec3 > >( 2, J_sp_row );
	for ( int i = 0; i < s; ++i ) {
		B_sp[0][i] = J_sp[0][i].prod( M[ J_map[0][i] ] );
		B_sp[1][i] = J_sp[1][i].prod( M[ J_map[1][i] ] );
	}

	// Solve for lambda	
	// TODO: a is initialized to B L with warm starting
	auto a = std::vector < Vec3 >( n );
	auto d = std::vector < Scalar >( s );
	auto L = std::vector < Scalar >( s );
	auto L_0 = std::vector < Scalar >( s );
	// L_0 = H;
	// L = L_0;
	// // compute a = B L
	// for ( int i = 0; i < s; ++i ) {
	// 	a[ J_map[0][i] ] += B_sp[0][i] * ( L[i] );
	// 	a[ J_map[1][i] ] += B_sp[1][i] * ( L[i] );
	// }
	// initialize diagonal
	for ( int i = 0; i < s; ++i ) {
		d[i] =
			B_sp[0][i].dot( J_sp[0][i] ) +
			B_sp[1][i].dot( J_sp[1][i] );
	}
	for ( int m = 0; m < 10; ++m ) { // iteration
		for ( int i = 0; i < s; ++i ) {
			int b0 = J_map[0][i];
			int b1 = J_map[1][i];
			Scalar delta = ( H[i] - J_sp[0][i].dot( a[b0] ) - J_sp[1][i].dot( a[b1] ) ) / d[i];
			L_0[i] = L[i];
			Scalar tmp = L_0[i] + delta;
			// TODO: bounds provided by Contact-is-Constraint
			L[i] = tmp > 0.0 ? tmp : 0.0;
			delta = L[i] - L_0[i];
			a[b0] += B_sp[0][i] * delta; // scale
			a[b1] += B_sp[1][i] * delta; // scale
		}
	}

	// Compute F_c = Jt L
	auto F = std::vector < Vec3 >( n, 0 );
	for ( int i = 0; i < s; ++i ) {
		F[ J_map[0][i] ] += J_sp[0][i] * ( L[i] ); // scale
		F[ J_map[1][i] ] += J_sp[1][i] * ( L[i] ); // scale
	}

	// Compute P_c = M F_c
	auto P = std::vector < Vec3 >( n );
	for ( int i = 0; i < n; ++i ) {
		P[i] = F[i].prod( M[i] );
	}

	// Update V += P_c
	for ( int i = 0; i < n; ++i ) {
		V[i] += P[i];
	}

	// Read V back into rigid bodies
	for ( int i = 0; i < n; ++i ) {
		Rigid* rg = rgs[i];
		rg->setVelocity( Vec2( V[i].x, V[i].y ) );
		rg->setAngularVelocity( V[i].z );
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
