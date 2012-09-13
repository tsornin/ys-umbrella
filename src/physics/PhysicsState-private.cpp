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
	contact_cache.clear();

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
	rigid_index_contacts();
	rigid_detect_rigid();
	rigid_expire_contacts();
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
PhysicsState::rigid_index_contacts
================================
*/
void PhysicsState::rigid_index_contacts()
{
	// Index contacts by contact identifier
	for ( Contact* ct : contacts ) {
		contact_cache[ ct->key ] = ct;
	}

	// Mark all contacts as expired.
	// We are now holding on to contacts from the previous step.
	// Instead of wiping the contacts, we run collision detection
	// and try to hit the contact cache with new contacts.
	// Contact cache hits will be un-marked before wiping.
	for ( Contact* ct : contacts ) {
		ct->expire_enable = true;
	}
}

/*
================================
PhysicsState::rigid_detect_rigid

Detects all collisions between Rigid bodies.
================================
*/
void PhysicsState::rigid_detect_rigid()
{
	// The integer indexes rigid_shapes
	RD_BruteForce < int > rd;
	for ( unsigned int ii = 0; ii < rigid_shapes.size(); ++ii ) {
		Rigid* rg = rigid_shapes[ii].first.first;
		// int cid = rigid_shapes[ii].first.second;
		Convex& pg = rigid_shapes[ii].second;

		AABB box = pg.getAABB();
		box.fatten( 2.0 );

		// Broad-phase happens here
		for ( int jj : rd.query( box ) ) {
			Rigid* rg2 = rigid_shapes[jj].first.first;
			// int cid2 = rigid_shapes[jj].first.second;
			// Convex& pg2 = rigid_shapes[jj].second;

			// Avoid self-collision
			if ( rg == rg2 ) continue;

			// Avoid sad matrices
			if ( rg->frozen() && rg2->frozen() ) continue;

			// TODO: Masking happens here

			// Narrow-phase
			rigid_caltrops(
				rigid_shapes[ii].first, rigid_shapes[ii].second,
				rigid_shapes[jj].first, rigid_shapes[jj].second );
		}

		// Broad-phase happens here
		// Insert after query, so we don't query ourselves.
		rd.insert( box, ii );
	}
}

/*
================================
PhysicsState::rigid_caltrops

Narrow phase.
================================
*/
void PhysicsState::rigid_caltrops(
	ConvexTag& ta, Convex& a, ConvexTag& tb, Convex& b )
{
	// Make sure these shapes are overlapping
	auto sat = Convex::sat( a, b );
	if ( ! sat.first ) return;

	// Caltrop measurements
	Vec2 caltrop_unit( sat.second * 2.0 );
	Scalar caltrop_length = caltrop_unit.normalize();

	// Y-wall (with minimum overlap shadows)
	Vec2 y = sat.second.unit();
	Wall wy( Vec2(0), y.lperp() );

	// X-wall
	Vec2 x = y.rperp();
	Wall wx( Vec2(0), x.lperp() );

	// Produce the AABBs for the basis ( n.rperp(), n )
	int na = a.points.size();
	auto sya = wy.shadow( a );
	auto sxa = wx.shadow( a );

	int nb = b.points.size();
	auto syb = wy.shadow( b );
	auto sxb = wx.shadow( b );

	// Caltrops on B against A
	caltrop_unit = -caltrop_unit;
	for ( int ib = 0; ib < nb; ++ib ) {
		Vec2& pb = b.points[ ib ];

		// Y-culling
		Scalar sypb = wy.shadow( pb );
		if ( sypb > sya.second ) continue;

		// X culling
		Scalar sxpb = wx.shadow( pb );
		if ( sxpb < sxa.first || sxa.second < sxpb ) continue;

		// This is the caltrop
		Ray r( pb - caltrop_unit * caltrop_length, caltrop_unit );

		// Fire the caltrop at each segment
		for ( int ia = 0; ia < na; ++ia ) {
			Vec2& n = a.normals[ ia ];
			if ( caltrop_unit * n > 0 ) continue;

			Vec2& p = a.points[ ia ];
			Vec2& q = a.points[ (ia+1) % na ];
			Segment s( p, q );

			auto rxs = r.intersects( s );
			if ( ! rxs.first ) continue;
			// We actually found a Ray-Convex intersection here,
			// so from now on all filters use break instead of continue.

			if ( rxs.second > caltrop_length ) break;

			// Only admit caltrops whose endpoints project onto the segment
			if ( Wall( p, n.lperp() ).contains( pb ) ) break;
			if ( Wall( q, n.rperp() ).contains( pb ) ) break;

			// Generate unique key for this Contact
			ContactKey key;
				key.a.pid = ta.first->pid;
				key.a.cid = ta.second;
				key.a.fid = ia;
				key.b.pid = tb.first->pid;
				key.b.cid = tb.second;
				key.b.fid = ib;

			// Hit the Contact cache or make a new Contact
			Contact* ct;
			bool cached;
			auto find = contact_cache.find( key );
			if ( find != contact_cache.end() ) {
				ct = find->second;
				ct->expire_enable = false;
				cached = true;
			}
			else {
				ct = createContact( ta.first, tb.first );
				ct->key = key;
				cached = false;
			}

			Wall w( p, n );
			ct->overlap = - w.distance( pb );
			ct->normal = w.normal;
			ct->a_p = w.nearest( pb );
			ct->b_p = pb;

			// Compute local_lambda after writing to ct
			// TODO: the logic here could be a lot better
			if ( !cached ) {
				ct->lambda = ct->local_lambda();
			}

			ct->ft->normal_lambda = ct->lambda;
			ct->ft->tangent = w.normal.lperp();
			ct->ft->a_p = ct->a_p;
			ct->ft->b_p = ct->b_p;

			break; // Only one intersection per caltrop
		}
	}

	// Caltrops on A against B
	caltrop_unit = -caltrop_unit;
	for ( int ia = 0; ia < na; ++ia ) {
		Vec2& pa = a.points[ ia ];

		Scalar sypa = wy.shadow( pa );
		if ( sypa < syb.first ) continue;

		Scalar sxpa = wx.shadow( pa );
		if ( sxpa < sxb.first || sxb.second < sxpa ) continue;

		Ray r( pa - caltrop_unit * caltrop_length, caltrop_unit );

		for ( int ib = 0; ib < nb; ++ib ) {
			Vec2& n = b.normals[ ib ];
			if ( caltrop_unit * n > 0 ) continue;

			Vec2& p = b.points[ ib ];
			Vec2& q = b.points[ (ib+1) % nb ];
			Segment s( p, q );

			auto rxs = r.intersects( s );
			if ( ! rxs.first ) continue;

			if ( rxs.second > caltrop_length ) break;

			if ( Wall( p, n.lperp() ).contains( pa ) ) break;
			if ( Wall( q, n.rperp() ).contains( pa ) ) break;

			ContactKey key;
				key.a.pid = tb.first->pid;
				key.a.cid = tb.second;
				key.a.fid = ib;
				key.b.pid = ta.first->pid;
				key.b.cid = ta.second;
				key.b.fid = ia;

			Contact* ct;
			bool cached;
			auto find = contact_cache.find( key );
			if ( find != contact_cache.end() ) {
				ct = find->second;
				ct->expire_enable = false;
				cached = true;
			}
			else {
				ct = createContact( tb.first, ta.first );
				ct->key = key;
				cached = false;
			}

			Wall w( p, n );
			ct->overlap = - w.distance( pa );
			ct->normal = w.normal;
			ct->a_p = w.nearest( pa );
			ct->b_p = pa;

			if ( !cached ) {
				ct->lambda = ct->local_lambda();
			}

			ct->ft->normal_lambda = ct->lambda;
			ct->ft->tangent = w.normal.lperp();
			ct->ft->a_p = ct->a_p;
			ct->ft->b_p = ct->b_p;

			break; // Only one intersection per caltrop
		}
	}
}

/*
================================
PhysicsState::rigid_expire_contacts

Collision detection finds persistent contacts.
After detection, all Contacts still marked as expired are deleted.
================================
*/
void PhysicsState::rigid_expire_contacts()
{
	for ( Contact* ct : contacts ) {
		if ( ct->expired() ) destroyContact( ct );
	}
	contacts.erase( std::remove_if( contacts.begin(), contacts.end(), Expire < Contact* >() ), contacts.end() );
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
		if ( rg->edges.empty() || rg->frozen() ) rg->marked = true;
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
	// NOTE: This shadows this->rgs and this->cts
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
================================
*/
void PhysicsState::rigid_solve_island( RigidGraph& rgg )
{
	// NOTE: This shadows this->rgs and this->cts
	std::vector < Rigid* >& rgs = rgg.first;
	std::vector < Constraint* >& cts = rgg.second;

	// TODO: Reproducibility (platforms)
	// Observed different results on PC vs Mac.

	// TODO: Reproducibility (pointers)
	// If we use raw pointers, we have no guarantee on the order of results of
	// rigid_find_islands (since it uses std::set). Order does matter, since
	// the solver is iterative.
	bool random = false;
	if ( random ) {
		std::random_shuffle( rgs.begin(), rgs.end() );
		std::random_shuffle( cts.begin(), cts.end() );
	}
	else {
		std::sort( rgs.begin(), rgs.end(), PhysicsTags::pid_lt );
		std::sort( cts.begin(), cts.end(), PhysicsTags::pid_lt );
	}

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
		V[i] = rg->getVelocityState();
	}

	// Inverse mass matrix, M
	auto M = std::vector < Vec3 >( n );
	for ( int i = 0; i < n; ++i ) {
		Rigid* rg = rgs[i];
		M[i] = rg->getInverseMass();
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
		Scalar jv =
			J_sp[i].first.dot( V[ J_map[i].first ] ) +
			J_sp[i].second.dot( V[ J_map[i].second ] );
		H[i] = cts[i]->bias( jv ) - jv;
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
	int m = (int) std::ceil( std::sqrt( s + n ) ) * 4;
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

	// Integrate velocity
	// R_c = M F_c
	// V += R_c
	for ( int i = 0; i < n; ++i ) {
		V[i] += F[i].prod( M[i] );
	}

	// Set new velocity
	for ( int i = 0; i < n; ++i ) {
		Rigid* rg = rgs[i];
		rg->setVelocityState( V[i] );
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
		if ( vl->edges.empty() || vl->frozen() ) vl->marked = true;
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
