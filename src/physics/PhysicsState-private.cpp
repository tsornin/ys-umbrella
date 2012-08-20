#include "PhysicsState.h"
#include <queue>
#include <algorithm> // for std::remove_if
#include "spatial/PD_BruteForce.h"
#include "spatial/Vec3.h"
#include <iostream>

/*
================================
PhysicsState::step

Steps the simulation.
================================
*/
void PhysicsState::step()
{
	expire();
	find_verlet_islands();
	detect_collisions();
	relax_verlet_islands();
	integrate();
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

Invariant: no Verlet particles are marked
================================
*/
void PhysicsState::find_verlet_islands()
{
	if ( ! dirty_verlet_islands ) return;

	verlet_islands.clear();

	// Undirected connected components algorithm
	for ( Verlet* vl : vls ) {
		if ( vl->marked ) continue;

		VerletGraph vg = mark_connected( vl );

		// Tag every Verlet with the component ID
		int id = verlet_islands.size();
		for ( Verlet* vl : vg.first ) vl->component_id = id;

		verlet_islands.push_back( vg );
	}

	// for ( Verlet* vl : vls ) assert( vl->marked );

	// Post-condition
	for ( Verlet* vl : vls ) {
		vl->marked = false;
	}

	dirty_verlet_islands = false;
}

/*
================================
PhysicsState::mark_connected

Returns all Verlet particles and Distance constraints in
the connected component of the specified Verlet particle.

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
			// We add frozen nodes even if they are marked: this means that
			// we consider frozen nodes with multiple neighbors
			// to belong to multiple connected components (which is correct).
			if ( ! w->linear_enable ) {
				vls.push_back( w );
				w->marked = true;
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
	for ( unsigned int i = 0; i < rgs.size(); ++i ) {
		Rigid* rg = rgs[i];

		// Skip non-colliding Rigid bodies
		if ( rg->mask == 0 ) continue;

		for ( const Convex& pg : rg->shapes ) {
			Convex t( pg );
			t.transform( rg->position, rg->angle );
			rigid_shapes.insert( std::pair < int, Convex >( i, t ) );
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
	// To generate contacts, we place all Rigid body shape vertices
	// into a single point-data and query the point-data with each shape.
	// Since it only queries points, this algorithm fails to detect
	// severely overpenetrating shapes (such as two crossed rectangles).

	// Rigid body vertex potpourri
	PD_BruteForce < std::pair < int, Vec2 > > pd;
	for ( const auto& pair : rigid_shapes ) {
		int i = pair.first;
		const Convex& pg = pair.second;

		// for ( Vec2 v : pg.points ) {
		// 	pd.insert( v, std::pair < int, Vec2 >( i, v ) );
		// }

		int n = pg.points.size();
		for ( int x = 0; x < n; ++x ) {
			const Vec2& p = pg.points[x];
			const Vec2& q = pg.points[ (x+1) % n ];

			Vec2 m = (p+q) * 0.5;
			pd.insert( p, std::pair < int, Vec2 >( i, p ) );
			// pd.insert( m, std::pair < int, Vec2 >( i, m ) ); // DOUBLED IT
		}
	}

	// Query the potpourri with every Convex shape
	for ( const auto& pair : rigid_shapes ) {
		int a_i = pair.first;
		const Convex& pg = pair.second;

		// Spatial partitioning speed-up happens here
		for ( const auto& pair : pd.query( pg.getAABB() ) ) {
			int b_i = pair.first;
			const Vec2& p = pair.second;

			Rigid* a = rgs[ a_i ];
			Rigid* b = rgs[ b_i ];

			// assert( (a==b) == (a_i==b_i) );

			// Skip our own points
			// TODO: We're going to see a lot of this. Problem?
			if ( a_i == b_i ) continue;

			// TODO: collision masking happens here

			// TODO: use bias at p when picking the edge here
			// Vec2 v_a = a->velocity + (p-a->position).lperp() * a->angular_velocity;
			// Vec2 v_b = b->velocity + (p-b->position).lperp() * b->angular_velocity;
			// Vec2 bias = v_a - v_b;
			Vec2 bias = p - b->position;

			auto maybe_correction = pg.correction( p, bias );
			if ( !maybe_correction.first ) continue;

			Vec2 normal = maybe_correction.second.first;
			Scalar overlap = maybe_correction.second.second;
			Vec2 correction = normal * overlap;


			Contact c;
				c.normal = normal;
				c.overlap = overlap;
				c.a = a;
				c.a_i = a_i;
				c.a_p = p + correction;
				c.b = b;
				c.b_i = b_i;
				c.b_p = p;
			rigid_contacts.push_back( c );
		}
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

		J_map[0][i] = c.a_i;
		J_map[1][i] = c.b_i;
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

	// TODO: must apply Rigid wind resistance in apply_wind_forces.
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
