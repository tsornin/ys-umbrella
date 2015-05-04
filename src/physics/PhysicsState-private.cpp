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
	clear_collision_data();

	rigid_step();
	euler_step();
	verlet_step();
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
			xf.transform( rg->position, rg->angular_position );
			rigid_shapes.push_back( std::pair < ConvexTag, Convex >(
				ConvexTag( rg, i ), xf ) );
		}
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
	for ( unsigned int i = 0; i < rigid_shapes.size(); ++i ) {
		Rigid* rg = rigid_shapes[i].first.first;
		// int cid = rigid_shapes[i].first.second;
		Convex& pg = rigid_shapes[i].second;

		AABB box = pg.getAABB();
		box.fatten( 2.0 );

		// Broad-phase happens here
		for ( int j : rd.query( box ) ) {
			Rigid* rg2 = rigid_shapes[j].first.first;
			// int cid2 = rigid_shapes[j].first.second;
			// Convex& pg2 = rigid_shapes[j].second;

			// Avoid self-collision
			if ( rg == rg2 ) continue;

			// Avoid sad matrices
			if ( rg->frozen() && rg2->frozen() ) continue;

			// Masking
			if ( !(rg->mask & rg2->mask) ) continue;

			// Narrow-phase
			rigid_caltrops(
				rigid_shapes[i].first, rigid_shapes[i].second,
				rigid_shapes[j].first, rigid_shapes[j].second );
		}

		// Broad-phase happens here
		// Insert after query, so we don't query ourselves.
		rd.insert( box, i );
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
			auto cc = createContact( ta.first, tb.first, key );
			Contact* ct = cc.second;

			Wall w( p, n );
			ct->overlap = - w.distance( pb );
			ct->normal = w.normal;
			ct->a_p = w.nearest( pb );
			ct->b_p = pb;

			// Compute "local lambda" for new contacts
			// (after writing to ct)
			if ( ! cc.first ) {
				ct->lambda = ct->local_lambda();

				// Friction optimization
				// TODO: Floating point == seems like a bad idea
				if ( Friction::mix_friction( ta.first->friction, tb.first->friction ) == 0.0 ) {
					ct->ft = 0;
				}
				else {
					// TODO: This logic is kind of bad?
					// 1. createFriction is far from createContact
					// 2. Every narrow-phase will have to create Friction
					ct->ft = createFriction( ta.first, tb.first );
				}
			}

			// Apply friction at the same point on both bodies
			if ( ct->ft ) {
				ct->ft->normal_lambda = ct->lambda;
				ct->ft->tangent = w.normal.lperp();
				ct->ft->p = (ct->a_p + ct->b_p) * 0.5;
			}

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

			auto cc = createContact( tb.first, ta.first, key );
			Contact* ct = cc.second;

			Wall w( p, n );
			ct->overlap = - w.distance( pa );
			ct->normal = w.normal;
			ct->a_p = w.nearest( pa );
			ct->b_p = pa;

			if ( ! cc.first ) {
				ct->lambda = ct->local_lambda();
				if ( Friction::mix_friction( tb.first->friction, ta.first->friction ) == 0.0 ) {
					ct->ft = 0;
				}
				else {
					ct->ft = createFriction( tb.first, ta.first );
				}
			}

			if ( ct->ft ) {
				ct->ft->normal_lambda = ct->lambda;
				ct->ft->tangent = w.normal.lperp();
				ct->ft->p = (ct->b_p + ct->a_p) * 0.5;
			}

			break; // Only one intersection per caltrop
		}
	}
}

// Returns a list of all Contacts (from the contact_cache).
std::list < Contact* > PhysicsState::contacts()
{
	std::list < Contact* > ret;

	for ( auto& pair : contact_cache ) {
		ret.push_back( pair.second );
	}

	return ret;
}

/*
================================
PhysicsState::rigid_expire_contacts

TODO: ???
================================
*/
void PhysicsState::rigid_expire_contacts()
{
	for ( Contact* ct : contacts() ) {
		if ( ct->expired ) {
			destroyContact( ct );
			if ( ct->ft ) {
				destroyFriction( ct->ft );
			}
		}
		else {
			ct->expired = true;
		}
	}
}

/*
================================
PhysicsState::rigid_find_islands

Finds all "islands" (connected components) of the
{ Rigid body, Constraint } graph.
================================
*/
void PhysicsState::rigid_find_islands()
{
	rigid_islands = PhysicsGraph < Rigid, Constraint >::find_islands( rgs );
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
		rg->velocity += rg->gravity;
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
	for ( RigidIsland& rgi : rigid_islands ) {
		rigid_solve_island( rgi );
	}
}

/*
================================
PhysicsState::rigid_solve_island

Computes and applies constraint forces for the specified Rigid island.
PDF: Interactive Dynamics (Catto 2005)
================================
*/
void PhysicsState::rigid_solve_island( RigidIsland& rgi )
{
	// NOTE: This shadows this->rgs and this->cts
	std::vector < Rigid* >& rgs = rgi.first;
	std::vector < Constraint* >& cts = rgi.second;

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

	// Set local ID
	// TODO: Okay, so minor_id out of PhysicsGraph isn't really useful,
	// because frozen objects belong to multiple islands. Without this,
	// forces mess up, and then the -1 index causes weird segfaults.
	for ( int i = 0; i < n; ++i ) {
		rgs[i]->minor_id = i;
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
			ct->a->minor_id,
			ct->b->minor_id );
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

	// Clear local ID
	for ( int i = 0; i < n; ++i ) {
		rgs[i]->minor_id = -1;
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
	PD_BruteForce < Euler* > pd;
	for ( Euler* eu : eus ) {
		if ( !eu->mask ) continue;
		pd.insert( eu->position, eu );
	}

	for ( unsigned int i = 0; i < rigid_shapes.size(); ++i ) {
		Rigid* rg = rigid_shapes[i].first.first;
		// int cid = rigid_shapes[i].first.second;
		Convex& pg = rigid_shapes[i].second;

		// Broad-phase happens here
		for ( Euler* eu : pd.query( pg.getAABB().fatter( 2.0 ) ) ) {
			if ( !(eu->mask & rg->mask) ) continue;

			// TODO: rg->getVelocityAt( eu->position ) is more accurate
			Vec2 bias = eu->velocity - rg->velocity;
			auto c = pg.correction( eu->position, bias );
			if ( c.first ) {
				Vec2& normal = c.second.first;
				Vec2 correction = c.second.first * c.second.second;

				// Squishy position correction
				eu->addPosition( correction * PHYSICS_CONTACT_BIAS );

				// Restitution
				if ( eu->velocity * normal < 0 ) {
					Scalar e = Contact::mix_restitution( eu->bounce, rg->bounce );
					eu->velocity -= eu->velocity.projection_unit( normal ) * (1+e);
				}
			}
		}
	}
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
	for ( Euler* eu : eus ) {
		eu->velocity *= eu->linear_damping;
	}
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

Finds all "islands" (connected components) of the
{ Verlet particle, Distance constraint } graph.
================================
*/
void PhysicsState::verlet_find_islands()
{
	verlet_islands = PhysicsGraph < Verlet, Distance >::find_islands( vls );
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
	for ( VerletIsland& vli : verlet_islands ) {
		verlet_solve_island( vli );
	}
}

/*
================================
PhysicsState::verlet_solve_island
================================
*/
void PhysicsState::verlet_solve_island( VerletIsland& vli )
{
	// std::vector < Verlet* >& vls = vli.first;
	std::vector < Distance* >& dcs = vli.second;

	// Estimate the number of iterations needed
	int m = (int) std::ceil( std::sqrt( dcs.size() ) );

	// TODO: Relax distance constraints with wall contacts.
	// TODO: Sort constraints by PID (like we do with rigid bodies)

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
	return ++next_pid;
}
