#include "Renderer.h"
#include "physics/Euler.h"
#include "physics/Rigid.h"
#include "physics/Verlet.h"
#include "physics/Distance.h"
#include "physics/Angular.h"
#include "physics/Contact.h"

/*
================================
mass_diameter

Returns the graphical diameter for a point-mass of the specified mass.
================================
*/
static Scalar mass_diameter( Scalar mass )
{
	static const Scalar MASS_DIAMETER_MIN = 4.0;
	static const Scalar MASS_DIAMETER_MAX = 64.0;

	Scalar d = MASS_DIAMETER_MIN;
	d += std::pow( mass, 0.33 ) * 0.5;
	if ( d > MASS_DIAMETER_MAX ) d = MASS_DIAMETER_MAX;
	return d;
}

/*
================================
Renderer::drawEuler
================================
*/
void Renderer::drawEuler( const Euler& eu )
{
	Scalar d = mass_diameter( eu.mass );

	glPointSize( d );
	gl_SetColor( RGBA_WHITE );
	glBegin( GL_POINTS );
		gl_SetVertex( eu.position );
	glEnd();

	if ( !eu.linear_enable ) {
		glPointSize( d/2 );
		gl_SetColor( RGBA_BLACK );
		glBegin( GL_POINTS );
			gl_SetVertex( eu.position );
		glEnd();
	}
}

/*
================================
Renderer::drawRigid
================================
*/
void Renderer::drawRigid( const Rigid& rg )
{
	int n = rg.shapes.size();

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
		glTranslatef( rg.position.x, rg.position.y, 0 );
		glRotatef( to_degrees( rg.angular_position ), 0, 0, 1 );

		for ( int i = 0; i < n; ++i ) {
			drawConvex( rg.shapes[i] );
		}
	glPopMatrix();

	// drawAABB( rg.getAABB() );

	// Draw centroid
	Scalar d = mass_diameter( rg.mass );

	glPointSize( d );
	gl_SetColor( RGBA_WHITE );
	glBegin( GL_POINTS );
		gl_SetVertex( rg.position );
	glEnd();

	if ( !rg.angular_enable ) {
		glPointSize( d * 0.8 );
		gl_SetColor( RGBA_BLACK );
		glBegin( GL_POINTS );
			gl_SetVertex( rg.position );
		glEnd();
	}

	glPointSize( d * 0.6 );
	gl_SetColor( RGBA_WHITE );
	glBegin( GL_POINTS );
		gl_SetVertex( rg.position );
	glEnd();

	if ( !rg.linear_enable ) {
		glPointSize( d * 0.4 );
		gl_SetColor( RGBA_BLACK );
		glBegin( GL_POINTS );
			gl_SetVertex( rg.position );
		glEnd();
	}
}

/*
================================
Renderer::drawVerlet
================================
*/
void Renderer::drawVerlet( const Verlet& vl )
{
	Scalar d = mass_diameter( vl.mass );

	glPointSize( d );
	gl_SetColor( RGBA_WHITE );
	glBegin( GL_POINTS );
		gl_SetVertex( vl.position );
	glEnd();

	if ( !vl.linear_enable ) {
		glPointSize( d/2 );
		gl_SetColor( RGBA_BLACK );
		glBegin( GL_POINTS );
			gl_SetVertex( vl.position );
		glEnd();
	}
}

/*
================================
Renderer::drawDistance
================================
*/
void Renderer::drawDistance( const Distance& dc )
{
	glLineWidth( 1.0 );
	gl_SetColor( RGBA_WHITE );
	glBegin( GL_LINES );
		gl_SetVertex( dc.a->position );
		gl_SetVertex( dc.b->position );
	glEnd();
}

/*
================================
Renderer::drawAngular
================================
*/
void Renderer::drawAngular( const Angular& ac )
{

}

/*
================================
Renderer::drawContact
================================
*/
void Renderer::drawContact( const Contact& ct )
{
	// Force
	gl_SetColor( RGBA_ROSE );
	glLineWidth( 1.0 );
	glBegin( GL_LINES );
		gl_SetVertex( ct.a_p );
		gl_SetVertex( ct.a_p + ct.normal * ct.lambda / ct.b->mass * 50 );
	glEnd();

	// Constraint
	gl_SetColor( RGBA_GREEN );
	glLineWidth( 1.0 );
	glBegin( GL_LINES );
		gl_SetVertex( ct.a_p );
		gl_SetVertex( ct.b_p );
	glEnd();

	// Reference point
	glPointSize( 4.0 );
	glBegin( GL_POINTS );
		gl_SetVertex( ct.a_p );
	glEnd();

}

/*
================================
Renderer::drawFriction
================================
*/
void Renderer::drawFriction( const Friction& ft )
{
	Scalar lambda = ft.normal_lambda * ft.mix_friction();

	// Force
	glLineWidth( 1.0 );
	glBegin( GL_LINES );
		// Normal force limit
		gl_SetColor( RGBA_ROSE.alpha( 0.5 ) );
		gl_SetVertex( ft.p - ft.tangent * lambda / ft.b->mass * 50 );
		gl_SetVertex( ft.p + ft.tangent * lambda / ft.b->mass * 50 );
		// Force
		gl_SetColor( RGBA_ROSE );
		gl_SetVertex( ft.p );
		gl_SetVertex( ft.p + ft.tangent * ft.lambda / ft.b->mass * 50 );
	glEnd();

	// Reference point
	gl_SetColor( RGBA_GREEN );
	glPointSize( 4.0 );
	glBegin( GL_POINTS );
		gl_SetVertex( ft.p );
	glEnd();
}
