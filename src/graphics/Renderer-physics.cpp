#include "Renderer.h"
#include "physics/Euler.h"
#include "physics/Rigid.h"
#include "physics/Verlet.h"
#include "physics/Distance.h"
#include "physics/Angular.h"
#include "physics/Contact.h"

/*
================================
Renderer::drawEuler
================================
*/
void Renderer::drawEuler( const Euler& eu )
{
	Scalar d = std::pow( eu.mass, 0.33 ) + 4.0;
	if ( d > 64.0 ) d = 64.0;

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
		glRotatef( to_degrees( rg.angle ), 0, 0, 1 );

		for ( int i = 0; i < n; ++i ) {
			drawConvex( rg.shapes[i] );
		}
	glPopMatrix();

	// drawAABB( rg.getAABB() );

	// Draw centroid
	Scalar d = std::pow( rg.mass, 0.33 ) + 4.0;
	if ( d > 64.0 ) d = 64.0;

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
	// Cube root of mass with minimum size
	Scalar d = std::pow( vl.mass, 0.33 ) + 4.0;

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
	glLineWidth( 1.0 );
	gl_SetColor( RGBA_GREEN );
	glBegin( GL_LINE_STRIP );
		gl_SetVertex( ct.a->position );
		gl_SetVertex( ct.a_p );
		gl_SetVertex( ct.b_p );
		gl_SetVertex( ct.b->position );
	glEnd();
}
