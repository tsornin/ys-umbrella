#include "Renderer.h"
#include "physics/Euler.h"
#include "physics/Verlet.h"
#include "physics/Distance.h"
#include "physics/Angular.h"

/*
================================
Renderer::drawEuler
================================
*/
void Renderer::drawEuler( const Euler& eu )
{

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
