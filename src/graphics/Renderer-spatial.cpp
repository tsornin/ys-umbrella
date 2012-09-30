#include "Renderer.h"
#include "spatial/AABB.h"
#include "spatial/Convex.h"

/*
================================
Renderer::drawAABB
================================
*/
void Renderer::drawAABB( const AABB& box )
{
	glLineWidth( 1.0 );
	gl_SetColor( RGBA_RED.alpha( 0.5 ) );

	glBegin( GL_LINE_LOOP );
		gl_SetVertex( box.max );
		gl_SetVertex( Vec2( box.min.x, box.max.y ) );
		gl_SetVertex( box.min );
		gl_SetVertex( Vec2( box.max.x, box.min.y ) );
	glEnd();
}

/*
================================
Renderer::drawConvex
================================
*/
void Renderer::drawConvex( const Convex& pg )
{
	int n = pg.points.size();

	// Transparent fill and normals
	gl_SetColor( RGBA_GRAY.alpha( 0.25 ) );

	// Fill color
	glBegin( GL_POLYGON );
		for ( int i = 0; i < n; ++i ) {
			gl_SetVertex( pg.points[i] );
		}
	glEnd();

	// Draw normals
	glLineWidth( 1.0 );
	glBegin( GL_LINES );
		for ( int i = 0; i < n; ++i ) {
			Vec2 mid = pg.points[i] + pg.points[ (i+1) % n ];
			mid *= 0.5;
			Vec2 nor = mid + pg.normals[i] * 50.0;
			gl_SetVertex( mid );
			gl_SetVertex( nor );
		}
	glEnd();

	// Opaque edges and vertices
	gl_SetColor( RGBA_GRAY );

	// Draw edges
	glBegin( GL_LINE_LOOP );
		for ( int i = 0; i < n; ++i ) {
			gl_SetVertex( pg.points[i] );
		}
	glEnd();

	// Draw vertices
	glPointSize( 4.0 );
	glBegin( GL_POINTS );
		for ( int i = 0; i < n; ++i ) {
			gl_SetVertex( pg.points[i] );
		}
	glEnd();

	// // Draw bounding box
	// drawAABB( pg.getAABB() );
}
