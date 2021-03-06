#include "Renderer.h"

/*
================================
Renderer::init
================================
*/
void Renderer::init()
{
	// Enable antialiasing on points and lines
	glEnable( GL_POINT_SMOOTH );
	glHint( GL_POINT_SMOOTH_HINT, GL_DONT_CARE );
	glEnable( GL_LINE_SMOOTH );
	glHint( GL_LINE_SMOOTH_HINT, GL_DONT_CARE );

	// Enable blending for antialiasing
	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

	// Enable 1D and 2D texture mapping
	glEnable( GL_TEXTURE_1D );
	glEnable( GL_TEXTURE_2D );
}

/*
================================
Renderer::cleanup
================================
*/
void Renderer::cleanup()
{

}

/*
================================
Renderer::begin

Clears renderer state.
Called at the beginning of every frame.
================================
*/
void Renderer::begin()
{
	glClearColor( 0, 0, 0, 0 );
	glClear( GL_COLOR_BUFFER_BIT );
}

/*
================================
Renderer::end

Flushes renderer state.
Called at the end of every frame.
================================
*/
void Renderer::end()
{

}

/*
================================
Renderer::drawAxes
================================
*/
void Renderer::drawAxes( Scalar m )
{
	glLineWidth( 2.0 );
	glBegin( GL_LINES );
		gl_SetColor( RGBA_RED );
		glVertex3f(  m, 0, 0 );
		glVertex3f( -m, 0, 0 );
		gl_SetColor( RGBA_GREEN );
		glVertex3f( 0,  m, 0 );
		glVertex3f( 0, -m, 0 );
		gl_SetColor( RGBA_BLUE );
		glVertex3f( 0, 0,  m );
		glVertex3f( 0, 0, -m );
	glEnd();

	// Hide the intersection
	glPointSize( 4.0 );
	glBegin( GL_POINTS );
		gl_SetColor( RGBA_WHITE );
		glVertex3f( 0, 0, 0 );
	glEnd();
}

/*
================================
gl_Enable2D

Adapted from SDL_GL_Enter2DMode in test/testgl.c.

Enables 2D rendering by setting up orthographic projection
and disabling depth test and lighting.
================================
*/
void gl_Enable2D()
{
	// Disable depth test/lighting.
	glPushAttrib( GL_DEPTH_BUFFER_BIT | GL_LIGHTING_BIT );
	glDisable( GL_DEPTH_TEST );
	glDisable( GL_LIGHTING );

	// Set up orthographic projection
	glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glLoadIdentity();

	GLint view[4];
	glGetIntegerv( GL_VIEWPORT, view );
	glOrtho( view[0], view[0]+view[2], view[1]+view[3], view[1], -1, 1 );

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glLoadIdentity();
}

/*
================================
gl_Disable2D

Adapted from SDL_GL_Leave2DMode in test/testgl.c.

Disables 2D rendering by restoring the previous matrix states.
================================
*/
void gl_Disable2D()
{
	glMatrixMode( GL_PROJECTION );
	glPopMatrix();

	glMatrixMode( GL_MODELVIEW );
	glPopMatrix();

	glPopAttrib();
}
