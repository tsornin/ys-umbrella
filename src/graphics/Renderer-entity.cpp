#include "Renderer.h"
#include "entity/Camera.h"

/*
================================
Renderer::drawCamera

Wrapper around gluLookAt.
Applies the specified Camera's settings to the OpenGL context.
================================
*/
void Renderer::drawCamera( const Camera& cam )
{
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective( cam.fov_y, cam.aspect, cam.z_near, cam.z_far );

	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	// In 2D, the camera always points along the negative Z-axis.
	gluLookAt(
		cam.eu->getX(), cam.eu->getY(), cam.z, // eye
		cam.eu->getX(), cam.eu->getY(), 0, // target
		0, 1, 0 ); // up
}
