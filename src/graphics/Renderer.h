#ifndef GRAPHICS_RENDERER_H
#define GRAPHICS_RENDERER_H

#include "SDL.h"
#include "SDL_opengl.h"
#include "Color.h"
#include "spatial/Vec2.h"

/*
================================
Forward declarations
================================
*/

// Spatial
struct AABB;
struct Convex;

// Physics
class Euler;
class Rigid;
class Constraint;
class Contact;
class Friction;
class Verlet;
class Distance;
class Angular;

// Entity
class Camera;

/*
================================
Renderer.

All virtual Entity::draw just call the appropriate Renderer function,
consolidating rendering in one class.

TODO: Debug shouldn't use immediate mode.
================================
*/
class Renderer
{
public:
	void init();
	void cleanup();

	// Must be called every frame.
	void begin();
	void end();

public: // Misc
	void drawAxes( Scalar m );

public: // Spatial
	void drawAABB( const AABB& box );
	void drawConvex( const Convex& pg );

public: // Physics
	void drawEuler( const Euler& eu );
	void drawRigid( const Rigid& rg );
	void drawContact( const Contact& ct );
	void drawFriction( const Friction& ft );
	void drawVerlet( const Verlet& vl );
	void drawDistance( const Distance& dc );
	void drawAngular( const Angular& ac );

public: // Entity
	void drawCamera( const Camera& cam );

private: // Functions

private: // Members

};

/*
================================
OpenGL helper functions
================================
*/

inline void gl_SetVertex( const Vec2& v )
#ifdef SCALAR_USE_DOUBLE_PRECISION
	{ glVertex2dv( (GLdouble*) &v ); }
#else
	{ glVertex2fv( (GLfloat*) &v ); }
#endif

void gl_Enable2D();
void gl_Disable2D();

#endif
