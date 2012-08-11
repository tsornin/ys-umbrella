#ifndef GRAPHICS_COLOR_H
#define GRAPHICS_COLOR_H

#include "SDL/SDL_opengl.h"
#include "spatial/Scalar.h"

/*
================================
Represents a RGBA color
as four Scalar coordinates.
================================
*/
struct Color
{
public:
	Scalar r, g, b, a;

public:
	Color() : r(0), g(0), b(0), a(0) {}
	Color( Scalar r, Scalar g, Scalar b, Scalar a = 1 ) :
		r(r), g(g), b(b), a(a) {}

	Color alpha( Scalar a ) const {
		Color ret( *this );
		ret.a = a;
		return ret;
	}
};

/*
================================
Color constants
================================
*/
const Color RGBA_BLACK		( 0.0, 0.0, 0.0 );
const Color RGBA_WHITE		( 1.0, 1.0, 1.0 );
const Color RGBA_GRAY		( 0.5, 0.5, 0.5 );
const Color RGBA_DARK_GRAY	( .25, .25, .25 );
const Color RGBA_LIGHT_GRAY	( .75, .75, .75 );
// Full saturation hues, every 30 degrees
const Color RGBA_RED		( 1.0, 0.0, 0.0 );
const Color RGBA_ORANGE		( 1.0, 0.5, 0.0 );
const Color RGBA_YELLOW		( 1.0, 1.0, 0.0 );
const Color RGBA_CHARTREUSE	( 0.5, 1.0, 0.0 );
const Color RGBA_GREEN		( 0.0, 1.0, 0.0 );
const Color RGBA_AQUA		( 0.0, 1.0, 0.5 );
const Color RGBA_CYAN		( 0.0, 1.0, 1.0 );
const Color RGBA_AZURE		( 0.0, 0.5, 1.0 );
const Color RGBA_BLUE		( 0.0, 0.0, 1.0 );
const Color RGBA_VIOLET		( 0.5, 0.0, 1.0 );
const Color RGBA_MAGENTA	( 1.0, 0.0, 1.0 );
const Color RGBA_ROSE		( 1.0, 0.0, 0.5 );

/*
================================
OpenGL helper functions
================================
*/

inline void gl_SetColor( const Color& c )
#ifdef SCALAR_USE_DOUBLE_PRECISION
	{ glColor4dv( (GLdouble*) &c ); }
#else
	{ glColor4fv( (GLfloat*) &c ); }
#endif

#endif
