#include "Engine.h"
#include <iostream> // for std::cout, std::cerr

/*
================================
Video size.

TODO: Carry data of allowed resolutions here.
================================
*/
static const int SCREEN_WIDTH = 1200;
static const int SCREEN_HEIGHT = 800;
static const int SCREEN_BPP = 32;

/*
================================
Engine::initVideo
================================
*/
bool Engine::initVideo()
{
	if ( !setVideoMode( SCREEN_WIDTH, SCREEN_HEIGHT, false ) ) {
		return false;
	}

	return true;
}

/*
================================
Engine::setVideoMode

TODO: OpenGL context destroyed; reload all textures (Windows only)
================================
*/
bool Engine::setVideoMode( const int wx, const int wy, const bool fullscreen )
{
	// Caption before video mode
	SDL_WM_SetCaption( "Loading...", NULL );

	// Specify OpenGL double buffer before SDL_SetVideoMode
	if ( SDL_GL_SetAttribute( SDL_GL_DOUBLEBUFFER, 1 ) == -1 ) {
		std::cerr << "Video initialization failed: "
			<< SDL_GetError() << std::endl;
		return false;
	}

	// OpenGL context
	Uint32 flags = SDL_OPENGL;
	if ( fullscreen ) flags |= SDL_FULLSCREEN;
	screen = SDL_SetVideoMode( wx, wy, SCREEN_BPP, flags );
	if ( screen == NULL ) {
		std::cerr << "Video initialization failed: "
			<< SDL_GetError() << std::endl;
		return false;
	}

	// Set viewport according to actual screen size
	glViewport( 0, 0, screen->w, screen->h );

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

	// OpenGL report
	std::cout << "Engine::setVideoMode:" << std::endl;
	std::cout << "Resolution: "
		<< screen->w << "x" << screen->h << " "
		<< ((screen->flags & SDL_FULLSCREEN) ? "fullscreen" : "windowed")
		<< std::endl;
	std::cout << "OpenGL: "
		<< glGetString( GL_VENDOR ) << ", "
		<< glGetString( GL_RENDERER ) << ", "
		<< glGetString( GL_VERSION ) << std::endl;

	return true;
}

/*
================================
Engine::getAspectRatio
================================
*/
double Engine::getAspectRatio() const
{
	return (double) screen->w / screen->h;
}
