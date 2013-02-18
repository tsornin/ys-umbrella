#include "Video.h"
#include <iostream> // for std::cout, std::cerr
#include "Engine.h"

/*
================================
Video size.

TODO: Carry data of allowed resolutions here.
TODO: Consider mobile devices as well as PC resolutions.
================================
*/
static const int SCREEN_WIDTH = 1200;
static const int SCREEN_HEIGHT = 800;
static const int SCREEN_BPP = 32;

/*
================================
Video::init
================================
*/
bool Video::init()
{
	if ( SDL_InitSubSystem( SDL_INIT_VIDEO ) == -1 ) {
		std::cerr << "SDL video subsystem initialization failed: "
			<< SDL_GetError() << std::endl;
		return false;
	}

	if ( !setVideoMode( SCREEN_WIDTH, SCREEN_HEIGHT, false ) ) {
		return false;
	}

	return true;
}

/*
================================
Video::cleanup
================================
*/
void Video::cleanup()
{
	engine.rd.cleanup();
}

/*
================================
Video::setVideoMode

TODO: OpenGL context destroyed; reload all textures (Windows only)
================================
*/
bool Video::setVideoMode( const int wx, const int wy, const bool fullscreen )
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

	// OpenGL report
	std::cout << "Video::setVideoMode:" << std::endl;
	std::cout << "Resolution: "
		<< screen->w << "x" << screen->h << " "
		<< ((screen->flags & SDL_FULLSCREEN) ? "fullscreen" : "windowed")
		<< std::endl;
	std::cout << "OpenGL: "
		<< glGetString( GL_VENDOR ) << ", "
		<< glGetString( GL_RENDERER ) << ", "
		<< glGetString( GL_VERSION ) << std::endl;

	engine.rd.init();

	return true;
}

/*
================================
Video::getAspectRatio
================================
*/
double Video::getAspectRatio() const
{
	return (double) screen->w / screen->h;
}

/*
================================
Video::toggle_fullscreen
================================
*/
void Video::toggle_fullscreen()
{
	bool fullscreen = screen->flags & SDL_FULLSCREEN;
	setVideoMode( screen->w, screen->h, !fullscreen );
}
