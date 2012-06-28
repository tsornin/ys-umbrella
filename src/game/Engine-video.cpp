#include "Engine.h"
#include <iostream> // for std::cout, std::cerr
/*
================================
Engine::initVideo
================================
*/
bool Engine::initVideo()
{
	static const int SCREEN_BPP = 32;
	screen = SDL_SetVideoMode( 1200, 800, SCREEN_BPP, SDL_SWSURFACE );
	if ( screen == NULL ) {
		std::cerr << "Video initialization failed: "
			<< SDL_GetError() << std::endl;
		return false;
	}

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
