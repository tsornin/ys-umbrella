#include "Engine.h"
#include <iostream> // for std::cout, std::cerr
#include "SDL_mixer.h"

/*
================================
Engine::initAudio
================================
*/
bool Engine::initAudio()
{
	if ( Mix_Init( MIX_INIT_MP3 ) == 0 ) {
		return false;
	}

	return true;
}

void Engine::cleanupAudio()
{
	return;
}
