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
	if ( SDL_InitSubSystem( SDL_INIT_AUDIO ) == -1 ) {
		std::cerr << "SDL audio subsystem initialization failed: "
			<< SDL_GetError() << std::endl;
		return false;
	}

	if ( Mix_Init( MIX_INIT_MP3 ) == 0 ) {
		return false;
	}

	return true;
}

void Engine::cleanupAudio()
{
	return;
}
