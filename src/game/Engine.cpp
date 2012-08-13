#include "Engine.h"
#include "State.h"
#include <iostream> // for std::cerr

/*
================================
Engine::init
================================
*/
bool Engine::init()
{
	frames_elapsed = 0;
	quit = false;
	pause = false;

	screen = 0;

	if ( SDL_Init( 0 ) == -1 ) {
		std::cerr << "SDL initialization failed: "
			<< SDL_GetError() << std::endl;
		return false;
	}

	if ( !initInput() ) {
		return false;
	}

	if ( !initVideo() ) {
		return false;
	}

	// if ( !initAudio() ) {
	// 	return false;
	// }

	return true;
}

/*
================================
Engine::cleanup

Cleans up the state stack and shuts down SDL.
================================
*/
void Engine::cleanup()
{
	// cleanup all the states
	while ( !states.empty() ) {
		states.back()->cleanup();
		states.pop_back();
	}

	// TODO: cleanup for subsystems.
	rd.cleanup();

	cleanupAudio();

	SDL_Quit();
}
