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

	if ( SDL_Init( 0 ) == -1 ) {
		std::cerr << "SDL initialization failed: "
			<< SDL_GetError() << std::endl;
		return false;
	}

	input_sys = new Input( *this );
	video_sys = new Video( *this );
	audio_sys = new Audio( *this );
	for ( Subsystem* sub : all_subsystems() ) {
		if ( !sub->init() ) {
			return false;
		}
	}

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

	auto subs = all_subsystems();
	std::reverse( subs.begin(), subs.end() );
	for ( Subsystem *sub : subs ) {
		sub->cleanup();
	}

	SDL_Quit();
}

/*
================================
Engine::all_subsystems
================================
*/
std::vector < Subsystem* > Engine::all_subsystems()
{
	return std::vector < Subsystem* >{ input_sys, video_sys, audio_sys };
}
