#include "Engine.h"
#include "State.h"

/*
================================
Engine::pop

Cleans up and pops the current state.
================================
*/
void Engine::pop()
{
	if ( !states.empty() ) {
		states.back()->cleanup();
		states.pop_back();
	}
}

/*
================================
Engine::push

Initializes and pushes the specified state.
================================
*/
void Engine::push( State* state )
{
	states.push_back( state );
	states.back()->init( this );

	// Ad-hoc memory leak detection
	// TODO: use valgrind
	// for ( int i = 0; i < 100; i++ ) {
		// states.back()->cleanup();
		// states.back()->init( this );
	// }
}

/*
================================
Engine::change

Pops the current state and pushes the specified state.

A running State can effect a reset by passing itself to this function:
game->change( this );
================================
*/
void Engine::change( State* state )
{
	// cleanup the current state
	pop();
	// store and init the new state
	push( state );
}
