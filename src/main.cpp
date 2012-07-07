#include "game/Engine.h"
#include "game/BlankState.h"
#include "physics/PhysicsState.h"

int main( int argc, char** argv )
{
	Engine game;

	if ( !game.init() ) return 1;

	game.push( BlankState::Instance() );
	game.push( PhysicsState::Instance() );

	game.run();

	game.cleanup();

	return 0;
}
