#include "game/Engine.h"
#include "game/BlankState.h"
#include "physics/PhysicsState.h"
#include "entity/EntityState.h"
#include "state/StringTestState.h"
#include "state/ClothTestState.h"
#include "state/BarTestState.h"

#include <iostream>

int main( int argc, char** argv )
{
	Engine game;

	if ( !game.init() ) return 1;

	game.push( BlankState::Instance() );
	game.push( PhysicsState::Instance() );
	game.push( EntityState::Instance() );

	game.push( BarTestState::Instance() );
	game.push( ClothTestState::Instance() );
	game.push( StringTestState::Instance() );

	game.run();

	game.cleanup();

	return 0;
}
