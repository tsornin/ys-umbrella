#include "game/Engine.h"
#include "game/BlankState.h"
#include "physics/PhysicsState.h"
#include "entity/EntityState.h"
#include "state/States.h"
#include <iostream>

int main( int argc, char** argv )
{
#ifdef DEBUG
	std::cout << "Debug build!" << std::endl;
#endif

	Engine game;

	if ( !game.init() ) return 1;

	// game.push( BlankState::Instance() );
	// game.push( PhysicsState::Instance() );
	// game.push( EntityState::Instance() );

	game.push( PyramidTestState::Instance() );
	game.push( ColumnTestState::Instance() );
	// game.push( FrictionTestState::Instance() );

	game.push( BarTestState::Instance() );
	game.push( ClothTestState::Instance() );
	game.push( StringTestState::Instance() );

	game.run();

	game.cleanup();

	return 0;
}
