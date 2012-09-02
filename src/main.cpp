#include "game/Engine.h"
#include "game/BlankState.h"
#include "physics/PhysicsState.h"
#include "entity/EntityState.h"
#include "state/States.h"

int main( int argc, char** argv )
{
	Engine game;

	if ( !game.init() ) return 1;

	// game.push( BlankState::Instance() );
	// game.push( PhysicsState::Instance() );
	// game.push( EntityState::Instance() );

	game.push( BarTestState::Instance() );
	game.push( ClothTestState::Instance() );
	game.push( StringTestState::Instance() );

	game.push( PyramidTestState::Instance() );
	game.push( ColumnTestState::Instance() );

	game.run();

	game.cleanup();

	return 0;
}
