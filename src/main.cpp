#include "game/Engine.h"

int main( int argc, char** argv )
{
	Engine game;

	if ( !game.init() ) return 1;

	game.run();

	game.cleanup();

	return 0;
}
