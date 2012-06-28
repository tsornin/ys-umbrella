#include "SDL.h"

int main( int argc, char** argv )
{
	SDL_Init( SDL_INIT_EVERYTHING );
	SDL_WM_SetCaption( "Caption", NULL );
	SDL_Surface* screen = SDL_SetVideoMode( 640, 480, 32, SDL_SWSURFACE );
	SDL_Flip( screen );
	SDL_Delay( 1000 );
	SDL_Quit();
	return 0;
}

