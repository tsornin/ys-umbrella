#include "Engine.h"
#include "State.h"
#include <iostream> // for std::ostringstream

/*
================================
Framerate constants.
================================
*/
static const int TICK_INTERVAL = 17; // milliseconds
static const int FPS = 60;

/*
================================
time_left

Support function for GameEngine::run:
http://www.libsdl.org/intro.en/usingtimers.html

Returns the time in milliseconds until the next frame.
Always returns a non-negative number.
================================
*/
static Uint32 next_time;
static Uint32 time_left()
{
	Uint32 now = SDL_GetTicks();
	if ( next_time <= now ) {
		// If we're going to return 0, we should also reset next_time.
		// This fixes the problem of next_time taking several frames to
		// catch up (causing a big speed-up) when the window is dragged
		// (since dragging the window simulates a huge processing lag).
		// NOTE: Windows only.
		next_time = now;
		return 0;
	}
	else {
		return next_time - now;
	}
}

/*
================================
Engine::run

Main loop.
================================
*/
void Engine::run()
{
	next_time = SDL_GetTicks() + TICK_INTERVAL;

	while ( !states.empty() && !quit )
	{
		// Poll for input even when unfocused
		input_sys->poll();

		if ( !pause && (SDL_GetAppState() & SDL_APPINPUTFOCUS ) ) {
			tick();
		}

		SDL_Delay( time_left() );
		// while ( SDL_GetTicks() < next_time ) {
		// 	SDL_Delay(1);
		// }
		next_time += TICK_INTERVAL;
	}
}

/*
================================
Engine::tick
================================
*/
void Engine::tick()
{
	input();
	update();
	draw();

	setCaption();
}

/*
================================
Engine::input

Lets the current state access processed InputSets.
================================
*/
void Engine::input()
{
	if ( !states.empty() ) {
		states.back()->input( this );
	}
}

/*
================================
Engine::update

Lets the current state update the game.
================================
*/
void Engine::update()
{
	if ( !states.empty() ) {
		states.back()->update( this );
	}

	frames_elapsed++;
}

/*
================================
Engine::draw

Lets the current state draw to the screen.
================================
*/
void Engine::draw()
{
	rd.begin();

	if ( !states.empty() ) {
		states.back()->draw( this );
	}

	rd.end();

	SDL_GL_SwapBuffers();
}

/*
================================
Engine::setCaption

Displays a frame count on the title bar,
then lets the current state write some more to the title bar.
================================
*/
void Engine::setCaption()
{
	std::ostringstream caption;
	caption << "Umbrella: ";

	// Display overload
	// (Shows a sad face if we drop below 60 fps)
	static std::string happy = "^-^";
	static std::string sad = ">.<";
	caption << ( time_left() ? happy : sad );

	// Display frame count
	int x = frames_elapsed;
	int f = x % FPS; x /= FPS;
	int s = x % 60; x /= 60;
	int m = x % 60; x /= 60;
	int h = x;
	caption << " || Time: ";
	if ( h ) caption << h << "h ";
	if ( m ) caption << m << "m ";
	if ( s ) caption << s << "s ";
	if ( f < 10 ) caption << "0"; // zero in front of framecount
	caption << f << "f"; // always show framecount

	// OpenGL error
	GLenum err = glGetError();
	if ( err != GL_NO_ERROR ) {
		caption << " || OpenGL Error: " << gluErrorString( err );
	}

	if ( !states.empty() ) {
		states.back()->setCaption( caption );
	}

	// Send caption to window title bar
	SDL_WM_SetCaption( caption.str().c_str(), NULL );
}
