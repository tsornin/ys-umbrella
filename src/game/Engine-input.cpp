#include "Engine.h"
#include "State.h"

/*
================================
Engine::initInput

TODO: Even without any input-configuration functions,
we should read these settings from a configuration file.
================================
*/
bool Engine::initInput()
{
	// Controller 1
	keymap1[ SDLK_SPACE ]	= IK_START;
	keymap1[ SDLK_UP ]		= IK_U;
	keymap1[ SDLK_DOWN ]	= IK_D;
	keymap1[ SDLK_LEFT ]	= IK_L;
	keymap1[ SDLK_RIGHT ]	= IK_R;
	keymap1[ SDLK_z ]		= IK_A;
	keymap1[ SDLK_x ]		= IK_B;
	keymap1[ SDLK_c ]		= IK_X;
	keymap1[ SDLK_v ]		= IK_Y;
	keymap1[ SDLK_LSHIFT ]	= IK_SL;
	keymap1[ SDLK_RSHIFT ]	= IK_SR;

	// Controller 2
	keymap2[ SDLK_RETURN ]	= IK_START;
	keymap2[ SDLK_w ]		= IK_U;
	keymap2[ SDLK_s ]		= IK_D;
	keymap2[ SDLK_a ]		= IK_L;
	keymap2[ SDLK_d ]		= IK_R;
	keymap2[ SDLK_g ]		= IK_A;
	keymap2[ SDLK_h ]		= IK_B;
	keymap2[ SDLK_j ]		= IK_X;
	keymap2[ SDLK_k ]		= IK_Y;
	keymap2[ SDLK_q ]		= IK_SL;
	keymap2[ SDLK_e ]		= IK_SR;

	return true;
}

/*
================================
Engine::specialKeyDown

Processes special non-InputSet keyboard commands.
================================
*/
void Engine::specialKeyDown( SDLKey sym )
{
	// State stack control (TODO: for testing only)
	if ( sym == SDLK_HOME ) reset();
	if ( sym == SDLK_END ) pop();
	if ( sym == SDLK_ESCAPE ) pop();

	// Single-frame stepping (TODO: for testing only)
	if ( sym == SDLK_F1 ) pause = !pause;
	if ( pause && sym == SDLK_PAGEDOWN ) {
		input();
		update();
		draw();
		setCaption();
	}

	// Fullscreen toggle.
	// TODO: X11 supports fullscreen toggling using SDL_WM_ToggleFullScreen:
	// http://www.libsdl.org/docs/html/sdlwmtogglefullscreen.html
	if ( sym == SDLK_F2 ) {
		bool fullscreen = screen->flags & SDL_FULLSCREEN;
		setVideoMode( screen->w, screen->h, !fullscreen );
	}
}

/*
================================
Engine::pollInput

Polls for all queued SDL input events
and writes keyboard input to the input sets.

This function sits outside the unfocus/pause guard in Engine::run,
since we always need to check for SDL_Event SDL_QUIT.

TODO: Since all inputs are clocked even when the game is paused,
all "held" inputs will be broken by pausing.
================================
*/
void Engine::pollInput()
{
	is1.clock();
	is2.clock();

	SDL_Event event;
	while ( SDL_PollEvent( &event ) ) {
	switch ( event.type )
	{
		case SDL_KEYDOWN: {
			SDLKey sym = event.key.keysym.sym;
			std::map < SDLKey, InputKey >::iterator kmi;
			if ( (kmi = keymap1.find(sym)) != keymap1.end() ) {
				is1.set( kmi->second, true );
			}
			if ( (kmi = keymap2.find(sym)) != keymap2.end() ) {
				is2.set( kmi->second, true );
			}

			specialKeyDown( sym );
		}
		break;

		case SDL_KEYUP: {
			SDLKey sym = event.key.keysym.sym;
			std::map < SDLKey, InputKey >::iterator kmi;
			if ( (kmi = keymap1.find(sym)) != keymap1.end() ) {
				is1.set( kmi->second, true );
			}
			if ( (kmi = keymap2.find(sym)) != keymap2.end() ) {
				is2.set( kmi->second, true );
			}
		}
		break;

		case SDL_MOUSEMOTION: {
			if ( states.empty() ) break;
			states.back()->mouseMoved( event.motion );
			if ( event.motion.state & SDL_PRESSED ) {
				states.back()->mouseDragged( event.motion );
			}
		}
		break;

		case SDL_MOUSEBUTTONUP: {
			if ( states.empty() ) break;
			states.back()->mouseUp( event.button );
		}
		break;

		case SDL_MOUSEBUTTONDOWN: {
			if ( states.empty() ) break;
			states.back()->mouseDown( event.button );
		}
		break;

		case SDL_QUIT: {
			quit = true;
		}
		break;
	}
	}
}
