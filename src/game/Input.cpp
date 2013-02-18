#include "Input.h"
#include <iostream> // for std::cerr
#include "Engine.h"
#include "State.h"

/*
================================
Input::init

TODO: Even without any input-configuration functions,
we should read these settings from a configuration file.
================================
*/
bool Input::init()
{
    keymap1 = {
        { SDLK_SPACE,   IK_START },
        { SDLK_UP,      IK_U },
        { SDLK_DOWN,    IK_D },
        { SDLK_LEFT,    IK_L },
        { SDLK_RIGHT,   IK_R },
        { SDLK_z,       IK_A },
        { SDLK_x,       IK_B },
        { SDLK_c,       IK_X },
        { SDLK_v,       IK_Y },
        { SDLK_LSHIFT,  IK_SL },
        { SDLK_RSHIFT,  IK_SR }
    };

    keymap2 = {
       { SDLK_RETURN,   IK_START },
       { SDLK_w,        IK_U },
       { SDLK_s,        IK_D },
       { SDLK_a,        IK_L },
       { SDLK_d,        IK_R },
       { SDLK_g,        IK_A },
       { SDLK_h,        IK_B },
       { SDLK_j,        IK_X },
       { SDLK_k,        IK_Y },
       { SDLK_q,        IK_SL },
       { SDLK_e,        IK_SR },
    };

	return true;
}

/*
================================
Input::cleanup
================================
*/
void Input::cleanup()
{
	keymap1.clear();
	keymap2.clear();
}

/*
================================
Input::specialKeyDown

Processes special non-InputSet keyboard commands.
================================
*/
void Input::specialKeyDown( SDLKey sym )
{
	// State stack control (TODO: for testing only)
	if ( sym == SDLK_HOME ) engine.reset();
	if ( sym == SDLK_END ) engine.pop();
	if ( sym == SDLK_ESCAPE ) engine.pop();

	// Single-frame stepping (TODO: for testing only)
	if ( sym == SDLK_F1 ) engine.pause = !engine.pause;
	if ( engine.pause && sym == SDLK_PAGEDOWN ) {
		engine.tick();
	}

	// Fullscreen toggle.
	// TODO: X11 supports fullscreen toggling using SDL_WM_ToggleFullScreen:
	// http://www.libsdl.org/docs/html/sdlwmtogglefullscreen.html
	if ( sym == SDLK_F2 ) {
		engine.video_sys->toggle_fullscreen();
	}
}

/*
================================
Input::poll

Polls for all queued SDL input events
and writes keyboard input to the input sets.

This function sits outside the unfocus/pause guard in Engine::run,
since we always need to check for SDL_Event SDL_QUIT.

NOTE: Since all inputs are clocked even when the game is paused,
all "held" inputs will be broken by unfocus pausing.
================================
*/
void Input::poll()
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
				is1.set( kmi->second, false );
			}
			if ( (kmi = keymap2.find(sym)) != keymap2.end() ) {
				is2.set( kmi->second, false );
			}
		}
		break;

		case SDL_MOUSEMOTION: {
			if ( engine.states.empty() ) break;
			engine.states.back()->mouseMoved( event.motion );
			if ( event.motion.state & SDL_PRESSED ) {
				engine.states.back()->mouseDragged( event.motion );
			}
		}
		break;

		case SDL_MOUSEBUTTONUP: {
			if ( engine.states.empty() ) break;
			engine.states.back()->mouseUp( event.button );
		}
		break;

		case SDL_MOUSEBUTTONDOWN: {
			if ( engine.states.empty() ) break;
			engine.states.back()->mouseDown( event.button );
		}
		break;

		case SDL_QUIT: {
			engine.quit = true;
		}
		break;
	}
	}
}
