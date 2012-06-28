#include "Engine.h"
#include "State.h"

/*
================================
Engine::pollInput

Polls for all queued SDL input events
TODO: and writes keyboard input to the input sets.

This function sits outside the unfocus/pause guard in Engine::run,
since we always need to check for SDL_Event SDL_QUIT.

TODO: Since all inputs are clocked even when the game is paused,
all "held" inputs will be broken by pausing.
================================
*/
void Engine::pollInput()
{
	// is1.clock();
	// is2.clock();

	SDL_Event event;
	while ( SDL_PollEvent( &event ) ) {
	switch ( event.type )
	{
		case SDL_KEYDOWN: {
			SDLKey sym = event.key.keysym.sym;

			// Fullscreen toggle button.
			// TODO: X11 supports fullscreen toggling using SDL_WM_ToggleFullScreen.
			// http://www.libsdl.org/docs/html/sdlwmtogglefullscreen.html
			// if ( sym == SDLK_F11 ) {
			// 	bool fullscreen = screen->flags & SDL_FULLSCREEN;
			// 	setVideoMode( screen->w, screen->h, !fullscreen );
			// }

			// State stack control (TODO: for testing only)
			if ( sym == SDLK_HOME ) change( states.back() );
			if ( sym == SDLK_END ) pop();
			if ( sym == SDLK_ESCAPE ) pop();

			// Single-frame stepping (TODO: for testing only)
			if ( sym == SDLK_F10 ) pause = !pause;
			if ( pause && sym == SDLK_PAGEDOWN ) {
				input();
				update();
				draw();

				setCaption();
			}

			break;
		}

		case SDL_KEYUP: {
			SDLKey sym = event.key.keysym.sym;

			break;
		}

		case SDL_MOUSEMOTION: {
			if ( states.empty() ) break;

			SDL_MouseMotionEvent& motion = event.motion;
			states.back()->mouseMoved( motion );

			if ( motion.state & SDL_PRESSED ) {
				states.back()->mouseDragged( motion );
			}

			break;
		}

		case SDL_MOUSEBUTTONUP: {
			if ( states.empty() ) break;

			SDL_MouseButtonEvent& button = event.button;
			states.back()->mouseUp( button );
			break;
		}

		case SDL_MOUSEBUTTONDOWN: {
			if ( states.empty() ) break;

			SDL_MouseButtonEvent& button = event.button;
			states.back()->mouseDown( button );
			break;
		}

		case SDL_QUIT: {
			quit = true;
			break;
		}
	}
	}
}
