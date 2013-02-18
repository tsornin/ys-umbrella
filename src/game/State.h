#ifndef GAME_STATE_H
#define GAME_STATE_H

#include "Engine.h"
#include <sstream> // for frame_caption, frame_printout

/*
================================
Game state of a game engine.
http://gamedevgeek.com/tutorials/managing-game-states-in-c/
================================
*/
class State
{
public:
	virtual ~State() {}

	virtual void init( Engine* game ) = 0;
	virtual void cleanup() = 0;

	virtual void input( Engine* game ) = 0;
	virtual void update( Engine* game ) = 0;
	virtual void draw( Engine* game ) = 0;

	virtual void frame_caption( std::ostringstream& buffer ) = 0;
	virtual void frame_printout( std::ostringstream& buffer ) = 0;

	virtual void mouseMoved( const SDL_MouseMotionEvent& e ) = 0;
	virtual void mouseDragged( const SDL_MouseMotionEvent& e ) = 0;
	virtual void mouseUp( const SDL_MouseButtonEvent& e ) = 0;
	virtual void mouseDown( const SDL_MouseButtonEvent& e ) = 0;

protected:
	State() {}
};

#endif
