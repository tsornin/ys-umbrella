#ifndef GAME_BLANK_STATE_H
#define GAME_BLANK_STATE_H

#include "State.h" // superclass GameState

/*
================================
A dummy GameState that does nothing except count frames.
This class stubs out all abstract functions of State
(so subclasses don't have to).

This file also serves as a find-and-replace template to write new states:
	1. Replace "Blank" with the state name.
	2. Don't forget to change the header guard.
================================
*/
class BlankState : public State
{
public: // State implementation
	virtual ~BlankState() {}

	virtual void init( Engine* game );
	virtual void cleanup();

	virtual void input( Engine* game );
	virtual void update( Engine* game );
	virtual void draw( Engine* game );

	virtual void setCaption( std::ostringstream& buffer );

	virtual void mouseMoved( const SDL_MouseMotionEvent& e );
	virtual void mouseDragged( const SDL_MouseMotionEvent& e );
	virtual void mouseUp( const SDL_MouseButtonEvent& e );
	virtual void mouseDown( const SDL_MouseButtonEvent& e );

public: // Singleton pattern
	static BlankState* Instance();
protected:
	BlankState();

public: // Public functions

private: // Private functions

protected: // Members
	int frames_elapsed;
};

#endif
