#ifndef GAME_INPUT
#define GAME_INPUT

#include "Subsystem.h"
#include "SDL.h"
#include <map>
#include "InputSet.h"

class Input : public Subsystem
{
public:
	Input( Engine& e ) : Subsystem( e ) {}

	virtual bool init();
	virtual void cleanup();
	void poll();
	void specialKeyDown( SDLKey sym );

	std::map < SDLKey, InputKey > keymap1, keymap2;
	InputSet is1, is2;
};

#endif
