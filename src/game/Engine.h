#ifndef GAME_ENGINE_H
#define GAME_ENGINE_H

#include <vector> // for std::vector < State* >
#include <map> // for std::map < SDLKey, InputKey >
#include "SDL.h" // for SDL_Surface
#include "InputSet.h"
#include "graphics/Renderer.h"

class State;

/*
================================
Game engine with a stack of game states:
http://gamedevgeek.com/tutorials/managing-game-states-in-c/

TODO: This class definition conflates several subsystems.
Should we arrange the groups of functions into subsystem classes?
Subsystems should carry some data:
	Input: key bindings, default key bindings
	Video: allowed video modes
	Audio: ???

TODO: Move subsystem data into a configuration subsystem.

TODO: Think about the resource split for subsystems and wrappers?
video - graphics init
image - image file format loading -> opengl
audio - audio init
sound - sound file format loading -> ???
================================
*/
class Engine
{
public:
	// Engine
	bool init();
	void cleanup();

	// Stack
	void pop();
	void push( State* state );
	void change( State* state );
	void reset();

	// Timer
	void run();
	void input();
	void update();
	void draw();
	void setCaption();

	// Input
	bool initInput();
	void pollInput();
	void specialKeyDown( SDLKey sym );

	// Video
	bool initVideo();
	void cleanupAudio();
	bool setVideoMode( const int wx, const int wy, const bool fullscreen );
	double getAspectRatio() const;

	// Audio
	bool initAudio();

public: // Members
	// Stack
	std::vector < State* > states;

	// Timer
	int frames_elapsed;
	bool quit;
	bool pause;

	// Input
	std::map < SDLKey, InputKey > keymap1, keymap2;
	InputSet is1, is2;

	// Video
	SDL_Surface* screen;

	// Audio

public: // More members
	Renderer rd;
};

#endif
