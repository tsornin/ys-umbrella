#ifndef GAME_ENGINE_H
#define GAME_ENGINE_H

#include <vector> // for std::vector < State* >
#include "SDL.h" // for SDL_Surface

class State;

/*
================================
Game engine with a stack of game states:
http://gamedevgeek.com/tutorials/managing-game-states-in-c/

TODO: Arrange groups of functions into subsystem classes?
Subsystems should carry some data:
	Input: key bindings, default key bindings
	Video: Allowed video modes
	Audio: ???

TODO: Move subsystem data into a configuration subsystem.
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

	// Timer
	void run();
		void input();
		void update();
		void draw();
	void setCaption();

	// Input
	void pollInput();

	// Video
	bool initVideo();
	double getAspectRatio() const;
	// bool setVideoMode( const int wx, const int wy, const bool fullscreen )

	// Audio
	bool initAudio();

private: // Members
	// Stack
	std::vector < State* > states;

	// Timer
	int frames_elapsed;
	bool quit;
	bool pause;

	// Input
	// InputSet is1, is2;

	// Video
	SDL_Surface* screen;

	// Audio

};

#endif
