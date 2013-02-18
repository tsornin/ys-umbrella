#ifndef GAME_ENGINE_H
#define GAME_ENGINE_H

#include <vector> // for std::vector < State* >
#include "SDL.h" // TODO: is this necessary?
#include "graphics/Renderer.h" // TODO: what's this doing here?

#include "Input.h"
#include "Video.h"
#include "Audio.h"
// #include "Config.h"

class State;

/*
================================
Game engine with a stack of game states:
http://gamedevgeek.com/tutorials/managing-game-states-in-c/
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
	void tick();
		void input();
		void update();
		void draw();
	void setCaption();

	// Subsystems
	std::vector < Subsystem* > all_subsystems();

public: // Members
	// Stack
	std::vector < State* > states;

	// Timer
	int frames_elapsed;
	bool quit;
	bool pause;

	Input* input_sys;
	Video* video_sys;
	Audio* audio_sys;

	Renderer rd;
};

#endif
