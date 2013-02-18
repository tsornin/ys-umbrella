#ifndef GAME_SUBSYSTEM
#define GAME_SUBSYSTEM

class Engine;

class Subsystem
{
public:
	Subsystem( Engine& e );

	virtual bool init() = 0;
	virtual void cleanup() = 0;

	Engine& engine;
};

#endif
