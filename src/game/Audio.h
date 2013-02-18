#ifndef GAME_AUDIO
#define GAME_AUDIO

#include "Subsystem.h"
#include "SDL.h"
#include "SDL_mixer.h"

class Audio : public Subsystem
{
public:
	Audio( Engine& e ) : Subsystem( e ) {}

	virtual bool init();
	virtual void cleanup();
};

#endif
