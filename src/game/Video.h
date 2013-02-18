#ifndef GAME_VIDEO
#define GAME_VIDEO

#include "Subsystem.h"
#include "SDL.h"
#include "SDL_opengl.h"

class Video : public Subsystem
{
public:
	Video( Engine& e ) : Subsystem( e ) {}

	virtual bool init();
	virtual void cleanup();
	bool setVideoMode( const int wx, const int wy, const bool fullscreen );
	double getAspectRatio() const;
	void toggle_fullscreen();

	SDL_Surface* screen;
};

#endif
