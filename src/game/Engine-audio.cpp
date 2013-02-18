#include "Engine.h"
#include <iostream> // for std::cout, std::cerr
#include "SDL_mixer.h"

/*
================================
Audio settings.

TODO: Do we need anything more involved?
Consider mobile devices as well as PC settings.
================================
*/
	// static const int AUDIO_FREQUENCY = 22050;
	static const int AUDIO_FREQUENCY = 44100;
static const Uint16 AUDIO_FORMAT = MIX_DEFAULT_FORMAT;
	static const int AUDIO_MONO = 1;
	static const int AUDIO_STEREO = 2;
static const int AUDIO_CHANNELS = AUDIO_STEREO;
static const int AUDIO_CHUNK_SIZE = 1024;

/*
================================
Engine::initAudio
================================
*/
bool Engine::initAudio()
{
	if ( SDL_InitSubSystem( SDL_INIT_AUDIO ) == -1 ) {
		std::cerr << "SDL audio subsystem initialization failed: "
			<< SDL_GetError() << std::endl;
		return false;
	}

	if ( Mix_Init( MIX_INIT_OGG ) == -1 ) {
		std::cerr << "Mix_Init failed: "
			<< SDL_GetError() << std::endl;
		return false;
	}

	if ( Mix_OpenAudio( AUDIO_FREQUENCY, AUDIO_FORMAT, AUDIO_CHANNELS, AUDIO_CHUNK_SIZE ) == -1 ) {
		std::cerr << "Mix_OpenAudio failed: "
			<< SDL_GetError() << std::endl;
		return false;
	}

	int frequency; Uint16 format; int channels;
	Mix_QuerySpec( &frequency, &format, &channels );
	if ( frequency != AUDIO_FREQUENCY || channels != AUDIO_CHANNELS ) {
		std::cerr << "Engine::initAudio:" << std::endl;
		std::cerr << "Warning: Actual audio format does not match specified audio format." << std::endl;
	}

	return true;
}

/*
================================
Engine::cleanupAudio
================================
*/
void Engine::cleanupAudio()
{
	Mix_CloseAudio();
	Mix_Quit();
}
