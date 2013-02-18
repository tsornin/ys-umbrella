#include "BlankState.h"

/*
================================
BlankState::Instance

Singleton pattern.
================================
*/
BlankState* BlankState::Instance()
{
	static BlankState the_BlankState;
	return &the_BlankState;
}

/*
================================
BlankState::BlankState
================================
*/
BlankState::BlankState()
{
	
}

/*
================================
BlankState::init
================================
*/
void BlankState::init( Engine* game )
{
	frames_elapsed = 0;
}

/*
================================
BlankState::cleanup
================================
*/
void BlankState::cleanup()
{
	
}

/*
================================
BlankState::input
================================
*/
void BlankState::input( Engine* game )
{
	
}

/*
================================
BlankState::update
================================
*/
void BlankState::update( Engine* game )
{
	frames_elapsed++;
}

/*
================================
BlankState::draw
================================
*/
void BlankState::draw( Engine* game )
{
	
}

/*
================================
BlankState::frame_caption
================================
*/
void BlankState::frame_caption( std::ostringstream& buffer )
{
	buffer << " || Blank:";
	buffer << " " << frames_elapsed << " frames elapsed";
}

/*
================================
BlankState::frame_printout
================================
*/
void BlankState::frame_printout( std::ostringstream& buffer )
{
	
}

/*
================================
BlankState::mouseMoved
================================
*/
void BlankState::mouseMoved( const SDL_MouseMotionEvent& e )
{
	
}

/*
================================
BlankState::mouseDragged
================================
*/
void BlankState::mouseDragged( const SDL_MouseMotionEvent& e )
{
	
}

/*
================================
BlankState::mouseUp
================================
*/
void BlankState::mouseUp( const SDL_MouseButtonEvent& e )
{
	
}

/*
================================
BlankState::mouseDown
================================
*/
void BlankState::mouseDown( const SDL_MouseButtonEvent& e )
{
	
}
