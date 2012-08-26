#include "EntityState.h"
#include "Entity.h"
#include "Camera.h"

/*
================================
EntityState::Instance

Singleton pattern.
================================
*/
EntityState* EntityState::Instance()
{
	static EntityState the_EntityState;
	return &the_EntityState;
}

/*
================================
EntityState::init
================================
*/
void EntityState::init( Engine* game )
{
	PhysicsState::init( game );

	next_eid = 0;

	add( cam = new Camera( *this ) );

	mx = 0;
	my = 0;
	ml = false;
	mr = false;
	cursor = Vec2( 0 );
	cursor_prev = Vec2( 0 );
}

/*
================================
EntityState::cleanup
================================
*/
void EntityState::cleanup()
{
	for ( Entity* en : entities ) delete en;
	entities.clear();

	PhysicsState::cleanup();
}

/*
================================
EntityState::input
================================
*/
void EntityState::input( Engine* game )
{
	cam->input( game->is2 );

	Entity* t = cam->getTarget();
	if ( t ) t->input( game->is1 );

	if ( ml ) {
		// TODO: cutting Distance constraints
	}
	if ( mr ) {
		if ( mv ) mv->setPosition( cursor );
	}
}

/*
================================
EntityState::update
================================
*/
void EntityState::update( Engine* game )
{
	PhysicsState::update( game );

	for ( Entity* en : entities ) en->update();

	// Compute cursor
	cursor_prev = cursor;
	cursor = cam->world( mx, my );
}

/*
================================
EntityState::draw
================================
*/
void EntityState::draw( Engine* game )
{
	for ( Entity* en : entities ) en->draw( game->rd );

	PhysicsState::draw( game );

	// Display cursor
	Color color = (ml||mr) ? RGBA_RED : RGBA_GREEN;
	glPointSize( 10.0 );
	glBegin( GL_POINTS );
		gl_SetColor( color );
		gl_SetVertex( cursor );
		gl_SetColor( color.alpha( 0.5 ) );
		gl_SetVertex( cursor_prev );
	glEnd();
}

/*
================================
EntityState::setCaption
================================
*/
void EntityState::setCaption( std::ostringstream& buffer )
{
	PhysicsState::setCaption( buffer );

	buffer << " || Entity:";
	buffer << " " << entities.size();
}

/*
================================
EntityState mouse functions
================================
*/

void EntityState::mouseMoved( const SDL_MouseMotionEvent& e )
{
	mx = e.x;
	my = e.y;
}

void EntityState::mouseDragged( const SDL_MouseMotionEvent& e )
{
	
}

void EntityState::mouseUp( const SDL_MouseButtonEvent& e )
{
	if ( e.button == SDL_BUTTON_LEFT ) {
		ml = false;
	}
	if ( e.button == SDL_BUTTON_RIGHT ) {
		mr = false;
		mv = 0;
	}
}

void EntityState::mouseDown( const SDL_MouseButtonEvent& e )
{
	if ( e.button == SDL_BUTTON_LEFT ) {
		ml = true;
	}
	if ( e.button == SDL_BUTTON_RIGHT ) {
		mv = PhysicsState::nearestVerlet( cursor, 50 );
		mr = true;
	}

	if ( e.button == SDL_BUTTON_WHEELUP ) {
		cam->zoomIn();
	}
	if ( e.button == SDL_BUTTON_WHEELDOWN ) {
		cam->zoomOut();
	}
}

/*
================================
EntityState::add
================================
*/
void EntityState::add( Entity* en )
{
	// TODO: we need to assign EID's to non top-level Entities too.
	en->eid = nextEID();
	entities.push_back( en );
}

/*
================================
EntityState::nextEID
================================
*/
int EntityState::nextEID()
{
	return next_eid++;
}
