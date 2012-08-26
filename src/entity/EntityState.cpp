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

	// TODO: What is this nonsense? Make some Rigid test cases!

	MeshOBJ o_frame;
	o_frame.load( Path( "level/pong/", "frame.obj" ) );
	o_frame.setScale( 50 );

	Rigid* frame = PhysicsState::createRigid( o_frame, 1 );
	frame->setPosition( Vec2( 0, -1600 ) );
	frame->setLinearEnable( false );
	frame->setAngularEnable( false );

	frame = PhysicsState::createRigid( o_frame, 1 );
	frame->setPosition( Vec2( 0, -600 ) );
	frame->setAngle( 1.57 );
	frame->setLinearEnable( false );
	frame->setAngularEnable( false );

	MeshOBJ o_rg;
	o_rg.load( Path( "level/test/", "4gon.obj" ) );
	o_rg.setScale( 50 );

	// Column
	Scalar s = 50 * 1.5;
	for ( int j = 0; j < 50; ++j ) {
		rg = PhysicsState::createRigid( o_rg, 1 );
		rg->setPosition( Vec2( 0, ((float)j+0.5)*s ) );
		rg->setGravity( Vec2( 0, -0.2 ) );
	}

	// rg->setMass( rg->getMass() * 10 );
	// rg->setMoment( rg->getMoment() * 10 );

	// Pyramid
	// Scalar s = 50 * 1.5;
	// int n = 5;
	// for ( int j = 0; j < n; ++j ) {
	// for ( int i = 0; i < n-j; ++i ) {
	// 	rg = PhysicsState::createRigid( o_rg, 1 );
	// 	rg->setPosition( Vec2( (i - n/2 + (Scalar)j/2)*s, j*s ) );
	// 	rg->setGravity( Vec2( 0, -0.2 ) );
	// }}
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
	rg->input( game->is1 );
	cam->input( game->is2 );

	Entity* t = cam->getTarget();
	if ( t ) t->input( game->is1 );

	if ( ml ) {

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
