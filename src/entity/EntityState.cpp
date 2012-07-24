#include "EntityState.h"
#include "Entity.h"

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

	// In the header: Camera* cam
	// add( cam = new Camera( *this ) );
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
	// cam->input( game-is2 );
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
}

/*
================================
EntityState::draw
================================
*/
void EntityState::draw( Engine* game )
{
	// for ( Entity* en : entities ) en->draw( game->rd );
	PhysicsState::draw( game );
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
