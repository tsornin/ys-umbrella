#ifndef ENTITY_STATE_H
#define ENTITY_STATE_H

#include <list>
#include "physics/PhysicsState.h"

class Entity;

// TODO: Since EntityState forward-declares Entity,
// all top-level entities must also be forward-declared here.
// Doesn't this seem kind of backwards?
class Camera;
class Fires;
class Flames;

/*
================================
Entity Manager.

Also provides Camera functionality.
================================
*/
class EntityState : public PhysicsState
{
public: // State implementation
	virtual ~EntityState() {}

	virtual void init( Engine* game );
	virtual void cleanup();

	virtual void input( Engine* game );
	virtual void update( Engine* game );
	virtual void draw( Engine* game );

	virtual void frame_caption( std::ostringstream& buffer );
	virtual void frame_printout( std::ostringstream& buffer );

	virtual void mouseMoved( const SDL_MouseMotionEvent& e );
	virtual void mouseDragged( const SDL_MouseMotionEvent& e );
	virtual void mouseUp( const SDL_MouseButtonEvent& e );
	virtual void mouseDown( const SDL_MouseButtonEvent& e );

public: // Singleton pattern
	static EntityState* Instance();
protected:
	EntityState() {}

public: // Public functions
	void add( Entity* en );

private: // Private functions
	int nextEID();

private: // Members
	// (next) Entity ID
	int next_eid;

	// All top-level entities will be in this list
	std::list < Entity* > entities;

	// All top-level entities are also available here
	Camera* cam;
	Fires* fires;
	Flames* flames; friend class Fire;

	int mx, my;
	bool ml, mr;
	Vec2 cursor, cursor_prev;
	Rigid* crg;
	Verlet* mvl;
	Rigid* mrg;
	Friction *mftx, *mfty;
};

#endif
