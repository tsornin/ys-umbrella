#ifndef ENTITY_STATE_H
#define ENTITY_STATE_H

#include <list>
#include "physics/PhysicsState.h"

class Entity;

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

	virtual void setCaption( std::ostringstream& buffer );

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

	int mx, my;
	bool ml, mr;
	Vec2 cursor, cursor_prev;
	Rigid* crg;
	Verlet* mvl;
	Rigid* mrg;
	MouseConstraint* mct;
	Friction* mftx;
	Friction* mfty;
};

#endif
