#ifndef FRICTION_TEST_STATE_H
#define FRICTION_TEST_STATE_H

#include "entity/EntityState.h" // superclass EntityState

/**
 * Test state.
 */
class FrictionTestState : public EntityState
{
public: // GameState implementation
	virtual ~FrictionTestState() {}

	virtual void init( Engine* game );

public: // Singleton pattern
	static FrictionTestState* Instance();
protected:
	FrictionTestState() {};

public: // Public functions

private: // Private functions

private: // Members
};

#endif
