#ifndef PYRAMID_TEST_STATE_H
#define PYRAMID_TEST_STATE_H

#include "entity/EntityState.h" // superclass EntityState

/**
 * Test state.
 */
class PyramidTestState : public EntityState
{
public: // GameState implementation
	virtual ~PyramidTestState() {}

	virtual void init( Engine* game );

public: // Singleton pattern
	static PyramidTestState* Instance();
protected:
	PyramidTestState() {};

public: // Public functions

private: // Private functions

private: // Members
};

#endif
