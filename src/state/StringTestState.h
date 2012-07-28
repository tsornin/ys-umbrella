#ifndef STRING_TEST_STATE_H
#define STRING_TEST_STATE_H

#include "entity/EntityState.h" // superclass EntityState

/**
 * Test state.
 */
class StringTestState : public EntityState
{
public: // GameState implementation
	virtual ~StringTestState() {}

	virtual void init( Engine* game );

public: // Singleton pattern
	static StringTestState* Instance();
protected:
	StringTestState() {};

public: // Public functions

private: // Private functions

private: // Members
};

#endif
