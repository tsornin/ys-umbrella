#ifndef BAR_TEST_STATE_H
#define BAR_TEST_STATE_H

#include "entity/EntityState.h" // superclass EntityState

/**
 * Test state.
 */
class BarTestState : public EntityState
{
public: // GameState implementation
	virtual ~BarTestState() {}

	virtual void init( Engine* game );

public: // Singleton pattern
	static BarTestState* Instance();
protected:
	BarTestState() {};

public: // Public functions

private: // Private functions

private: // Members
};

#endif
