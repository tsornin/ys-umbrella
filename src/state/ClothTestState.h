#ifndef CLOTH_TEST_STATE_H
#define CLOTH_TEST_STATE_H

#include "entity/EntityState.h" // superclass EntityState

/**
 * Test state.
 */
class ClothTestState : public EntityState
{
public: // GameState implementation
	virtual ~ClothTestState() {}

	virtual void init( Engine* game );

public: // Singleton pattern
	static ClothTestState* Instance();
protected:
	ClothTestState() {};

public: // Public functions

private: // Private functions

private: // Members
};

#endif
