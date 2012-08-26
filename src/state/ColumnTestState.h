#ifndef COLUMN_TEST_STATE_H
#define COLUMN_TEST_STATE_H

#include "entity/EntityState.h" // superclass EntityState

/**
 * Test state.
 */
class ColumnTestState : public EntityState
{
public: // GameState implementation
	virtual ~ColumnTestState() {}

	virtual void init( Engine* game );

public: // Singleton pattern
	static ColumnTestState* Instance();
protected:
	ColumnTestState() {};

public: // Public functions

private: // Private functions

private: // Members
};

#endif
