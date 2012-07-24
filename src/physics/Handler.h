#ifndef PHYSICS_HANDLER_H
#define PHYSICS_HANDLER_H

/*
================================
Collision handling interface.
================================
*/
class Handler
{
	virtual void handle() = 0;

protected:
	~Handler() {}
};

#endif
