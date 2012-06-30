#include "InputSet.h"

/*
================================
InputSet::InputSet
================================
*/
InputSet::InputSet() :
	current( IK_TOTAL ),
	previous( IK_TOTAL ) {}

/*
================================
InputSet::clock
================================
*/
void InputSet::clock()
{
	previous = current;
}

/*
================================
InputSet::set
================================
*/
void InputSet::set( const InputKey i, const bool b )
{
	current[i] = b;
}

/*
================================
InputSet::put
================================
*/
void InputSet::put( const InputKey i, const bool b )
{
	current[i] = b;
	previous[i] = b;
}

/*
================================
InputSet::get
================================
*/
bool InputSet::get( const InputKey i ) const
{
	return current[i];
}

/*
================================
InputSet::operator []
================================
*/
bool InputSet::operator [] ( const InputKey i ) const
{
	return current[i];
}

/*
================================
InputSet::rising
================================
*/
bool InputSet::rising( const InputKey i ) const
{
	return current[i] && !previous[i];
}

/*
================================
InputSet::falling
================================
*/
bool InputSet::falling( const InputKey i ) const
{
	return !current[i] && previous[i];
}
