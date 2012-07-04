#ifndef GAME_INPUT_SET_H
#define GAME_INPUT_SET_H

#include <vector>

enum InputKey {
	IK_START,
	IK_U, IK_D, IK_L, IK_R,
	IK_A, IK_B, IK_X, IK_Y,
	IK_SL, IK_SR,
	IK_TOTAL
};

/*
================================
Abstracts input into an NES-like controller.

Key events only appear on key-down and key-up.
To detect if a key is currently pressed, we need additional state.
To store key-down and key-up events, we also track the previous state.
================================
*/
class InputSet
{
public:
	InputSet();
	void clock();

	void set( const InputKey a, const bool b );
	void put( const InputKey a, const bool b );

	bool get( const InputKey ) const;
	bool operator [] ( const InputKey ) const;

	bool rising( const InputKey ) const;
	bool falling( const InputKey ) const;

private:
	std::vector < bool > current;
	std::vector < bool > previous;
};

#endif
