#ifndef POINT_DATA_BRUTE_FORCE_H
#define POINT_DATA_BRUTE_FORCE_H

#include <vector>
#include "AABB.h" // for query

/*
================================
PD_BruteForce

Brute force implementation of PointData.
================================
*/
template < typename T >
class PD_BruteForce
{
public:
	~PD_BruteForce() {}

	void insert( Vec2, T );
	std::vector < T > query( const AABB& ) const;

private: // Members
	typedef std::pair < Vec2, T > Entry;
	std::vector < Entry > entries;
};

/*
================================
PD_BruteForce::insert
================================
*/
template < typename T >
void PD_BruteForce < T >::insert( Vec2 v, T t )
{
	entries.push_back( Entry( v, t ) );
}

/*
================================
PD_BruteForce::query
================================
*/
template < typename T >
std::vector < T > PD_BruteForce < T >::query( const AABB& box ) const
{
	std::vector < T > ts;

	for ( const Entry& e : entries ) {
		if ( box.contains( e.first ) ) {
			ts.push_back( e.second );
		}
	}

	return ts;
}

#endif
