#ifndef REGION_DATA_BRUTE_FORCE_H
#define REGION_DATA_BRUTE_FORCE_H

#include <vector>
#include "AABB.h" // for query

/*
================================
RD_BruteForce

Brute force implementation of RegionData.
================================
*/
template < typename T >
class RD_BruteForce
{
public:
	~RD_BruteForce() {}

	void insert( AABB, T );
	std::vector < T > query( const AABB& ) const;

private: // Members
	typedef std::pair < AABB, T > Entry;
	std::vector < Entry > entries;
};

/*
================================
RD_BruteForce::insert
================================
*/
template < typename T >
void RD_BruteForce < T >::insert( AABB box, T t )
{
	entries.push_back( Entry( box, t ) );
}

/*
================================
RD_BruteForce::query
================================
*/
template < typename T >
std::vector < T > RD_BruteForce < T >::query( const AABB& box ) const
{
	std::vector < T > ts;

	for ( const Entry& e : entries ) {
		if ( box.intersects( e.first ) ) {
			ts.push_back( e.second );
		}
	}

	return ts;
}

#endif
