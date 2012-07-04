#ifndef SPATIAL_VEC_2_H
#define SPATIAL_VEC_2_H

#include <utility> // for std::pair
#include "Scalar.h"

/*
================================
Represents a (mathematical) two-dimensional vector
as two Scalar coordinates.

operator * returns the dot-product.
operator ^ returns the cross-product (a Scalar in 2D).
prod returns the pairwise product.
================================
*/
struct Vec2
{
public:
	Scalar x, y;

	/*
	================================
	Constructors
	================================
	*/
	Vec2() : x(0), y(0) {}
	Vec2( const Scalar& s ) : x(s), y(s) {}
	Vec2( const Scalar& x, const Scalar& y ) : x(x), y(y) {}
	Vec2( const Vec2& u, const Vec2& v ) { (*this) = v - u; }
	// TODO: Delegating constructor Vec2( const Vec2& u, const Vec2& v ) : Vec2(v) { operator -= (u); }

	/*
	================================
	Access operators
	================================
	*/
	Scalar& operator [] ( const unsigned int i ) { return ( (Scalar*) this )[i]; }
	const Scalar operator [] ( const unsigned int i ) const { return ( (Scalar*) this )[i]; }

	Scalar& operator () ( const unsigned int i ) { return operator [] (i); }
	const Scalar operator () ( const unsigned int i ) const { return operator [] (i); }

	/*
	================================
	Comparison operators
	================================
	*/
	friend bool operator == ( const Vec2& u, const Vec2& v ) { return ( u.x == v.x ) && ( u.y == v.y ); }
	friend bool operator != ( const Vec2& u, const Vec2& v ) { return !( u == v ); }

	/*
	================================
	Compound assignment operators
	================================
	*/
	Vec2& operator += ( const Vec2& v ) { x += v.x; y += v.y; return *this; }
	Vec2& operator -= ( const Vec2& v ) { x -= v.x; y -= v.y; return *this; }

	Vec2& operator *= ( const Scalar& s ) { x *= s; y *= s; return *this; }
	Vec2& operator /= ( const Scalar& s ) { x /= s; y /= s; return *this; }

	/*
	================================
	Arithmetic operators
	================================
	*/
	const Vec2 operator + ( const Vec2& v ) const { return Vec2( *this ) += v; }
	const Vec2 operator - ( const Vec2& v ) const { return Vec2( *this ) -= v; }

	const Vec2 operator * ( const Scalar& s ) const { return Vec2( *this ) *= s; }
	const Vec2 operator / ( const Scalar& s ) const { return Vec2( *this ) /= s; }

	const Vec2 operator - () const { return Vec2( -x, -y ); }

	/*
	================================
	Vector multiplication operators
	================================
	*/
	const Scalar operator * ( const Vec2& v ) const { return ( x*v.x + y*v.y ); }
	const Scalar operator ^ ( const Vec2& v ) const { return ( x*v.y - y*v.x ); }

	const Scalar dot  ( const Vec2& v ) const { return (*this) * v; }
	const Scalar cross( const Vec2& v ) const { return (*this) ^ v; }

	/*
	================================
	Length functions
	================================
	*/
	const Scalar length2() const { return dot( *this ); }

	const Scalar length() const { return std::sqrt( length2() ); }

	const Vec2 unit() const { return operator / ( length() ); }

	const Scalar normalize() {
		Scalar ret = length();
		operator /= ( ret );
		return ret;
	}

	bool limit( const Scalar d ) {
		if ( length2() > d*d ) {
			operator *= ( d / length() );
			return true;
		}
		return false;
	}

	/*
	================================
	Projection functions
	================================
	*/
	const Vec2 projection( const Vec2& b ) const {
		return b * ( this->dot( b ) / b.dot( b ) );
	}

	const Vec2 rejection( const Vec2& b ) const {
		return (*this) - projection( b );
	}

	std::pair < Vec2, Vec2 > split( const Vec2& b ) {
		Vec2 p = projection( b );
		return std::pair < Vec2, Vec2 >( p, (*this) - p );
	}

	const Vec2 projection_unit( const Vec2& b ) const {
		return b * ( this->dot( b ) );
	}

	const Scalar scalar_component( const Vec2& b ) const {
		return this->dot( b.unit() );
	}

	/*
	================================
	Rotation functions
	================================
	*/
	const Vec2 rotation( const Scalar& rad ) const {
		Scalar sin = std::sin( rad );
		Scalar cos = std::cos( rad );
		return Vec2(
			x * cos - y * sin,
			x * sin + y * cos );
	}

	const Vec2 rotation( const Scalar& cos, const Scalar& sin ) const {
		return Vec2(
			x * cos - y * sin,
			x * sin + y * cos );
	}

	const Scalar theta() const { return std::atan2( y, x ); }

	/*
	================================
	Perpendicular functions
	================================
	*/
	const Vec2 lperp() const { return Vec2( -y, x ); }
	const Vec2 rperp() const { return Vec2( y, -x ); }

	/*
	================================
	Miscellaneous functions
	================================
	*/
	// What quadrant is v in (this vector is the origin)
	unsigned int quadrant( const Vec2& v ) const
	{
		unsigned int ret = 0;
		for ( int i = 2-1; i >= 0; --i ) {
			ret <<= 1;
			if ( v[i] < (*this)[i] ) ret += 1;
		}
		return ret;
	}

	// Returns a vector with all components = 1 in a specified quadrant
	// TODO: this is pretty weird...
	static Vec2 unquadrant( unsigned int quad ) {
		Vec2 q(1);
		for ( int i = 0; i < 2; ++i ) {
			if ( quad % 2 ) q[i] = -q[i];
			quad >>= 1;
		}
		return q;
	}

	// Pairwise product
	static const Vec2 prod( const Vec2& u, const Vec2& v ) {
		return Vec2( u.x*v.x, u.y*v.y );
	}

	// Smaller angle between two vectors
	static const Scalar angle( const Vec2& u, const Vec2& v ) {
		return std::acos( u.dot( v ) / u.length() / v.length() );
	}

	// Interpolation
	static const Vec2 interp( const Vec2& u, const Vec2& v, const Scalar k ) {
		return u*k + v*( 1-k );
	}
};

#endif
