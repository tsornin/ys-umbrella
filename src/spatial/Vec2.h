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
	Vec2( const Vec2& u, const Vec2& v ) : Vec2(v) { operator -= (u); }

	/*
	================================
	Access operators
	================================
	*/
	Scalar& operator [] ( unsigned int i ) { return ( (Scalar*) this )[i]; }
	Scalar operator [] ( unsigned int i ) const { return ( (Scalar*) this )[i]; }

	Scalar& operator () ( unsigned int i ) { return operator [] (i); }
	Scalar operator () ( unsigned int i ) const { return operator [] (i); }

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
	Scalar operator * ( const Vec2& v ) const { return ( x*v.x + y*v.y ); }
	Scalar operator ^ ( const Vec2& v ) const { return ( x*v.y - y*v.x ); }

	Scalar dot  ( const Vec2& v ) const { return (*this) * v; }
	Scalar cross( const Vec2& v ) const { return (*this) ^ v; }

	Vec2 prod( const Vec2& v ) const { return Vec2( x*v.x, y*v.y ); }

	/*
	================================
	Length functions
	================================
	*/
	Scalar length2() const { return dot( *this ); }

	Scalar length() const { return std::sqrt( length2() ); }

	Vec2 unit() const { return operator / ( length() ); }

	Scalar normalize() {
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

	Scalar scalar_component( const Vec2& b ) const {
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

	Scalar theta() const { return std::atan2( y, x ); }

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
	static Scalar angle( const Vec2& u, const Vec2& v ) {
		return std::acos( u.dot( v ) / u.length() / v.length() );
	}

	// Interpolation
	static const Vec2 interp( const Vec2& u, const Vec2& v, const Scalar k ) {
		return u*k + v*( 1-k );
	}
};

#endif
