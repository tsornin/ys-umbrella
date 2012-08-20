#ifndef SPATIAL_VEC_3_H
#define SPATIAL_VEC_3_H

#include <utility>
#include "Scalar.h"
#include "Vec2.h"

/*
================================
Represents a (mathematical) three-dimensional vector
as three Scalar coordinates.

operator * returns the dot-product.
operator ^ returns the cross-product.
prod returns the pairwise product.
================================
*/
struct Vec3
{
public:
	Scalar x, y, z;

	/*
	================================
	Constructors
	================================
	*/
	Vec3() : x(0), y(0), z(0) {}
	Vec3( const Scalar& s ) : x(s), y(s), z(s) {}
	Vec3( const Scalar& x, const Scalar& y, const Scalar& z ) : x(x), y(y), z(z) {}
	Vec3( const Vec3& u, const Vec3& v ) : Vec3(v) { operator -= (u); }

	Vec3( const Vec2& v, const Scalar& z ) : x(v.x), y(v.y), z(z) {}

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
	friend bool operator == ( const Vec3& u, const Vec3& v ) { return ( u.x == v.x ) && ( u.y == v.y ) && ( u.z == v.z ); }
	friend bool operator != ( const Vec3& u, const Vec3& v ) { return !( u == v ); }

	/*
	================================
	Compound assignment operators
	================================
	*/
	Vec3& operator += ( const Vec3& v ) { x += v.x; y += v.y; z += v.z; return *this; }
	Vec3& operator -= ( const Vec3& v ) { x -= v.x; y -= v.y; z -= v.z; return *this; }

	Vec3& operator *= ( const Scalar& s ) { x *= s; y *= s; z *= s; return *this; }
	Vec3& operator /= ( const Scalar& s ) { x /= s; y /= s; z /= s; return *this; }

	/*
	================================
	Arithmetic operators
	================================
	*/
	const Vec3 operator + ( const Vec3& v ) const { return Vec3( *this ) += v; }
	const Vec3 operator - ( const Vec3& v ) const { return Vec3( *this ) -= v; }

	const Vec3 operator * ( const Scalar& s ) const { return Vec3( *this ) *= s; }
	const Vec3 operator / ( const Scalar& s ) const { return Vec3( *this ) /= s; }

	const Vec3 operator - () const { return Vec3( -x, -y, -z ); }

	/*
	================================
	Vector multiplication operators
	================================
	*/
	Scalar operator * ( const Vec3& v ) const { return ( x*v.x + y*v.y + z*v.z ); }
	Vec3 operator ^ ( const Vec3& v ) const {
		return Vec3(
			y*v.z - z*v.y,
			z*v.x - x*v.z,
			x*v.y - y*v.x );
	}

	Scalar dot  ( const Vec3& v ) const { return (*this) * v; }
	Vec3 cross( const Vec3& v ) const { return (*this) ^ v; }

	Vec3 prod( const Vec3& v ) const { return Vec3( x*v.x, y*v.y, z*v.z ); }

	/*
	================================
	Length functions
	================================
	*/
	Scalar length2() const { return dot( *this ); }

	Scalar length() const { return std::sqrt( length2() ); }

	Vec3 unit() const { return operator / ( length() ); }

	Scalar normalize() {
		Scalar ret = length();
		operator /= ( ret );
		return ret;
	}
};

#endif
