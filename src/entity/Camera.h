#ifndef ENTITY_CAMERA_H
#define ENTITY_CAMERA_H

#include "Entity.h" // superclass Entity

/*
================================
Camera which centers on an Entity's AABB.
================================
*/
class Camera : public Entity
{
public: // Life cycle
	Camera( EntityState& es );
	virtual ~Camera();

public: // Entity functions
	virtual void input( const InputSet& is );
	virtual void update();
	virtual void draw( Renderer& rd ) const
		{ rd.drawCamera( *this ); }
	friend class Renderer;
	virtual AABB getAABB() const;

public: // Camera functions
	void addTarget( Entity* en );
	Entity* getTarget() const;

	Vec2 world( int wx, int wy );
	// std::pair < int, int > window( Vec2 );

	void zoomIn();
	void zoomOut();

private: // Members
	Euler* eu;
	std::list < Entity* > targets;

	// gluPerspective arguments
	double fov_y, aspect, z_near, z_far;

	int zoom;
	double z;
	double dz;
};

#endif
