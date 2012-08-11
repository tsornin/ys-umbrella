#ifndef MESH_OBJ_H
#define MESH_OBJ_H

#include <string> // for filename, processLine, material names
#include <vector> // for vertex/face lists
#include <map> // for material names
#include "Path.h"

/**
 * An incomplete OBJ/MTL parser.
 *
 * http://netghost.narod.ru/gff/graphics/summary/waveobj.htm
 * http://people.sc.fsu.edu/~jburkardt/data/mtl/mtl.html
 */

// OBJ format vertex.
struct VertexOBJ
{
public:
	double x, y, z, w;
	
public:
	VertexOBJ() : x(0), y(0), z(0), w(1.0) { }
	VertexOBJ( double x_, double y_, double z_=0.0, double w_=1.0 )
		: x(x_), y(y_), z(z_), w(w_) { }
};

// MTL format material.
struct MTL
{
public:
	VertexOBJ Ks, Ka, Kd;
	double Tr, Ns;
	int illum;
	//Texture map_Kd;
	
public:
	MTL() : Tr( 1.0 ), Ns( 1.0 ), illum( 1 ) { }
};

// MTL format library.
// TODO: currently one library per mesh (see MeshOBJ)
struct MTLlib
{
public:
	std::map < std::string, MTL > mtls;
	
public:
	bool load( const Path& path );
	// bool save( const std::string& filename );
	void clear(); // TODO: call glDeleteTextures
	
	const MTL& get( const std::string& mtl_name ) const;
	
private:
	void processLine(
		const std::string& line, std::string& usemtl, const std::string& dir );
};

// OBJ format face.
struct FaceOBJ
{
public:
	std::vector< int > vis;
	std::vector< int > tis;
	std::string mtl_name;
	
public:
	void addVertexIndex( const int k )
		{ vis.push_back( k ); }
	
	void addTextureVertexIndex( const int k )
		{ tis.push_back( k ); }
	
	void setMaterial( const std::string& mtl_name_ )
		{ mtl_name = mtl_name_; };
};

// OBJ format mesh.
// TODO: currently one library per mesh (see MTLlib)
// TODO: data structure for groups?
struct MeshOBJ
{
public:
	std::vector < VertexOBJ > verts;
	std::vector < VertexOBJ > texverts;
	std::vector < FaceOBJ > faces;
	MTLlib mtllib;
	double scale;
	
public:
	MeshOBJ() : scale( 1.0 ) { }
	
	bool load( const Path& path );
	// bool save( const std::string& filename );
	void clear();
	
	VertexOBJ getVertex( const int index ) const;
	VertexOBJ getTextureVertex( const int index ) const;
	
	void setScale( double s )
		{ scale = s; }
	
private:
	void processLine(
		const std::string& line, std::string& usemtl, const std::string& dir );
	
	void addVertex( const VertexOBJ& v )
		{ verts.push_back( v ); }
	
	void addTextureVertex( const VertexOBJ& v )
		{ texverts.push_back( v ); }
	
	void addFace( const FaceOBJ& f )
		{ faces.push_back( f ); }
};

#endif
