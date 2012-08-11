#include "MeshOBJ.h"

#include <string>
#include <fstream> // for ifstream
#include <sstream> // for istringstream

bool MTLlib::load( const Path& path )
{
	std::ifstream ifs( path.full().c_str(), std::ios::in );
	if ( !ifs ) return false;
	
	std::string usemtl; // active material
	while ( ifs ) {
		std::string line;
		std::getline( ifs, line );
		processLine( line, usemtl, path.directory );
	}
	return true;
}

void MTLlib::clear()
{
	// unload textures
	// for ( std::map < std::string, MTL >::iterator
	// i = mtls.begin(); i != mtls.end(); ++i ) {
		// MTL& mtl = i->second;
		// TextureManager::Instance()->unload( mtl.map_Kd );
	// }
	mtls.clear();
}

const MTL& MTLlib::get( const std::string& mtl_name ) const
{
	static MTL empty;
	std::map < std::string, MTL >::const_iterator i = mtls.find( mtl_name );
	return i != mtls.end() ? i->second : empty;
}

void MTLlib::processLine(
	const std::string& line, std::string& usemtl, const std::string& dir )
{
	MTL& mtl = mtls[ usemtl ];
	
	std::istringstream ss( line );
	if ( !ss ) return;
	std::string ele_id;
	ss >> ele_id;
	if ( "newmtl" == ele_id ) {
		ss >> usemtl;
	}
	else if ( "Ka" == ele_id ) {
		double r, g, b;
		ss >> r >> g >> b;
		mtls[ usemtl ].Ka = VertexOBJ( r, g, b );
	}
	else if ( "Kd" == ele_id ) {
		// Alternatively:
		ss >> mtl.Kd.x >> mtl.Kd.y >> mtl.Kd.z;
	}
	else if ( "Ks" == ele_id ) {
		// Alternatively (with anonymous struct):
		ss >> mtl.Ks.x >> mtl.Ks.y >> mtl.Kd.z;
	}
	else if ( "Tr" == ele_id || "d" == ele_id ) {
		ss >> mtl.Tr;
	}
	else if ( "illum" == ele_id ) {
		ss >> mtl.illum;
	}
	else if ( "map_Kd" == ele_id ) {
		std::string filename;
		ss >> filename;
		//Path p( dir, filename );
		//mtl.map_Kd.path = p;
		//TextureManager::Instance()->load( mtl.map_Kd );
	}
	else {
		// Ignored characteristics:
		// map_Ka, map_Kd, map_Ks,
		// ???
		return;
	}
}

bool MeshOBJ::load( const Path& path )
{
	std::ifstream ifs( path.full().c_str(), std::ios::in );
	if ( !ifs ) return false;
	
	// March through all lines.
	// TODO: line continuation character "\"
	std::string usemtl;
	while ( ifs ) {
		std::string line;
		std::getline( ifs, line );
		processLine( line, usemtl, path.directory );
	}
	return true;
}

void MeshOBJ::clear()
{
	verts.clear();
	texverts.clear();
	faces.clear();
	mtllib.clear();
}

VertexOBJ MeshOBJ::getVertex( const int index ) const
{
	if ( index > 0 )
		return verts[ index - 1 ];
	else // negative indices
		return verts[ verts.size() + index ];
}

VertexOBJ MeshOBJ::getTextureVertex( const int index ) const
{
	if ( index > 0 )
		return texverts[ index - 1 ];
	else // negative indices
		return texverts[ texverts.size() + index ];
}

void MeshOBJ::processLine(
	const std::string& line, std::string& usemtl, const std::string& dir )
{
	std::istringstream ss( line );
	if ( !ss ) return;
	std::string ele_id;
	ss >> ele_id;
	
	// string "switch" ( ele_id )
	if ( "v" == ele_id ) {
		double x, y, z;
		ss >> x >> y >> z;
		addVertex( VertexOBJ( x, y, z ) );
	}
	else if ( "vt" == ele_id ) {
		double x, y;
		ss >> x >> y;
		addTextureVertex( VertexOBJ( x, y ) );
	}
	else if ( "f" == ele_id ) {
		FaceOBJ f;
		char c; // holds the slash
		int vi, ti; // TODO: no normals
		while ( ss ) {
			// read in the vert (and tex) index(ices)
			// TODO: we can probably do this better
			ss >> vi;
			if ( texverts.size() ) ss >> c >> ti;
			
			if ( !ss ) break;
			
			f.addVertexIndex( vi );
			if ( texverts.size() ) f.addTextureVertexIndex( ti );
		}
		f.setMaterial( usemtl );
		addFace( f );
	}
	else if ( "mtllib" == ele_id ) {
		std::string filename;
		ss >> filename;
		mtllib.load( Path( dir, filename ) );
	}
	else if ( "usemtl" == ele_id ) {
		ss >> usemtl;
	}
	else {
		// Ignored elements:
		// vn, vp,
		// deg, bmat, step, cstype,
		// p, l, curv, curv2, surf,
		// param, trim, hole, scrv, sp, end,
		// con,
		// g, s, mg, o,
		// bevel, c_interp, d_interp, lod,
		// shadow_obj, trace_obj, ctech, stech
		return;
	}
}
