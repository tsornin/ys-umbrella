#ifndef PATH_H
#define PATH_H

#include <string>

/**
 * Directory + filename path.
 */
struct Path
{
public:
	std::string directory;
	std::string filename;

public:
	// Path - empty
	Path() { }
	// Path - from std::string
	Path( const std::string& d, const std::string& f )
		: directory( d ), filename( f ) { }
	// Path - from string literal
	Path( const char* d, const char* f )
		: directory( d ), filename( f ) { }

	/**
	 * Returns the entire local path (directory + filename);
	 * suitable for loading files.
	 */
	std::string full() const { return directory + filename; }

	friend bool operator < ( const Path& lhs, const Path& rhs )
	{
		return lhs.full() < rhs.full();
	}

	friend bool operator == ( const Path& lhs, const Path& rhs )
	{
		return // filename-first "heuristic"
			( lhs.filename == rhs.filename ) &&
			( lhs.directory == rhs.directory );
	}
};

#endif
