#ifndef PHYSICS_GRAPH_H
#define PHYSICS_GRAPH_H

#include <set>
#include <vector>
#include <queue>
#include <algorithm>

/*
================================
???

// V subclass Vertex (CRTP)
// E subclass Edge (CRTP)
// V implements frozen()
================================
*/
template < typename V, typename E >
class PhysicsGraph
{
public:
	class Vertex {
	public:
		Vertex() :
			marked( false ),
			component_id( -1 ),
			local_id( -1 ) {
			;
		}

		void insertVertex() {

		}

		void eraseVertex() {
			// Can't iterate edges (removeEdge modifies our edges)
			auto edges_copy = edges;
			for ( E* e : edges_copy ) {
				e->eraseEdge();
			}
		}

		// Edges
		// Invariant:
		// for ( E* e : edges ) assert( e->a == this || e->b == this );
		std::set < E* > edges;

		bool marked;
		int component_id; // The ID of the island this vertex belongs to
		int local_id; // The ID this vertex within its island
	};

	class Edge {
	public:
		Edge( V* a, V* b ) : a(a), b(b) {

		}

		void insertEdge() {
			E* e = static_cast < E* >( this );
			a->edges.insert( e );
			b->edges.insert( e );
		}

		void eraseEdge() {
			E* e = static_cast < E* >( this );
			a->edges.erase( e );
			b->edges.erase( e );
		}

		// Vertices
		V
			// Invariant: assert( a->edges.contains( this ) );
			*a, // source vertex (reference)
			// Invariant: assert( b->edges.contains( this ) );
			*b; // target vertex (incident)
	};

	typedef std::pair < std::vector < V* >, std::vector < E* > > Island;

	/*
	================================
	PhysicsGraph::find_islands

	Finds all connected components among the specified Vertexs.

	This is a slightly modified connected-components algorithm:
		1. Edge-less vertices are not considered components.
		2. "Frozen" vertices belong to as many components as they have edges.
	================================
	*/
	static std::vector < Island > find_islands( std::vector < V* >& vs ) {
		// Pre-condition
		// for ( V* v : vs ) assert( ! v->marked );

		std::vector < Island > islands;

		// Pre-processing: we'll skip edge-less and frozen vertices
		// Pre-processing: reset ID tags
		for ( V* v : vs ) {
			v->marked = v->edges.empty() || v->frozen();
			v->component_id = -1;
			v->local_id = -1;
		}

		// Undirected connected components algorithm
		for ( V* v : vs ) {
			if ( v->marked ) continue;
			islands.push_back( mark_connected( v ) );
		}

		// Post-condition
		// for ( V* v : vs ) assert( v->marked );

		// Set ID tags
		for ( unsigned int j = 0; j < islands.size(); ++j ) {
			auto& vs = islands[j].first;
			for ( unsigned int i = 0; i < vs.size(); ++i ) {
				V* v = vs[i];
				v->component_id = j;
				v->local_id = i;
			}
		}

		// Restore invariant
		for ( V* v : vs ) v->marked = false;

		return islands;
	}

	/*
	================================
	PhysicsGraph::mark_connected

	Returns all Vertexs and Edges in
	the connected component of the specified Vertex.
	================================
	*/
	static Island mark_connected( V* root ) {
		Island island;
		auto& vs = island.first;
		auto& es = island.second;

		// Breadth-first search
		std::queue < V* > unseen;
		unseen.push( root );
		vs.push_back( root );
		root->marked = true;

		while ( ! unseen.empty() ) {
			V* v = unseen.front();
			unseen.pop();

			for ( E* e : v->edges ) {
				// assert( e->a == v || e->b == v );
				V* w = ( e->a == v ) ? e->b : e->a;

				// Include and mark, but do not expand, frozen nodes.
				// We add frozen nodes even though they are marked:
				// this means that we consider frozen nodes with multiple
				// neighbors to belong to multiple connected components.
				if ( w->frozen() ) {
					vs.push_back( w );
				}
				// Normal BFS on non-frozen nodes.
				else if ( ! w->marked ) {
					unseen.push( w );
					vs.push_back( w );
					w->marked = true;
				}

				// This finds all edges twice
				es.push_back( e );
			}
		}

		// Remove duplicate vertices (frozen nodes)
		std::sort( vs.begin(), vs.end() );
		vs.erase( std::unique( vs.begin(), vs.end() ), vs.end() );

		// Remove duplicate edges
		std::sort( es.begin(), es.end() );
		es.erase( std::unique( es.begin(), es.end() ), es.end() );

		// Post-condition
		// for ( V* v : vs ) assert( v->marked );

		return island;
	}
};

#endif
