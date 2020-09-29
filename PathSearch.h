//Owen Farthing
//2/21/20
#include "../../Source/Framework/platform.h"
#include "../../Source/Framework/TileSystem/Tile.h"
#include "../../Source/Framework/TileSystem/TileMap.h"
#include "../../Source/PriorityQueue.h"
#include <queue>
#include <cmath>
#include <iostream>
#include <chrono>
#include <unordered_map>
#pragma once

namespace ufl_cap4053
{
	namespace searches
	{
			//Main class
			class PathSearch
			{
			public:

				//Struct for defining custom hashing function
				//Used to hash pairs for std::unordered_map
				struct hash_pair {

					template <class T1, class T2>
					size_t operator()(const std::pair<T1, T2>& p) const {
						auto hash1 = std::hash<T1>{}(p.first);
						auto hash2 = std::hash<T1>{}(p.second);
						return hash1 ^ hash2;
					}

				};

				//Struct to create and set up timer
				struct timer {

					typedef std::chrono::milliseconds msecs;
					typedef std::chrono::steady_clock clock;

					unsigned long long milliseconds_elapsed() const {
						return std::chrono::duration_cast<msecs>(clock::now() - startClock).count();
					}
				private:
					clock::time_point startClock = clock::now();

				};

				//Search graph
				class TileGraph {

				public:

					//Wrapper struct for tiles
					struct TileWrapper {
					public:
						Tile* current;
						unsigned char weight;
						double x, y;
						std::vector<TileWrapper*> neighbors;
						bool visited;
						double radius;

						DLLEXPORT TileWrapper(Tile* tile);
					};

					//Wrapper struct for tile wrappers (used for A*)
					struct PlannerNode {
					public:
						TileWrapper* current;
						PlannerNode* parent;
						double heuristicCost;
						double givenCost;
						double finalCost;

						DLLEXPORT PlannerNode(TileWrapper* current, PlannerNode* parent);
					};

					//Map for search graph
					std::unordered_map<std::pair<int, int>, TileWrapper*, hash_pair> _tiles;

					//Constructor
					DLLEXPORT TileGraph();

				};

				//Body of PathSearch class
			private:

				//Heuristic cost calculator
				double estimateDistance(TileGraph::TileWrapper* begin, TileGraph::TileWrapper* end);

			public:

				//Graph
				TileGraph myGraph;

				//Search variables
				PathSearch::TileGraph::TileWrapper* start;
				PathSearch::TileGraph::TileWrapper* goal;
				bool done;
				bool noSearch;

				//Visited map
				std::unordered_map<std::pair<int,int>, TileGraph::PlannerNode*,hash_pair> nodes;
				
				//Constructor and destructor
				DLLEXPORT PathSearch();

				DLLEXPORT ~PathSearch();

				//API
				DLLEXPORT void load(TileMap* _tileMap);

				DLLEXPORT void initialize(int startRow, int startCol, int goalRow, int goalCol);

				DLLEXPORT void update(long timeslice);

				DLLEXPORT void shutdown();

				DLLEXPORT void unload();

				DLLEXPORT bool isDone() const;

				DLLEXPORT std::vector<Tile const*> const getSolution() const;
			};
	}
}  // close namespace ufl_cap4053::searches
