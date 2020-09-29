//Owen Farthing
//2/21/20
#include "../PathSearch/PathSearch.h"
using namespace std;
using namespace chrono;
#pragma once

namespace ufl_cap4053
{
	namespace searches
	{

		std::priority_queue<std::pair<double, PathSearch::TileGraph::PlannerNode*>, std::vector<std::pair<double, PathSearch::TileGraph::PlannerNode*>>, std::greater<std::pair<double, PathSearch::TileGraph::PlannerNode*>>> priorityQueue;

		//Constructors
		//
		//TileWrapper
		PathSearch::TileGraph::TileWrapper::TileWrapper(Tile* tile) {
			this->current = tile;
			this->weight = tile->getWeight();
			this->x = tile->getXCoordinate();
			this->y = tile->getYCoordinate();
			this->neighbors = neighbors;
			this->radius = 0.0;
			this->visited = false;
		}

		//PlannerNode
		PathSearch::TileGraph::PlannerNode::PlannerNode(PathSearch::TileGraph::TileWrapper* current, PathSearch::TileGraph::PlannerNode* parent) {
			this->current = current;
			this->parent = parent;
			this->heuristicCost = 0.0;
			this->givenCost = 0.0;
			this->finalCost = 0.0;
		}

		//Search graph initialization
		PathSearch::TileGraph::TileGraph() {
			this->_tiles = _tiles;
		}

		//No-arg constructor explicity defined
		PathSearch::PathSearch() {}

		//Destructor calls unload() as a precaution
		PathSearch::~PathSearch() {
			this->unload();
		}

		//Definition of priority queue
		//ufl_cap4053::PriorityQueue< PathSearch::TileGraph::PlannerNode*> priorityQueue{ PathSearch::greaterThan };

		//Distance estimation for heuristic
		double PathSearch::estimateDistance(PathSearch::TileGraph::TileWrapper* begin, PathSearch::TileGraph::TileWrapper* end) {
			return (sqrt((pow((end->current->getXCoordinate() - begin->current->getXCoordinate()), 2)) + (pow((end->current->getYCoordinate() - begin->current->getYCoordinate()), 2)))) / begin->radius;
		}

		//////////////////////////////////////////////////////////////////////////////////////////////
		//
		//Main API
		//
		//Load
		//
		void PathSearch::load(TileMap* _tileMap) {

				//Loading _tileMap into search graph
				for (int i = 0; i < _tileMap->getRowCount(); i++) {
					for (int j = 0; j < _tileMap->getColumnCount(); j++) {
						PathSearch::TileGraph::TileWrapper* temp = new PathSearch::TileGraph::TileWrapper(_tileMap->getTile(i, j));
						temp->radius = _tileMap->getTileRadius();
						myGraph._tiles[{i,j}] = temp;
					}
				}

				//Identifying each tile's neighbors and storing them in graph
				std::unordered_map<std::pair<int, int>, PathSearch::TileGraph::TileWrapper*, PathSearch::hash_pair>::iterator itr = myGraph._tiles.begin();
				while (itr != myGraph._tiles.end()) {
					if (itr->second != nullptr) {
						int rowNum = itr->second->current->getRow();
						int colNum = itr->second->current->getColumn();
						bool untrav = false;

						//For untraversable tiles
						if (itr->second->weight == 0)
							untrav = true;
						//For odd rows
						if ((rowNum % 2) == 1 && untrav == false) {
							if ((rowNum - 1) >= 0)
								itr->second->neighbors.push_back(myGraph._tiles[{rowNum - 1, colNum}]);
							if ((rowNum - 1) >= 0 && (colNum + 1) < _tileMap->getColumnCount())
								itr->second->neighbors.push_back(myGraph._tiles[{rowNum - 1, colNum + 1}]);
							if ((colNum - 1) >= 0)
								itr->second->neighbors.push_back(myGraph._tiles[{rowNum, colNum - 1}]);
							if ((colNum + 1) < _tileMap->getColumnCount())
								itr->second->neighbors.push_back(myGraph._tiles[{rowNum, colNum + 1}]);
							if ((rowNum + 1) < _tileMap->getRowCount())
								itr->second->neighbors.push_back(myGraph._tiles[{rowNum + 1, colNum}]);
							if ((rowNum + 1) < _tileMap->getRowCount() && (colNum + 1) < _tileMap->getColumnCount())
								itr->second->neighbors.push_back(myGraph._tiles[{rowNum + 1, colNum + 1}]);
						}
						//For even rows
						else if ((rowNum % 2) == 0 && untrav == false) {
							if ((rowNum - 1) >= 0 && (colNum - 1) >= 0)
								itr->second->neighbors.push_back(myGraph._tiles[{rowNum - 1, colNum - 1}]);
							if ((rowNum - 1) >= 0)
								itr->second->neighbors.push_back(myGraph._tiles[{rowNum - 1, colNum}]);
							if ((colNum - 1) >= 0)
								itr->second->neighbors.push_back(myGraph._tiles[{rowNum, colNum - 1}]);
							if ((colNum + 1) < _tileMap->getColumnCount())
								itr->second->neighbors.push_back(myGraph._tiles[{rowNum, colNum + 1}]);
							if ((rowNum + 1) < _tileMap->getRowCount() && (colNum - 1) >= 0)
								itr->second->neighbors.push_back(myGraph._tiles[{rowNum + 1, colNum - 1}]);
							if ((rowNum + 1) < _tileMap->getRowCount())
								itr->second->neighbors.push_back(myGraph._tiles[{rowNum + 1, colNum}]);
						}
						untrav = false;
					}
					itr++;
				}

			}
			
			//Search variables initialized here
			PathSearch::TileGraph::TileWrapper* start = nullptr;
			PathSearch::TileGraph::TileWrapper* goal = nullptr;
			bool done = false;
			bool noSearch = false;

			//
			//Initialize
			//
			void PathSearch::initialize(int startRow, int startCol, int goalRow, int goalCol) {
			
				//Variables assigned
				done = false;
				start = myGraph._tiles[{startRow, startCol}];
				goal = myGraph._tiles[{goalRow, goalCol}];

				//Print message if user enters same coordinates for start and goal
				//Put starting tile in visited map and lock search algorithm
				if (startRow == goalRow && startCol == goalCol) {
					cout << "You are already at the goal.\n";
					//PathSearch::TileGraph::PlannerNode* startNode = new PathSearch::TileGraph::PlannerNode(start, nullptr);
					done = true;
					noSearch = true;
					PathSearch::TileGraph::PlannerNode* startNode = new PathSearch::TileGraph::PlannerNode(start, nullptr);
					startNode->givenCost = 0.0;
					startNode->heuristicCost = estimateDistance(start, goal);
					startNode->finalCost = startNode->heuristicCost * 1 + startNode->givenCost;
					nodes[{startRow, startCol}] = startNode;
					return;
				}
				else {
					//Getting ready for search
					PathSearch::TileGraph::PlannerNode* startNode = new PathSearch::TileGraph::PlannerNode(start,nullptr);
					start->visited = true;
					startNode->givenCost = 0.0;
					startNode->heuristicCost = estimateDistance(start, goal);
					startNode->finalCost = startNode->heuristicCost * 1 + startNode->givenCost;
					priorityQueue.emplace(startNode->finalCost,startNode);
					nodes[{startRow, startCol}] = startNode;
				}
			}
	
			//
			//Update
			//
			void PathSearch::update(long timeslice) {

				//For Play and Step
				if (timeslice == 0) {
					PathSearch::TileGraph::PlannerNode* temp = priorityQueue.top().second;
					priorityQueue.pop();

					if (temp->current->current == goal->current) {
						done = true;
						return;
					}

					for (unsigned int i = 0; i < temp->current->neighbors.size(); i++) {

						PathSearch::TileGraph::TileWrapper* successor = temp->current->neighbors[i];
						double tempGivenCost = (temp->givenCost + (2 * successor->radius * successor->weight));

						if (successor->visited) {
							PathSearch::TileGraph::PlannerNode* successorNode = nodes[{successor->current->getRow(), successor->current->getColumn()}];
							if (tempGivenCost < successorNode->givenCost) {
								successorNode->givenCost = tempGivenCost;
								successorNode->finalCost = successorNode->heuristicCost * 1 + successorNode->givenCost;
								successorNode->parent = temp;
								priorityQueue.emplace(successorNode->finalCost, successorNode);
							}
						}
						else {
							if (successor->weight != 0) {
								PathSearch::TileGraph::PlannerNode* successorNode = new PathSearch::TileGraph::PlannerNode(successor, temp);
								successorNode->givenCost = tempGivenCost;
								successorNode->heuristicCost = estimateDistance(successor, goal);
								successorNode->finalCost = successorNode->heuristicCost * 1 + successorNode->givenCost;
								priorityQueue.emplace(successorNode->finalCost, successorNode);
								successorNode->current->current->setFill(0x000000FF);
								successor->visited = true;
								nodes[{successor->current->getRow(), successor->current->getColumn()}] = successorNode;
							}
						}
					}
					return;
				}
				//For Timed Play
				else {

					//Start timer
					timer myTimer;

					while (!priorityQueue.empty() && myTimer.milliseconds_elapsed() < timeslice) {

						PathSearch::TileGraph::PlannerNode* temp = priorityQueue.top().second;
						priorityQueue.pop();

						if (temp->current->current == goal->current) {
							done = true;
							return;
						}

						for (unsigned int i = 0; i < temp->current->neighbors.size(); i++) {

							PathSearch::TileGraph::TileWrapper* successor = temp->current->neighbors[i];
							double tempGivenCost = (temp->givenCost + (2 * successor->radius * successor->weight));

							if (successor->visited) {
								PathSearch::TileGraph::PlannerNode* successorNode = nodes[{successor->current->getRow(), successor->current->getColumn()}];
								if (tempGivenCost < successorNode->givenCost) {
									successorNode->givenCost = tempGivenCost;
									successorNode->finalCost = successorNode->heuristicCost * 1 + successorNode->givenCost;
									successorNode->parent = temp;
									priorityQueue.emplace(successorNode->finalCost, successorNode);
								}
							}
							else {
								if (successor->weight != 0) {
									PathSearch::TileGraph::PlannerNode* successorNode = new PathSearch::TileGraph::PlannerNode(successor, temp);
									successorNode->givenCost = tempGivenCost;
									successorNode->heuristicCost = estimateDistance(successor, goal);
									successorNode->finalCost = successorNode->heuristicCost * 1 + successorNode->givenCost;
									priorityQueue.emplace(successorNode->finalCost, successorNode);
									//successorNode->current->current->setFill(0x000000FF);
									successor->visited = true;
									nodes[{successor->current->getRow(), successor->current->getColumn()}] = successorNode;
								}
							}
						}

					}
					//If we didn't find a solution and the algorithm wasn't locked
					//Print error message
					if (noSearch==false)
						cout << "Couldn't find solution.\n";
				}
				
			}

			//
			//Shutdown
			//
			void PathSearch::shutdown() {

				start = nullptr;
				goal = nullptr;
				noSearch = false;

				//Clear priority queue
				while (!priorityQueue.empty()) {
					priorityQueue.pop();
				}

				//Clear map entries that hold values
				std::unordered_map <std::pair<int,int>, PathSearch::TileGraph::PlannerNode*, PathSearch::hash_pair>::iterator itr = nodes.begin();
				while (itr != nodes.end()) {
					if (itr->second != nullptr) {
						itr->second->current->visited = false;
						PathSearch::TileGraph::PlannerNode* temp = itr->second;
						itr->second = nullptr;
						delete temp;
					}
					itr++;
				}
			
			}

			//
			//Unload
			//
			void PathSearch::unload() {
				
				//Popping all tiles and their stored neighbors
				//after deallocating their memory (if they are valid)
				std::unordered_map<std::pair<int, int>, PathSearch::TileGraph::TileWrapper*,PathSearch::hash_pair>::iterator itr = myGraph._tiles.begin();
				while (itr != myGraph._tiles.end()) {
					if (itr->second != nullptr) {
						while (!itr->second->neighbors.empty()) {
							itr->second->neighbors.pop_back();
						}
						PathSearch::TileGraph::TileWrapper* temp = itr->second;
						itr->second = nullptr;
						delete temp;
					}
					itr++;
				}
			
			}

			//
			//isDone
			//
			bool PathSearch::isDone() const { 
				if (done == true) {
					return true;
				}
				else
					return false;
			}

			//
			//getSolution
			//
			std::vector<Tile const*> const PathSearch::getSolution() const {
				std::vector<Tile const*> endSolution;

				//If we are already at the goal
				//Just push back the starting tile
				//And return the solution
				if (noSearch == true) {
					endSolution.push_back(nodes.at({ start->current->getRow(), start->current->getColumn() })->current->current);
					return endSolution;
				}
				//Otherwise, trace solution backwards through visited map
				//And push it into endSolution vector
				endSolution.push_back(nodes.at({ goal->current->getRow(),goal->current->getColumn() })->current->current);
				PathSearch::TileGraph::PlannerNode* itr = nodes.at({ goal->current->getRow(),goal->current->getColumn() });
				while (itr != nodes.at({start->current->getRow(), start->current->getColumn()}) && itr->parent != nullptr) {
					endSolution.push_back(itr->parent->current->current);
					itr = itr->parent;
				}

				return endSolution;

			}
	}
}  // close namespace ufl_cap4053::searches
