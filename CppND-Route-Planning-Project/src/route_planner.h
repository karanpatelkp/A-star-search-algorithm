#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"

// RoutePlanner class:
// Responsible for performing pathfinding on a RouteModel using the A* search algorithm.
// Stores start and end points, maintains the open list of nodes to explore,
// and constructs the final path after search completion.
class RoutePlanner {
  public:
    /**
     * Constructor
     * Initializes the RoutePlanner with a reference to the RouteModel and
     * start/end coordinates (expressed as percentages of the map dimensions).
     *
     * @param model Reference to the RouteModel containing the map data.
     * @param start_x X coordinate of the start point (percentage 0-100).
     * @param start_y Y coordinate of the start point (percentage 0-100).
     * @param end_x X coordinate of the end point (percentage 0-100).
     * @param end_y Y coordinate of the end point (percentage 0-100).
     */
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);

    /**
     * GetDistance
     * Returns the total distance of the computed path after A* search completes.
     *
     * @return Distance in meters (float).
     */
    float GetDistance() const { return distance; }

    /**
     * AStarSearch
     * Main method that performs the A* search algorithm to find the shortest
     * path between the start and end nodes on the RouteModel.
     * After completion, the path is stored in the model's path vector.
     */
    void AStarSearch();

    // The following methods are public to allow for unit testing:

    /**
     * AddNeighbors
     * Expands the current node by finding all unvisited neighbors, setting
     * their g and h values, marking them visited, and adding them to the open list.
     *
     * @param current_node Pointer to the node currently being expanded.
     */
    void AddNeighbors(RouteModel::Node *current_node);

    /**
     * CalculateHValue
     * Calculates the heuristic value (straight-line distance) from the given node
     * to the end node; used by A* to estimate cost to goal.
     *
     * @param node Pointer to the node for which the heuristic is calculated.
     * @return Heuristic cost as float.
     */
    float CalculateHValue(RouteModel::Node const *node);

    /**
     * ConstructFinalPath
     * Constructs the path by traversing parent pointers from the current node back
     * to the start node, accumulating total distance along the way.
     *
     * @param current_node Pointer to the end node where path construction starts.
     * @return Vector of nodes representing the final path from start to end.
     */
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *current_node);

    /**
     * NextNode
     * Selects the next node to explore from the open list based on the lowest
     * sum of g (cost so far) and h (heuristic) values.
     *
     * @return Pointer to the next node to explore.
     */
    RouteModel::Node *NextNode();

  private:
    // List of nodes pending exploration during the search.
    std::vector<RouteModel::Node*> open_list;

    // Pointers to the starting and ending nodes within the model.
    RouteModel::Node *start_node;
    RouteModel::Node *end_node;

    // Total distance of the final path found by A* search, in meters.
    float distance = 0.0f;

    // Reference to the RouteModel on which pathfinding operates.
    RouteModel &m_Model;
};

#endif
