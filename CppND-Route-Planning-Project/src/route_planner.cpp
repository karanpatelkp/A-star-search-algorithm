#include "route_planner.h"
#include <algorithm>  // For std::sort and std::reverse
#include <vector>     // For std::vector container

// --- RoutePlanner class implementation ---

// Constructor:
// Initializes the RoutePlanner with a RouteModel reference and start/end coordinates (percentage values).
// Converts input coordinates from percentages (0-100) to decimals (0-1) before finding closest nodes in the model.
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y)
    : m_Model(model) {
    start_x *= 0.01f;
    start_y *= 0.01f;
    end_x *= 0.01f;
    end_y *= 0.01f;

    // Find and store pointers to the closest nodes to start and end coordinates.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// CalculateHValue:
// Computes the heuristic value (h) used in A* search.
// Heuristic is the straight-line (Euclidean) distance from the current node to the goal node.
// Input: pointer to current node.
// Output: float heuristic estimate to goal.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

// AddNeighbors:
// Expands the current node by adding all unvisited neighboring nodes to the open list for exploration.
// Sets neighbor node's parent to current_node, calculates g and h values, marks as visited, and adds to open_list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Populate neighbors vector of current node.
    current_node->FindNeighbors();

    // Iterate over all neighbors.
    for (auto neighbor : current_node->neighbors) {
        if (!neighbor->visited) {
            neighbor->parent = current_node;  // Set parent for path reconstruction.
            neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);  // Cost so far.
            neighbor->h_value = CalculateHValue(neighbor);  // Heuristic cost to goal.
            open_list.push_back(neighbor);  // Add neighbor to the open list.
            neighbor->visited = true;       // Mark neighbor as visited.
        }
    }
}

// NextNode:
// Selects the next node to explore from the open list based on lowest total estimated cost (f = g + h).
// Sorts the open list in descending order of f-value, then pops and returns the last element (lowest f).
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(),
        [](RouteModel::Node* a, RouteModel::Node* b) {
            return (a->g_value + a->h_value) > (b->g_value + b->h_value);
        }
    );

    RouteModel::Node* next_node = open_list.back();
    open_list.pop_back();
    return next_node;
}

// ConstructFinalPath:
// Constructs the path by walking backwards from the current (goal) node to the start node via parent pointers.
// Calculates total distance traveled and returns the path vector from start to end.
// Distance is scaled to meters using model's MetricScale.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;                      // Reset path distance accumulator.
    std::vector<RouteModel::Node> path_found;

    // Traverse from end node to start node via parent pointers.
    while (current_node != nullptr) {
        path_found.push_back(*current_node);  // Add current node to path.

        if (current_node->parent != nullptr) {
            distance += current_node->distance(*(current_node->parent));  // Accumulate distance.
        }

        current_node = current_node->parent;  // Move to parent node.
    }

    // Reverse the path to get start-to-end order.
    std::reverse(path_found.begin(), path_found.end());

    // Scale total distance according to model scale (meters).
    distance *= m_Model.MetricScale();

    return path_found;
}

// AStarSearch:
// Performs the A* search algorithm to find the shortest path from start_node to end_node.
// Uses an open list to keep track of nodes to explore.
// Expands nodes until the end node is reached, then constructs the final path.
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;

    start_node->visited = true;
    open_list.push_back(start_node);

    // Loop until there are no nodes left to explore or destination is reached.
    while (!open_list.empty()) {
        current_node = NextNode();

        // Check if goal node reached.
        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(current_node);
            return;  // Path found; exit search.
        }

        // Expand neighbors of current node.
        AddNeighbors(current_node);
    }
}
