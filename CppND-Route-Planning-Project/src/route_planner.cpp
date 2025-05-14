#include "route_planner.h"
#include <algorithm>
#include <vector>

// Constructor: Initializes the route planner with a model and start/end coordinates
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y)
    : m_Model(model) {
    // Convert input coordinates from percentage to decimal
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes in the model to the given start and end points
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

// Helper function to calculate the heuristic value (straight-line distance to the goal)
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

// Expands the current node by adding all unvisited neighboring nodes to the open list
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Find all neighboring nodes for the current node
    current_node->FindNeighbors();

    for (auto neighbor : current_node->neighbors) {
        if (!neighbor->visited) {
            // Set up the neighbor's properties for pathfinding
            neighbor->parent = current_node;
            neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
            neighbor->h_value = CalculateHValue(neighbor);
            open_list.push_back(neighbor);  // Add neighbor to the open list
            neighbor->visited = true;       // Mark it as visited
        }
    }
}

// Chooses the next node to explore based on the lowest estimated total cost (f = g + h)
RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open list in descending order based on f-value (g + h)
    std::sort(open_list.begin(), open_list.end(), 
        [](RouteModel::Node* a, RouteModel::Node* b) {
            return (a->g_value + a->h_value) > (b->g_value + b->h_value);
        }
    );

    // Pick the node with the lowest total cost
    RouteModel::Node* next_node = open_list.back();
    open_list.pop_back();  // Remove it from the open list
    return next_node;
}

// Builds the final path by walking back from the end node to the start node
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Traverse the parent chain to build the path
    while (current_node != nullptr) {
        path_found.push_back(*current_node);

        if (current_node->parent != nullptr) {
            distance += current_node->distance(*(current_node->parent));
        }

        current_node = current_node->parent;  // Move to the parent
    }

    // Reverse the path so it goes from start to end
    std::reverse(path_found.begin(), path_found.end());

    // Scale distance to meters
    distance *= m_Model.MetricScale();
    return path_found;
}

// Main function to perform the A* Search
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;
    start_node->visited = true;
    open_list.push_back(start_node);

    // Continue searching while there are nodes to explore
    while (!open_list.empty()) {
        current_node = NextNode();

        // Check if we've reached the destination
        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }

        // Otherwise, expand the current node's neighbors
        AddNeighbors(current_node);
    }
}
