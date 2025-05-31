#include "route_model.h"    // Header file for the RouteModel class, extending the Model base class.
#include <iostream>         // For debugging and console output (not used explicitly in the code).

// --- Constructor Implementation ---

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    // Initializes the base Model with the XML map data (e.g., from OSM).
    
    int counter = 0;  // Used to assign indices to RouteModel nodes.
    for (Model::Node node : this->Nodes()) {
        // Iterate over all base Model nodes and convert them to RouteModel::Node instances.
        m_Nodes.emplace_back(Node(counter, this, node));
        counter++; // Increment index for the next node.
    }

    CreateNodeToRoadHashmap(); // Builds a hash map from node indices to roads that use them.
}


// --- Constructs a lookup table mapping each node to the roads it belongs to ---

void RouteModel::CreateNodeToRoadHashmap() {
    for (const Model::Road &road : Roads()) {
        // Iterate through all roads from the base model.

        if (road.type != Model::Road::Type::Footway) {
            // Skip footways (e.g., pedestrian paths), which are not used in A* pathfinding.

            for (int node_idx : Ways()[road.way].nodes) {
                // Iterate through node indices associated with the current road's way.

                if (node_to_road.find(node_idx) == node_to_road.end()) {
                    // If the node is not already in the map, initialize a vector of road pointers.
                    node_to_road[node_idx] = std::vector<const Model::Road *> ();
                }

                node_to_road[node_idx].push_back(&road);
                // Add a pointer to the current road for this node.
            }
        }
    }
}


// --- Finds the closest unvisited neighboring node from a given list of node indices ---

RouteModel::Node *RouteModel::Node::FindNeighbor(std::vector<int> node_indices) {
    Node *closest_node = nullptr;  // To store the nearest valid neighbor.
    Node node;                     // Temporary object for distance comparison.

    for (int node_index : node_indices) {
        // Iterate through each potential neighbor index.
        node = parent_model->SNodes()[node_index];  // Retrieve actual node from the model.

        if (this->distance(node) != 0 && !node.visited) {
            // Check that the node isn't the same (distance ≠ 0) and hasn't been visited.

            if (closest_node == nullptr || this->distance(node) < this->distance(*closest_node)) {
                // Update closest_node if this one is closer.
                closest_node = &parent_model->SNodes()[node_index];
            }
        }
    }

    return closest_node; // Return pointer to the closest unvisited neighbor, if any.
}


// --- Finds and stores all valid neighboring nodes for the current node ---

void RouteModel::Node::FindNeighbors() {
    for (auto & road : parent_model->node_to_road[this->index]) {
        // For each road that this node belongs to...

        RouteModel::Node *new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
        // Find the closest unvisited neighbor among the nodes on this road.

        if (new_neighbor) {
            // If a valid neighbor was found, add it to this node's neighbor list.
            this->neighbors.emplace_back(new_neighbor);
        }
    }
}


// --- Finds the node in the map closest to the given (x, y) coordinates ---

RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
    Node input;
    input.x = x;
    input.y = y;
    // Create a temporary node at the target coordinates to compare distances.

    float min_dist = std::numeric_limits<float>::max(); // Start with maximum possible distance.
    float dist;
    int closest_idx; // Index of the closest node found.

    for (const Model::Road &road : Roads()) {
        // Iterate over all roads.

        if (road.type != Model::Road::Type::Footway) {
            // Ignore footways (not used for vehicle pathfinding).

            for (int node_idx : Ways()[road.way].nodes) {
                // For each node on the current road...

                dist = input.distance(SNodes()[node_idx]);
                // Compute distance from input point to the node.

                if (dist < min_dist) {
                    // If this node is closer than previous ones, update the closest.
                    closest_idx = node_idx;
                    min_dist = dist;
                }
            }
        }
    }

    return SNodes()[closest_idx];
    // Return the node closest to the input coordinates.
}
