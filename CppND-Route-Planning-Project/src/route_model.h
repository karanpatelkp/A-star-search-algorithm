#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>            // For numeric limits such as max float value.
#include <cmath>             // For math functions like sqrt and pow.
#include <unordered_map>     // For hash map container to map nodes to roads.
#include "model.h"           // Base Model class and related data structures.
#include <iostream>          // Included for debugging (not used explicitly here).

// --- RouteModel class ---
// Extends the base Model to add routing-specific features such as neighbors, 
// search state variables, and node-to-road mappings useful for pathfinding algorithms.

class RouteModel : public Model {

public:
    // --- Nested Node class ---
    // Extends Model::Node to include additional attributes and methods required for routing.

    class Node : public Model::Node {
    public:
        Node * parent = nullptr;
        // Pointer to the parent node used during pathfinding to reconstruct the path.

        float h_value = std::numeric_limits<float>::max();
        // Heuristic cost to the goal node (initialized to a very large number).

        float g_value = 0.0;
        // Cost from the start node to this node (initialized to zero).

        bool visited = false;
        // Flag to indicate whether this node has been visited during search.

        std::vector<Node *> neighbors;
        // List of pointers to neighboring nodes connected to this node.

        void FindNeighbors();
        // Populates the neighbors vector with valid adjacent nodes.

        float distance(Node other) const {
            // Calculates Euclidean distance to another node.
            return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2));
        }

        Node() {}
        // Default constructor.

        Node(int idx, RouteModel * search_model, Model::Node node)
            : Model::Node(node), parent_model(search_model), index(idx) {}
        // Constructor that initializes the RouteModel node from a base Model::Node,
        // sets the node's index, and stores a pointer to the parent RouteModel.

    private:
        int index;
        // Unique index of this node within the RouteModel's node list.

        Node * FindNeighbor(std::vector<int> node_indices);
        // Helper method to find the closest unvisited neighbor from a list of node indices.

        RouteModel * parent_model = nullptr;
        // Pointer back to the RouteModel instance that owns this node.
    };

    // --- RouteModel public interface ---

    RouteModel(const std::vector<std::byte> &xml);
    // Constructor: initializes the RouteModel from XML map data, 
    // creates RouteModel nodes and builds node-to-road lookup.

    Node &FindClosestNode(float x, float y);
    // Returns a reference to the node closest to the given (x,y) coordinates.

    auto &SNodes() { return m_Nodes; }
    // Accessor for the internal list of RouteModel nodes.

    std::vector<Node> path;
    // Stores the final path found by the routing algorithm as a sequence of nodes.

private:
    void CreateNodeToRoadHashmap();
    // Creates a hash map from node indices to the roads that include them,
    // used for efficient neighbor searches and graph traversal.

    std::unordered_map<int, std::vector<const Model::Road *>> node_to_road;
    // Maps each node index to a vector of pointers to roads that the node lies on.

    std::vector<Node> m_Nodes;
    // List of all RouteModel nodes created from the base Model nodes.
};

#endif
