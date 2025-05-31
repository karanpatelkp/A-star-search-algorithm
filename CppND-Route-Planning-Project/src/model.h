#pragma once  // Ensure this header is included only once during compilation

#include <vector>              // For std::vector used in node and way containers
#include <unordered_map>      // May be used for ID-to-index mapping in source file
#include <string>             // For string operations
#include <cstddef>            // For std::byte, used in binary data parsing

// The Model class represents a parsed map model from XML (e.g., OSM data)
class Model
{
public:
    // Represents a geographic point (node) with converted metric coordinates
    struct Node {
        double x = 0.f;  // X position in meters (converted from longitude)
        double y = 0.f;  // Y position in meters (converted from latitude)
    };
    
    // Represents an OSM 'way', which is a sequence of node indices
    struct Way {
        std::vector<int> nodes;  // Indices into the m_Nodes vector
    };
    
    // Represents a road, which wraps a 'Way' and classifies it by type
    struct Road {
        enum Type { Invalid, Unclassified, Service, Residential,
            Tertiary, Secondary, Primary, Trunk, Motorway, Footway };  // Road categories
        int way;     // Index into m_Ways for the geometry
        Type type;   // Road type
    };
    
    // Represents a railway, which also wraps a way
    struct Railway {
        int way;     // Index into m_Ways vector
    };    
    
    // Represents a multipolygon, with outer and inner boundary ways
    struct Multipolygon {
        std::vector<int> outer;  // Outer way indices
        std::vector<int> inner;  // Inner way indices (holes)
    };
    
    // Buildings are represented as multipolygons
    struct Building : Multipolygon {};

    // Leisure areas (e.g., parks) are also multipolygons
    struct Leisure : Multipolygon {};

    // Water bodies (e.g., lakes, ponds)
    struct Water : Multipolygon {};

    // Land use areas (e.g., forest, residential) with a specific type
    struct Landuse : Multipolygon {
        enum Type { Invalid, Commercial, Construction, Grass, Forest, Industrial, Railway, Residential };  // Usage types
        Type type;  // Type of land use
    };
    
    // Constructor: parses the XML (in byte format) and initializes the model
    Model( const std::vector<std::byte> &xml );
    
    // Returns the metric scaling factor (degrees to meters)
    auto MetricScale() const noexcept { return m_MetricScale; }    
    
    // Getters for accessing internal data structures
    auto &Nodes() const noexcept { return m_Nodes; }
    auto &Ways() const noexcept { return m_Ways; }
    auto &Roads() const noexcept { return m_Roads; }
    auto &Buildings() const noexcept { return m_Buildings; }
    auto &Leisures() const noexcept { return m_Leisures; }
    auto &Waters() const noexcept { return m_Waters; }
    auto &Landuses() const noexcept { return m_Landuses; }
    auto &Railways() const noexcept { return m_Railways; }
    
private:
    // Converts lat/lon to metric coordinates and normalizes the map
    void AdjustCoordinates();

    // Constructs outer and inner boundary rings for a multipolygon
    void BuildRings( Multipolygon &mp );

    // Loads and parses the XML data into model components
    void LoadData(const std::vector<std::byte> &xml);
    
    // Internal storage of parsed map data
    std::vector<Node> m_Nodes;          // All nodes
    std::vector<Way> m_Ways;            // All ways
    std::vector<Road> m_Roads;          // All roads
    std::vector<Railway> m_Railways;    // All railways
    std::vector<Building> m_Buildings;  // All buildings
    std::vector<Leisure> m_Leisures;    // All leisure areas
    std::vector<Water> m_Waters;        // All water bodies
    std::vector<Landuse> m_Landuses;    // All land use areas
    
    // Bounds used to convert coordinates and normalize the map
    double m_MinLat = 0.;   // Minimum latitude
    double m_MaxLat = 0.;   // Maximum latitude
    double m_MinLon = 0.;   // Minimum longitude
    double m_MaxLon = 0.;   // Maximum longitude
    double m_MetricScale = 1.f;  // Conversion scale from degrees to meters
};
