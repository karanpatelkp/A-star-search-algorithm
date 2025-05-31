// Includes model definition and data structures for map representation
#include "model.h"
// pugixml is used for parsing the OSM (XML) map file
#include "pugixml.hpp"
#include <iostream>
#include <string_view>
#include <cmath>
#include <algorithm>
#include <assert.h>

// Converts string OSM road types into enumerated Road::Type for internal usage
static Model::Road::Type String2RoadType(std::string_view type)
{
    // Each condition matches a common OSM road type string
    // and returns the internal enum type for that road
    if( type == "motorway" )        return Model::Road::Motorway;
    if( type == "trunk" )           return Model::Road::Trunk;
    if( type == "primary" )         return Model::Road::Primary;
    if( type == "secondary" )       return Model::Road::Secondary;    
    if( type == "tertiary" )        return Model::Road::Tertiary;
    if( type == "residential" )     return Model::Road::Residential;
    if( type == "living_street" )   return Model::Road::Residential;    
    if( type == "service" )         return Model::Road::Service;
    if( type == "unclassified" )    return Model::Road::Unclassified;
    if( type == "footway" )         return Model::Road::Footway;
    if( type == "bridleway" )       return Model::Road::Footway;
    if( type == "steps" )           return Model::Road::Footway;
    if( type == "path" )            return Model::Road::Footway;
    if( type == "pedestrian" )      return Model::Road::Footway;    
    return Model::Road::Invalid;    
}

// Converts landuse types in OSM XML to internal enum representation
static Model::Landuse::Type String2LanduseType(std::string_view type)
{
    if( type == "commercial" )      return Model::Landuse::Commercial;
    if( type == "construction" )    return Model::Landuse::Construction;
    if( type == "grass" )           return Model::Landuse::Grass;
    if( type == "forest" )          return Model::Landuse::Forest;
    if( type == "industrial" )      return Model::Landuse::Industrial;
    if( type == "railway" )         return Model::Landuse::Railway;
    if( type == "residential" )     return Model::Landuse::Residential;    
    return Model::Landuse::Invalid;
}

// Model constructor: loads the OSM XML data, converts to internal map, and sorts roads by type
Model::Model( const std::vector<std::byte> &xml )
{
    LoadData(xml);                 // Parse XML and populate map data structures
    AdjustCoordinates();          // Convert lat/lon to metric coordinates

    // Sort roads by type enum value for efficient access/rendering
    std::sort(m_Roads.begin(), m_Roads.end(), [](const auto &_1st, const auto &_2nd){
        return (int)_1st.type < (int)_2nd.type; 
    });
}

// Parses the XML data and fills all map components: bounds, nodes, ways, roads, buildings etc.
void Model::LoadData(const std::vector<std::byte> &xml)
{
    using namespace pugi; // pugixml namespace for XML parsing
    
    xml_document doc;
    if( !doc.load_buffer(xml.data(), xml.size()) )
        throw std::logic_error("failed to parse the xml file"); // Error if file is corrupted
    
    // Extract map bounds (min/max lat-lon)
    if( auto bounds = doc.select_nodes("/osm/bounds"); !bounds.empty() ) {
        auto node = bounds.first().node();
        m_MinLat = atof(node.attribute("minlat").as_string());
        m_MaxLat = atof(node.attribute("maxlat").as_string());
        m_MinLon = atof(node.attribute("minlon").as_string());
        m_MaxLon = atof(node.attribute("maxlon").as_string());
    }
    else 
        throw std::logic_error("map's bounds are not defined");

    std::unordered_map<std::string, int> node_id_to_num;

    // Iterate through all <node> tags in the XML to populate node list
    for( const auto &node: doc.select_nodes("/osm/node") ) {
        node_id_to_num[node.node().attribute("id").as_string()] = (int)m_Nodes.size();
        m_Nodes.emplace_back();        
        m_Nodes.back().y = atof(node.node().attribute("lat").as_string());
        m_Nodes.back().x = atof(node.node().attribute("lon").as_string());
    }

    std::unordered_map<std::string, int> way_id_to_num;    

    // Iterate through all <way> tags to collect geometry and semantic labels
    for( const auto &way: doc.select_nodes("/osm/way") ) {
        auto node = way.node();
        const auto way_num = (int)m_Ways.size();
        way_id_to_num[node.attribute("id").as_string()] = way_num;
        m_Ways.emplace_back();
        auto &new_way = m_Ways.back();
        
        // For each child of <way> either gather node references or tag info
        for( auto child: node.children() ) {
            auto name = std::string_view{child.name()}; 
            if( name == "nd" ) {
                // If child is a node reference
                auto ref = child.attribute("ref").as_string();
                if( auto it = node_id_to_num.find(ref); it != end(node_id_to_num) )
                    new_way.nodes.emplace_back(it->second);
            }
            else if( name == "tag" ) {
                // If child is a tag, determine what this way represents
                auto category = std::string_view{child.attribute("k").as_string()};
                auto type = std::string_view{child.attribute("v").as_string()};

                if( category == "highway" ) {
                    if( auto road_type = String2RoadType(type); road_type != Road::Invalid ) {
                        m_Roads.emplace_back();
                        m_Roads.back().way = way_num;
                        m_Roads.back().type = road_type;
                    }
                }
                if( category == "railway" ) {
                    m_Railways.emplace_back();
                    m_Railways.back().way = way_num;
                }                
                else if( category == "building" ) {
                    m_Buildings.emplace_back();
                    m_Buildings.back().outer = {way_num};
                }
                else if( category == "leisure" ||
                        (category == "natural" && (type == "wood"  || type == "tree_row" || type == "scrub" || type == "grassland")) ||
                        (category == "landcover" && type == "grass" ) ) {
                    m_Leisures.emplace_back();
                    m_Leisures.back().outer = {way_num};
                }
                else if( category == "natural" && type == "water" ) {
                    m_Waters.emplace_back();
                    m_Waters.back().outer = {way_num};
                }
                else if( category == "landuse" ) {
                    if( auto landuse_type = String2LanduseType(type); landuse_type != Landuse::Invalid ) {
                        m_Landuses.emplace_back();
                        m_Landuses.back().outer = {way_num};
                        m_Landuses.back().type = landuse_type;
                    }                    
                }
            }
        }
    }
    
    // Parse <relation> tags which can group multiple ways (e.g., multipolygon buildings or water)
    for( const auto &relation: doc.select_nodes("/osm/relation") ) {
        auto node = relation.node();
        auto noode_id = std::string_view{node.attribute("id").as_string()};
        std::vector<int> outer, inner;

        // Lambda to commit current multipolygon (mp)
        auto commit = [&](Multipolygon &mp) {
            mp.outer = std::move(outer);
            mp.inner = std::move(inner);
        };

        // Iterate children of the relation
        for( auto child: node.children() ) {
            auto name = std::string_view{child.name()}; 
            if( name == "member" ) {
                // <member> tags define role and geometry
                if( std::string_view{child.attribute("type").as_string()} == "way" ) {
                    if( !way_id_to_num.count(child.attribute("ref").as_string()) )
                        continue;
                    auto way_num = way_id_to_num[child.attribute("ref").as_string()];
                    if( std::string_view{child.attribute("role").as_string()} == "outer" )
                        outer.emplace_back(way_num);
                    else
                        inner.emplace_back(way_num);
                }
            }
            else if( name == "tag" ) { 
                // Interpret what type of multipolygon this is (building/water/landuse)
                auto category = std::string_view{child.attribute("k").as_string()};
                auto type = std::string_view{child.attribute("v").as_string()};
                if( category == "building" ) {
                    commit( m_Buildings.emplace_back() );
                    break;
                }
                if( category == "natural" && type == "water" ) {
                    commit( m_Waters.emplace_back() );
                    BuildRings(m_Waters.back());
                    break;
                }
                if( category == "landuse" ) {
                    if( auto landuse_type = String2LanduseType(type); landuse_type != Landuse::Invalid ) {
                        commit( m_Landuses.emplace_back() );
                        m_Landuses.back().type = landuse_type;
                        BuildRings(m_Landuses.back());
                    }
                    break;
                }
            }
        }
    }
}
