#pragma once
// Ensures the file is included only once during compilation to avoid duplicate definitions.

#include <unordered_map>           // For using hash maps (associative containers).
#include <io2d.h>                  // For 2D rendering using the ISO C++ 2D graphics API.
#include "route_model.h"          // Contains RouteModel class used for map data.

using namespace std::experimental;  // Enables usage of std::experimental::io2d namespace elements directly.

class Render {
public:
    Render(RouteModel &model );
    // Constructor that takes a reference to a RouteModel instance (the data to be rendered).

    void Display(io2d::output_surface &surface);
    // Public method to render the entire map on the given output surface.

private:
    // --- Methods to precompute/prepare rendering data ---
    void BuildRoadReps();       // Sets up styles for different road types.
    void BuildLanduseBrushes(); // Initializes brushes for different land use types.

    // --- Drawing methods for different map features ---
    void DrawBuildings(io2d::output_surface &surface) const;   // Draws buildings.
    void DrawHighways(io2d::output_surface &surface) const;    // Draws roads/highways.
    void DrawRailways(io2d::output_surface &surface) const;    // Draws railway tracks.
    void DrawLeisure(io2d::output_surface &surface) const;     // Draws leisure areas (parks, etc.).
    void DrawWater(io2d::output_surface &surface) const;       // Draws water bodies.
    void DrawLanduses(io2d::output_surface &surface) const;    // Draws land use regions.
    void DrawStartPosition(io2d::output_surface &surface) const; // Draws the start point.
    void DrawEndPosition(io2d::output_surface &surface) const;   // Draws the end point.
    void DrawPath(io2d::output_surface &surface) const;        // Draws the computed path between points.

    // --- Helper functions to create drawable paths from data ---
    io2d::interpreted_path PathFromWay(const Model::Way &way) const;         // Converts a Way into a drawable path.
    io2d::interpreted_path PathFromMP(const Model::Multipolygon &mp) const;  // Converts a Multipolygon to a drawable path.
    io2d::interpreted_path PathLine() const;                                 // Generates a path line, probably for routing.

    // --- Member Variables ---

    RouteModel &m_Model;
    // Reference to the RouteModel object containing map data (e.g., roads, buildings, water bodies).

    float m_Scale = 1.f;
    // Scale factor used for rendering transformations.

    float m_PixelsInMeter = 1.f;
    // Conversion factor between map units (meters) and screen pixels.

    io2d::matrix_2d m_Matrix;
    // Transformation matrix used for scaling and translating coordinates to fit the screen.

    // --- Background brush ---
    io2d::brush m_BackgroundFillBrush{ io2d::rgba_color{238, 235, 227} };
    // Brush to paint the map background (light beige).

    // --- Building brushes and stroke properties ---
    io2d::brush m_BuildingFillBrush{ io2d::rgba_color{208, 197, 190} };
    // Fill color for buildings.

    io2d::brush m_BuildingOutlineBrush{ io2d::rgba_color{181, 167, 154} };
    // Outline color for buildings.

    io2d::stroke_props m_BuildingOutlineStrokeProps{1.f};
    // Stroke width for building outlines.

    // --- Leisure area brushes ---
    io2d::brush m_LeisureFillBrush{ io2d::rgba_color{189, 252, 193} };
    // Fill color for leisure areas.

    io2d::brush m_LeisureOutlineBrush{ io2d::rgba_color{160, 248, 162} };
    // Outline color for leisure areas.

    io2d::stroke_props m_LeisureOutlineStrokeProps{1.f};
    // Stroke width for leisure area outlines.

    // --- Water bodies brush ---
    io2d::brush m_WaterFillBrush{ io2d::rgba_color{155, 201, 215} };
    // Fill color for water bodies.

    // --- Railway rendering properties ---
    io2d::brush m_RailwayStrokeBrush{ io2d::rgba_color{93, 93, 93} };
    // Outer stroke for railway tracks.

    io2d::brush m_RailwayDashBrush{ io2d::rgba_color::white };
    // Inner dashed line for railways.

    io2d::dashes m_RailwayDashes{0.f, {3.f, 3.f}};
    // Pattern of dashes used on the railway tracks (3px on, 3px off).

    float m_RailwayOuterWidth = 3.f;
    // Outer stroke width for railway lines.

    float m_RailwayInnerWidth = 2.f;
    // Inner dashed line width for railways.

    // --- Road Representation Structure ---
    struct RoadRep {
        io2d::brush brush{io2d::rgba_color::black};
        // Color brush for the road type.

        io2d::dashes dashes{};
        // Optional dash pattern for styling the road (solid, dashed, etc.).

        float metric_width = 1.f;
        // Width of the road in meters (used to convert to pixels).
    };

    std::unordered_map<Model::Road::Type, RoadRep> m_RoadReps;
    // Stores render styles (color, width, dashes) for different road types.

    std::unordered_map<Model::Landuse::Type, io2d::brush> m_LanduseBrushes;
    // Maps land use types to corresponding brushes (e.g., residential, commercial, forest).
};
