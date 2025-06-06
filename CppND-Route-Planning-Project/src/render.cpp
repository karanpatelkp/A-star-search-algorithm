// Include the custom render header
#include "render.h"

// Include standard I/O for debugging/logging (if needed)
#include <iostream>

// Forward declarations for helper functions
static float RoadMetricWidth(Model::Road::Type type);                // Determines the visual width of road types
static io2d::rgba_color RoadColor(Model::Road::Type type);           // Determines the color of each road type
static io2d::dashes RoadDashes(Model::Road::Type type);              // Determines line dashes (e.g., for footpaths)
static io2d::point_2d ToPoint2D(const Model::Node &node) noexcept;   // Converts a node to a drawable 2D point

// Constructor: stores reference to the model and sets up brushes
Render::Render(RouteModel &model) : m_Model(model) {
    BuildRoadReps();           // Prepare road rendering styles
    BuildLanduseBrushes();     // Prepare land-use rendering brushes
}

// Main rendering function, called each frame or redraw
void Render::Display(io2d::output_surface &surface) {
    // Calculate the drawing scale based on output surface dimensions
    m_Scale = static_cast<float>(std::min(surface.dimensions().x(), surface.dimensions().y()));    
    m_PixelsInMeter = static_cast<float>(m_Scale / m_Model.MetricScale()); 

    // Transformation matrix for flipping Y-axis and scaling
    m_Matrix = io2d::matrix_2d::create_scale({m_Scale, -m_Scale}) *
               io2d::matrix_2d::create_translate({0.f, static_cast<float>(surface.dimensions().y())});

    surface.paint(m_BackgroundFillBrush); // Paint background

    // Draw map features in layering order
    DrawLanduses(surface);
    DrawLeisure(surface);
    DrawWater(surface);    
    DrawRailways(surface);
    DrawHighways(surface);    
    DrawBuildings(surface);  
    DrawPath(surface);
    DrawStartPosition(surface);   
    DrawEndPosition(surface);
}

// Draw the full path as an orange stroke
void Render::DrawPath(io2d::output_surface &surface) const {
    io2d::render_props aliased{io2d::antialias::none};
    io2d::brush foreBrush{io2d::rgba_color::orange}; 
    float width = 5.0f;  // Thickness of path line
    surface.stroke(foreBrush, PathLine(), std::nullopt, io2d::stroke_props{width});
}

// Draw red square marker at end of path
void Render::DrawEndPosition(io2d::output_surface &surface) const {
    if (m_Model.path.empty()) return;
    io2d::render_props aliased{io2d::antialias::none};
    io2d::brush foreBrush{io2d::rgba_color::red};

    auto pb = io2d::path_builder{};
    pb.matrix(m_Matrix);

    pb.new_figure({(float)m_Model.path.back().x, (float)m_Model.path.back().y});
    float constexpr l_marker = 0.01f;
    pb.rel_line({l_marker, 0.f});
    pb.rel_line({0.f, l_marker});
    pb.rel_line({-l_marker, 0.f});
    pb.rel_line({0.f, -l_marker});
    pb.close_figure();

    surface.fill(foreBrush, pb);
    surface.stroke(foreBrush, io2d::interpreted_path{pb}, std::nullopt, std::nullopt, std::nullopt, aliased);
}

// Draw green square marker at start of path
void Render::DrawStartPosition(io2d::output_surface &surface) const {
    if (m_Model.path.empty()) return;

    io2d::render_props aliased{io2d::antialias::none};
    io2d::brush foreBrush{io2d::rgba_color::green};

    auto pb = io2d::path_builder{};
    pb.matrix(m_Matrix);

    pb.new_figure({(float)m_Model.path.front().x, (float)m_Model.path.front().y});
    float constexpr l_marker = 0.01f;
    pb.rel_line({l_marker, 0.f});
    pb.rel_line({0.f, l_marker});
    pb.rel_line({-l_marker, 0.f});
    pb.rel_line({0.f, -l_marker});
    pb.close_figure();

    surface.fill(foreBrush, pb);
    surface.stroke(foreBrush, io2d::interpreted_path{pb}, std::nullopt, std::nullopt, std::nullopt, aliased);
}

// Draw all building polygons with fill and stroke
void Render::DrawBuildings(io2d::output_surface &surface) const {
    for (auto &building : m_Model.Buildings()) {
        auto path = PathFromMP(building);
        surface.fill(m_BuildingFillBrush, path);
        surface.stroke(m_BuildingOutlineBrush, path, std::nullopt, m_BuildingOutlineStrokeProps);
    }
}

// Draw leisure areas (parks, playfields) with outline and fill
void Render::DrawLeisure(io2d::output_surface &surface) const {
    for (auto &leisure : m_Model.Leisures()) {
        auto path = PathFromMP(leisure);
        surface.fill(m_LeisureFillBrush, path);
        surface.stroke(m_LeisureOutlineBrush, path, std::nullopt, m_LeisureOutlineStrokeProps);
    }
}

// Draw all water bodies (lakes, rivers)
void Render::DrawWater(io2d::output_surface &surface) const {
    for (auto &water : m_Model.Waters())
        surface.fill(m_WaterFillBrush, PathFromMP(water));
}

// Fill land-use polygons (e.g., residential, commercial) with proper brush
void Render::DrawLanduses(io2d::output_surface &surface) const {
    for (auto &landuse : m_Model.Landuses())
        if (auto br = m_LanduseBrushes.find(landuse.type); br != m_LanduseBrushes.end())
            surface.fill(br->second, PathFromMP(landuse));
}

// Draw roads based on type, width, color, and dashes
void Render::DrawHighways(io2d::output_surface &surface) const {
    auto ways = m_Model.Ways().data();
    for (auto road : m_Model.Roads())
        if (auto rep_it = m_RoadReps.find(road.type); rep_it != m_RoadReps.end()) {
            auto &rep = rep_it->second;
            auto &way = ways[road.way];
            auto width = rep.metric_width > 0.f ? (rep.metric_width * m_PixelsInMeter) : 1.f;
            auto sp = io2d::stroke_props{width, io2d::line_cap::round};
            surface.stroke(rep.brush, PathFromWay(way), std::nullopt, sp, rep.dashes);
        }
}

// Draw railways using outer and dashed inner strokes
void Render::DrawRailways(io2d::output_surface &surface) const {
    auto ways = m_Model.Ways().data();
    for (auto &railway : m_Model.Railways()) {
        auto &way = ways[railway.way];
        auto path = PathFromWay(way);
        surface.stroke(m_RailwayStrokeBrush, path, std::nullopt, io2d::stroke_props{m_RailwayOuterWidth * m_PixelsInMeter});
        surface.stroke(m_RailwayDashBrush, path, std::nullopt, io2d::stroke_props{m_RailwayInnerWidth * m_PixelsInMeter}, m_RailwayDashes);
    }
}

// Builds interpreted path object for the main path line
io2d::interpreted_path Render::PathLine() const {
    if (m_Model.path.empty())
        return {};

    const auto nodes = m_Model.path;
    auto pb = io2d::path_builder{};
    pb.matrix(m_Matrix);
    pb.new_figure(ToPoint2D(nodes[0]));

    for (int i = 1; i < nodes.size(); i++)
        pb.line(ToPoint2D(nodes[i]));

    return io2d::interpreted_path{pb};
}

// Converts a way (line of nodes) to a drawable path
io2d::interpreted_path Render::PathFromWay(const Model::Way &way) const {
    if (way.nodes.empty())
        return {};

    const auto nodes = m_Model.Nodes().data();
    auto pb = io2d::path_builder{};
    pb.matrix(m_Matrix);
    pb.new_figure(ToPoint2D(nodes[way.nodes.front()]));

    for (auto it = ++way.nodes.begin(); it != std::end(way.nodes); ++it)
        pb.line(ToPoint2D(nodes[*it]));

    return io2d::interpreted_path{pb};
}

// Converts a multipolygon (e.g., building) to a drawable path
io2d::interpreted_path Render::PathFromMP(const Model::Multipolygon &mp) const {
    const auto nodes = m_Model.Nodes().data();
    const auto ways = m_Model.Ways().data();

    auto pb = io2d::path_builder{};
    pb.matrix(m_Matrix);

    auto commit = [&](const Model::Way &way) {
        if (way.nodes.empty())
            return;
        pb.new_figure(ToPoint2D(nodes[way.nodes.front()]));
        for (auto it = ++way.nodes.begin(); it != std::end(way.nodes); ++it)
            pb.line(ToPoint2D(nodes[*it]));
        pb.close_figure();
    };

    for (auto way_num : mp.outer)
        commit(ways[way_num]);
    for (auto way_num : mp.inner)
        commit(ways[way_num]);

    return io2d::interpreted_path{pb};
}

// Prepares rendering styles for all road types
void Render::BuildRoadReps() {
    using R = Model::Road;
    auto types = {R::Motorway, R::Trunk, R::Primary, R::Secondary, R::Tertiary,
                  R::Residential, R::Service, R::Unclassified, R::Footway};
    for (auto type : types) {
        auto &rep = m_RoadReps[type];
        rep.brush = io2d::brush{RoadColor(type)};
        rep.metric_width = RoadMetricWidth(type);
        rep.dashes = RoadDashes(type);
    }
}

// Prepares brushes for land-use regions based on type
void Render::BuildLanduseBrushes() {
    m_LanduseBrushes.insert_or_assign(Model::Landuse::Commercial, io2d::brush{io2d::rgba_color{233, 195, 196}});
    m_LanduseBrushes.insert_or_assign(Model::Landuse::Construction, io2d::brush{io2d::rgba_color{187, 188, 165}});
    m_LanduseBrushes.insert_or_assign(Model::Landuse::Grass, io2d::brush{io2d::rgba_color{197, 236, 148}});
    m_LanduseBrushes.insert_or_assign(Model::Landuse::Forest, io2d::brush{io2d::rgba_color{158, 201, 141}});
    m_LanduseBrushes.insert_or_assign(Model::Landuse::Industrial, io2d::brush{io2d::rgba_color{223, 197, 220}});
    m_LanduseBrushes.insert_or_assign(Model::Landuse::Railway, io2d::brush{io2d::rgba_color{223, 197, 220}});
    m_LanduseBrushes.insert_or_assign(Model::Landuse::Residential, io2d::brush{io2d::rgba_color{209, 209, 209}});
}

// Returns standard width for each road type
static float RoadMetricWidth(Model::Road::Type type) {
    switch (type) {
        case Model::Road::Motorway: return 6.f;
        case Model::Road::Trunk: return 6.f;
        case Model::Road::Primary: return 5.f;
        case Model::Road::Secondary: return 5.f;
        case Model::Road::Tertiary: return 4.f;
        case Model::Road::Residential: return 2.5f;
        case Model::Road::Unclassified: return 2.5f;
        case Model::Road::Service: return 1.f;
        case Model::Road::Footway: return 0.f;
        default: return 1.f;
    }
}

// Returns standard color for each road type
static io2d::rgba_color RoadColor(Model::Road::Type type) {
    switch (type) {
        case Model::Road::Motorway: return io2d::rgba_color{226, 122, 143};
        case Model::Road::Trunk: return io2d::rgba_color{245, 161, 136};
        case Model::Road::Primary: return io2d::rgba_color{249, 207, 144};
        case Model::Road::Secondary: return io2d::rgba_color{244, 251, 173};
        case Model::Road::Tertiary: return io2d::rgba_color{244, 251, 173};
        case Model::Road::Residential: return io2d::rgba_color{254, 254, 254};
        case Model::Road::Service: return io2d::rgba_color{254, 254, 254};
        case Model::Road::Footway: return io2d::rgba_color{241, 106, 96};
        case Model::Road::Unclassified: return io2d::rgba_color{254, 254, 254};
        default: return io2d::rgba_color::grey;
    }
}

// Returns dashed pattern for footpaths only
static io2d::dashes RoadDashes(Model::Road::Type type) {
    return type == Model::Road::Footway ? io2d::dashes{0.f, {1.f, 2.f}} : io2d::dashes{};
}

// Helper: convert a Node struct to a 2D point for drawing
static io2d::point_2d ToPoint2D(const Model::Node &node) noexcept {
    return io2d::point_2d(static_cast<float>(node.x), static_cast<float>(node.y));
}
