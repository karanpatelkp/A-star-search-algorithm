#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <limits>    // For input validation
#include <io2d.h>    // For graphics rendering
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

// Reads the contents of a file into a byte vector
static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};  // Open file at end for size checking
    if (!is) 
        return std::nullopt;  // Return empty optional if file can't be opened
    
    auto size = is.tellg();   // Get file size
    std::vector<std::byte> contents(size);

    is.seekg(0);              // Go back to start
    is.read((char*)contents.data(), size);  // Read entire file into contents

    if (contents.empty()) 
        return std::nullopt;  // Return empty optional if nothing was read
    
    return std::move(contents);  // Return the file data
}

// Repeatedly prompts the user until they enter a valid float between 0 and 100
float GetValidatedInput(const std::string &prompt) 
{
    float value;
    while (true) {
        std::cout << prompt;
        std::cin >> value;

        if (std::cin.fail()) {
            // Handle non-numeric input
            std::cin.clear(); // Reset cin error state
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Discard invalid input
            std::cout << "Invalid input. Please enter a number between 0 and 100.\n";
        } 
        else if (value < 0.0f || value > 100.0f) {
            // Handle out-of-range input
            std::cout << "Value must be between 0 and 100. Try again.\n";
        } 
        else {
            break; // Valid input received
        }
    }
    return value;
}

int main(int argc, const char **argv)
{
    std::string osm_data_file = "";

    // Handle command-line arguments
    if (argc > 1) {
        for (int i = 1; i < argc; ++i) {
            if (std::string_view{argv[i]} == "-f" && ++i < argc) {
                osm_data_file = argv[i];  // Get filename after -f flag
            }
        }
    } else {
        // Default instructions if no file specified
        std::cout << "To specify a map file, use the following format:\n";
        std::cout << "Usage: [executable] [-f filename.osm]\n";
        osm_data_file = "../map.osm";  // Default file path
    }
    
    std::vector<std::byte> osm_data;

    // Read OpenStreetMap data
    if (osm_data.empty() && !osm_data_file.empty()) {
        std::cout << "Reading OpenStreetMap data from: " << osm_data_file << "\n";
        auto data = ReadFile(osm_data_file);
        if (!data) {
            std::cout << "Failed to read the map file.\n";
        } else {
            osm_data = std::move(*data);
        }
    }

    // --- Get user input for start and end coordinates ---
    float start_x = GetValidatedInput("Enter start_x (0 - 100): ");
    float start_y = GetValidatedInput("Enter start_y (0 - 100): ");
    float end_x   = GetValidatedInput("Enter end_x (0 - 100): ");
    float end_y   = GetValidatedInput("Enter end_y (0 - 100): ");

    // Build the model with the map data
    RouteModel model{osm_data};

    // Set up the RoutePlanner and perform A* search
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    // Display the distance found
    std::cout << "Distance: " << route_planner.GetDistance() << " meters.\n";

    // Render the results
    Render render{model};
    auto display = io2d::output_surface{
        400, 400, io2d::format::argb32, 
        io2d::scaling::none, io2d::refresh_style::fixed, 30
    };

    // Handle window resizing
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });

    // Draw the map
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });

    display.begin_show();  // Start the display loop
}
