// Including required libraries for data structures, file handling, input/output, rendering and route planning logic
#include <optional>           // For handling file read success/failure
#include <fstream>            // For reading the map file
#include <iostream>           // For terminal I/O
#include <vector>             // For storing map byte data
#include <string>             // For string manipulation
#include <limits>             // For input validation (numeric limits)
#include <io2d.h>             // For drawing the map and path on a GUI window
#include "route_model.h"      // Custom model class for handling OpenStreetMap data
#include "render.h"           // Responsible for rendering the map and path
#include "route_planner.h"    // Core A* algorithm implementation

using namespace std::experimental;  // Required for std::experimental::io2d

// Function to read an OpenStreetMap (.osm) file into a byte array
static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};  // Open file in binary mode, move cursor to end to get file size
    if (!is) 
        return std::nullopt;  // If file can't be opened, return an empty optional
    
    auto size = is.tellg();   // Get the size of the file in bytes
    std::vector<std::byte> contents(size);  // Allocate a vector of that size

    is.seekg(0);              // Move cursor back to beginning of file
    is.read((char*)contents.data(), size);  // Read full file content into the vector

    if (contents.empty()) 
        return std::nullopt;  // If vector is still empty, return failure
    
    return std::move(contents);  // Return the byte vector
}

// Function to get a valid floating point input from the user (range: 0 to 100)
float GetValidatedInput(const std::string &prompt) 
{
    float value;
    while (true) {
        std::cout << prompt;     // Print the prompt
        std::cin >> value;       // Read user input

        if (std::cin.fail()) {   // If user inputs non-float characters
            std::cin.clear();    // Clear the error state
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Ignore invalid input
            std::cout << "Invalid input. Please enter a number between 0 and 100.\n";
        } 
        else if (value < 0.0f || value > 100.0f) {
            std::cout << "Value must be between 0 and 100. Try again.\n";  // Validate range
        } 
        else {
            break;  // Valid input received
        }
    }
    return value;
}

// Entry point of the program
int main(int argc, const char **argv)
{
    std::string osm_data_file = "";  // Variable to hold path to .osm map file

    // Check if user passed a file via command-line arguments
    if (argc > 1) {
        for (int i = 1; i < argc; ++i) {
            if (std::string_view{argv[i]} == "-f" && ++i < argc) {
                osm_data_file = argv[i];  // Get file path after "-f" flag
            }
        }
    } else {
        // No command-line argument: show default usage info
        std::cout << "To specify a map file, use the following format:\n";
        std::cout << "Usage: [executable] [-f filename.osm]\n";
        osm_data_file = "../map.osm";  // Default file path
    }

    std::vector<std::byte> osm_data;  // Vector to store raw map data

    // If the file path is not empty, try reading it
    if (osm_data.empty() && !osm_data_file.empty()) {
        std::cout << "Reading OpenStreetMap data from: " << osm_data_file << "\n";
        auto data = ReadFile(osm_data_file);  // Try reading the file
        if (!data) {
            std::cout << "Failed to read the map file.\n";  // Handle read error
        } else {
            osm_data = std::move(*data);  // Assign file data to our vector
        }
    }

    // Take user inputs: simulated coordinates (scaled from 0 to 100) for robot's path
    float start_x = GetValidatedInput("Enter start_x (0 - 100): ");
    float start_y = GetValidatedInput("Enter start_y (0 - 100): ");
    float end_x   = GetValidatedInput("Enter end_x (0 - 100): ");
    float end_y   = GetValidatedInput("Enter end_y (0 - 100): ");

    // Constructing the route model using parsed OSM data
    // This will internally parse all nodes, ways, and construct a searchable graph
    RouteModel model{osm_data};

    // Create a planner instance, which takes the model and the user's inputs
    // This will initiate the internal state with start and end nodes mapped from given (x, y)
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};

    // Perform A* pathfinding search from start to end using the model
    route_planner.AStarSearch();

    // Display the final distance between start and end coordinates, based on actual node positions
    std::cout << "Distance: " << route_planner.GetDistance() << " meters.\n";

    // Rendering setup to show map and path
    // Render object is initialized with the final model that includes path results
    Render render{model};

    // Create an output window of 400x400 using io2d
    auto display = io2d::output_surface{
        400, 400, io2d::format::argb32, 
        io2d::scaling::none, io2d::refresh_style::fixed, 30  // refresh rate = 30Hz
    };

    // Callback for window resizing: adjust drawing area to new dimensions
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });

    // Callback function that draws the current model (map + path) on the surface
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);  // Internally draws the nodes, roads and path
    });

    display.begin_show();  // Start the GUI event loop (renders and keeps window active)
}
