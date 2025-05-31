# 🚗 A* Search Route Planner in C++

A C++17 implementation of the A* search algorithm that finds and visually renders the shortest path between two user-defined points on a real-world OpenStreetMap (`.osm`) map. Enter coordinates (0–100 scale) and watch the optimal path computed and displayed using the `io2d` graphics library.

---



## 🔍 Features

- 🌍 Loads and parses OpenStreetMap data in `.osm` format  
- 🧠 Implements the A* algorithm from scratch in C++  
- 🧭 Accepts interactive start/end inputs (0–100 normalized scale)  
- 📐 Computes and outputs total path distance  
- 🎨 Renders the path visually using the `io2d` graphics library  
- 🧪 Includes basic unit tests for validation

---

## 🛠 Tech Stack

- C++17  
- A* Heuristic Pathfinding  
- OpenStreetMap (`.osm`) data  
- [`io2d`](https://github.com/cpp-io2d/P0267_RefImpl) for rendering  
- CMake for cross-platform builds

---

## 🚀 Getting Started

### 1. Clone the Repository

Make sure to use `--recurse-submodules` to include required libraries:

```bash
git clone https://github.com/udacity/CppND-Route-Planning-Project.git --recurse-submodules
cd CppND-Route-Planning-Project
```

### 2. Install Dependencies

Ensure the following are installed on your system:

- `cmake` ≥ 3.11.3  
  [Install cmake](https://cmake.org/install/)
- `make` ≥ 4.1 (Linux/Mac), 3.81 (Windows)  
  [Install make (Windows)](http://gnuwin32.sourceforge.net/packages/make.htm)
- `gcc/g++` ≥ 7.4.0  
  [Install via Xcode on macOS](https://developer.apple.com/xcode/features/) or [MinGW on Windows](http://www.mingw.org/)
- `io2d` rendering library  
  Follow [this guide](https://github.com/cpp-io2d/P0267_RefImpl/blob/master/BUILDING.md) to build and install it

> 🔧 Ensure `io2d` is built where `CMake find_package()` can locate it (e.g., system path or CMAKE_PREFIX_PATH).

---

### 3. Obtain a `.osm` Map

Export `.osm` map data from:

- [https://www.openstreetmap.org/export](https://www.openstreetmap.org/export)

Save it in the project root directory as `map.osm`.

---

### 4. Build the Project

```bash
mkdir build && cd build
cmake ..
make
```

---

### 5. Run the Program

```bash
./OSM_A_star_search -f ../map.osm
```

Or simply run the default executable:

```bash
./OSM_A_star_search
```

> The program will prompt you to enter start and end coordinates (between 0 and 100).

---

### 6. Run Unit Tests

```bash
./test
```

---

## 📦 Project Structure

```
.
├── src/
│   ├── main.cpp               # User I/O and program flow
│   ├── render.cpp/.h          # Handles 2D graphics output
│   ├── route_model.cpp/.h     # Parses .osm into graph structure
│   └── route_planner.cpp/.h   # Implements A* search algorithm
├── map.osm                    # User-provided OSM map file
├── test/                      # Unit tests for A*
├── CMakeLists.txt             # Build configuration
└── README.md
```

---

## 📚 Educational Focus

This project is perfect for mastering:

- Graph traversal & A* search algorithms  
- Heuristics in pathfinding  
- Parsing real-world geographic data  
- Modular C++ design & abstraction  
- 2D rendering via external libraries (`io2d`)  
- Cross-platform CMake builds

---

## 🤝 Contributions

Contributions are welcome! Feel free to open issues for bug reports, feature requests, or improvements. Submit a pull request with your proposed changes.

---

## 📜 License

This project is licensed under the MIT License. See the `LICENSE` file for details.

---

## 🙌 Acknowledgments

- [OpenStreetMap](https://www.openstreetmap.org) for the map data  
- Udacity C++ Nanodegree program for project inspiration  
- The `io2d` reference implementation team for 2D rendering support  
- The open-source community for helpful resources and tools 💡

---
