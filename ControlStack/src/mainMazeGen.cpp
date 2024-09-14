#include <iostream>
#include <fstream>
#include "../inc/maze.hpp"


// g++ -o maze_generator main.cpp -std=c++11


int main() {
    // Parameters for the maze
    int rows = 20;                // Number of rows
    int cols = 20;                // Number of columns
    double cellWidth = 0.2;       // Width of each cell in meters
    double cellHeight = 0.2;      // Height of each cell in meters
    double wallThickness = 0.05;   // Thickness of walls in meters
    double density = 0.5;         // Proportion of walls that are active

    // Create the maze
    Maze maze(rows, cols, cellWidth, cellHeight, wallThickness);

    // Compute the wall vertices
    auto vertices = maze.computeWallVertices();

    // Output the vertices to a CSV file
    std::ofstream outFile("maze_vertices.csv");
    if (!outFile.is_open()) {
        std::cerr << "Error: Could not open the file for writing." << std::endl;
        return 1;
    }

    // Write the header
    outFile << "x1,y1,x2,y2,x3,y3,x4,y4,\n";

    // Write each wall's vertices
    for (const auto& vertex : vertices) {
        outFile << vertex(0, 0) << "," << vertex(0, 1) << "," << vertex(1, 0) << "," << vertex(1, 1) << ',' << vertex(2, 0) << "," << vertex(2, 1) << "," << vertex(3, 0) << "," << vertex(3, 1) << "\n";
    }

    outFile.close();

    std::cout << "Maze vertices have been written to maze_vertices.csv" << std::endl;
    return 0;
}
