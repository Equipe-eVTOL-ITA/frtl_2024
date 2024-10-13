#include <vector>
#include "arena_point.hpp"

class ArenaGrid {
public:
    ArenaGrid(float cell_size, float spacing, float arena_size) {
        // Calculate the number of points in each direction
        int num_points = arena_size / cell_size;

        // Create the path vector
        path_.reserve(num_points**2);

        // Start at (1, -1)
        float x = cell_size / 2;
        float y = - cell_size / 2;
        bool move_left = true;
        bool finished = false;

        // Add the first point
        path_.emplace_back(x, y);

        // Move in negative Y direction until reaching the end of the arena
        while (!finished){
            while (y > -arena_size)


        }
        
        while (y > -arena_size) {
            y -= cell_size + spacing;
            path_.emplace_back(x, y);
        }

        // Move in positive X direction
        x += cell_size + spacing;
        path_.emplace_back(x, y);

        // Move in positive Y direction until reaching the end of the arena
        while (y < arena_size - cell_size) {
            y += cell_size + spacing;
            path_.emplace_back(x, y);
        }
    }

    // Other member functions and variables can be added here

private:
    std::vector<ArenaPoint> path_;
    float cell_size_;
    float spacing_;
};