#pragma once

ArenaPoint* getNextPoint(std::vector<ArenaPoint>* waypoints) {
    for (auto& point : *waypoints) {
        if (!point.is_visited) {
            return &point;
        }
    }
    
    // Return nullptr if no unvisited points are found
    return nullptr;
}