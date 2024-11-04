#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "Base.hpp"
#include "GetNextPoint.hpp"
#include "CoordinateTransforms.hpp"

class SearchBasesState : public fsm::State {
public:
    SearchBasesState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: SEARCH BASES");

        orientation = drone->getOrientation();

        coord_transforms = blackboard.get<CoordinateTransforms>("coordinate_transforms");
        waypoints = blackboard.get<std::vector<ArenaPoint>>("waypoints");
        bases = blackboard.get<std::vector<Base>>("bases");
        Eigen::Vector3d* last_search_pos = blackboard.get<Eigen::Vector3d>("last search position");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        pos = drone->getLocalPosition();
        goal_ptr = getNextPoint(waypoints);

        if (goal_ptr == nullptr)
            return "SEARCH ENDED";

        if ((pos - goal_ptr->coordinates).norm() < 0.15){
            goal_ptr->is_visited = true;
            drone->log("Waypoint visited.");
        }

        goal = goal_ptr->coordinates;
        goal_diff = goal-pos;
        if (goal_diff.norm() > max_velocity){
            goal_diff = goal_diff.normalized() * max_velocity;
        }
        goal = goal_diff + pos;
        drone->setLocalPosition(goal[0], goal[1], goal[2], orientation[2]);

        bboxes = drone->getBoundingBox();

        if (!bboxes.empty() && !previous_bboxes.empty()) {
            if (bboxes[0].center_x != previous_bboxes[0].center_x){
                for (const auto& bbox : bboxes) {
                    //drone->log("BBOX: {" + std::to_string(bbox.center_x) + ", " 
                    //            + std::to_string(bbox.center_y) + ", " 
                    //            + std::to_string(bbox.size_x) + ", " 
                    //            + std::to_string(bbox.size_y) + "}");
                    
                    detected_pad_world_coords = coord_transforms->ImageToWorld(
                                                pos, orientation, bbox.center_x*640, bbox.center_y*480);

                    drone->log("WORLD: {" + std::to_string(detected_pad_world_coords[0]) + ", " 
                                + std::to_string(detected_pad_world_coords[1]) + ", " 
                                + std::to_string(detected_pad_world_coords[2]) + "}");

                    bool is_known_base = false;
                    for (const auto& base : *bases) {
                        double horizontal_distance = (detected_pad_world_coords.head<2>() - base.coordinates.head<2>()).norm();
                        drone->log("Horizontal Distance: " + std::to_string(horizontal_distance) + " to base: { " 
                                    + std::to_string(base.coordinates[0]) + " " + std::to_string(base.coordinates[1]) + "}");
                        if (horizontal_distance < 1.6) {
                            drone->log("This base is known");
                            is_known_base = true;
                            break;
                        }
                    }

                    if (!is_known_base) {
                        blackboard.set<Eigen::Vector3d>("base_to_visit", detected_pad_world_coords);
                        drone->log("This is a new base. Visiting!");
                        return "BASES FOUND";
                    }
                }
            }

        }
        previous_bboxes = bboxes;

        usleep(5e04);  // Control frequency to approximately 20Hz

        return ""; 
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        
        pos = drone->getLocalPosition();
        blackboard.set<Eigen::Vector3d>("last search position", pos);
    }
    
private:
    std::vector<Base>* bases;
    CoordinateTransforms* coord_transforms;
    std::vector<DronePX4::BoundingBox> bboxes, previous_bboxes;
    const float max_velocity = 0.5;
    Drone* drone;
    Eigen::Vector3d pos, orientation, goal, goal_diff, detected_pad_world_coords;
    ArenaPoint* goal_ptr;
    std::vector<ArenaPoint>* waypoints;
};