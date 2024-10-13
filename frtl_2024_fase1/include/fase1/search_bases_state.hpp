#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

#include <iostream>
#include <vector>
#include "base.hpp"
#include "get_next_point.hpp"

class SearchBasesState : public fsm::State {
public:
    SearchBasesState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;

        drone->log("STATE: SEARCH BASES");

        orientation = drone->getOrientation();

        waypoints_ptr = blackboard.get<std::vector<ArenaPoint>>("waypoints");

        Eigen::Vector3d* last_search_ptr = blackboard.get<Eigen::Vector3d>("last search position");
        if (last_search_ptr != nullptr){
            drone->log("Going to last search position.");
            Eigen::Vector3d& last_search = *last_search_ptr;
            drone->setLocalPositionSync(last_search[0], last_search[1], last_search[2], orientation[2]);
        }
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        pos = drone->getLocalPosition();
        
        blackboard.set<Eigen::Vector3d>("last search position", pos);
        
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        pos = drone->getLocalPosition();
        goal_ptr = getNextPoint(waypoints_ptr);

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
                    drone->log("BBOX: {" + std::to_string(bbox.center_x) + ", " 
                                + std::to_string(bbox.center_y) + ", " 
                                + std::to_string(bbox.size_x) + ", " 
                                + std::to_string(bbox.size_y) + "}");
                }
            }

        }
        previous_bboxes = bboxes;

        return ""; 
    }
    
private:
    std::vector<DronePX4::BoundingBox> bboxes, previous_bboxes;
    const float max_velocity = 0.8;
    Drone* drone;
    Eigen::Vector3d pos, orientation, goal, goal_diff;
    ArenaPoint* goal_ptr;
    std::vector<ArenaPoint>* waypoints_ptr;
};