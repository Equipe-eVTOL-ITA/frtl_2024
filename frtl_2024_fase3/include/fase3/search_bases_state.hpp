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
            drone->setLocalPositionSync(last_search[0], last_search[1], last_search[2], orientation[0]);
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
        drone->setLocalPosition(goal[0], goal[1], goal[2], orientation[0]);

        //target_detection = Yolo_Detection
        
        return ""; 
    }
    
private:
    Drone* drone;
    Eigen::Vector3d pos, orientation, goal;
    ArenaPoint* goal_ptr;
    std::vector<ArenaPoint>* waypoints_ptr;
};