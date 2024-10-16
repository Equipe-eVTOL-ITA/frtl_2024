#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

#include <opencv2/highgui.hpp>

#include "ArenaPoint.hpp"

class InitialTakeoffState : public fsm::State {
public:
    InitialTakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;

        drone->log("STATE: INITIAL TAKEOFF");

        drone->toOffboardSync();
        drone->armSync();
        
        pos = drone->getLocalPosition();
        orientation = drone->getOrientation();
        
        float target_height = *blackboard.get<float>("takeoff_height");

        drone->log("Home at: " + std::to_string(pos[0])
                    + " " + std::to_string(pos[1]) + " " + std::to_string(pos[2]));

        // ARENA POINTS
        std::vector<ArenaPoint> waypoints;
        Eigen::Vector3d fictual_home = Eigen::Vector3d({2.2, 8.0, -0.6});

        waypoints.push_back({Eigen::Vector3d({2.0, 8.0, target_height})}); 
        waypoints.push_back({Eigen::Vector3d({2.0, 2.0, target_height})});
        waypoints.push_back({Eigen::Vector3d({4.0, 2.0, target_height})});
        waypoints.push_back({Eigen::Vector3d({4.0, 8.0, target_height})});
        waypoints.push_back({Eigen::Vector3d({6.0, 8.0, target_height})});
        waypoints.push_back({Eigen::Vector3d({6.0, 2.0, target_height})});
        waypoints.push_back({Eigen::Vector3d({8.0, 2.0, target_height})});
        waypoints.push_back({Eigen::Vector3d({8.0, 8.0, target_height})});
        //Correcting coordinates based on spawn point
        for (ArenaPoint& waypoint : waypoints){
            waypoint.coordinates = waypoint.coordinates - fictual_home + pos;
        }

        blackboard.set<std::vector<ArenaPoint>>("waypoints", waypoints);

        goal = Eigen::Vector3d({pos[0], pos[1], target_height});
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        pos  = drone->getLocalPosition();

        if ((pos-goal).norm() < 0.20)
            return "INITIAL TAKEOFF COMPLETED";

        drone->setLocalPosition(goal[0], goal[1], goal[2], orientation[2]);
        
        return "";
    }

private:
    Eigen::Vector3d pos, orientation, goal;
    Drone* drone;
};