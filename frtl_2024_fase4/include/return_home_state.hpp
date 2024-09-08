#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

class ReturnHomeState : public fsm::State {
public:
    ReturnHomeState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: RETURN HOME");

        home_pos = *blackboard.get<Eigen::Vector3d>("home position");
        takeoff_height = *blackboard.get<float>("takeoff height");
        
        orientation = drone->getOrientation();

        goal = Eigen::Vector3d({home_pos[0], home_pos[1], takeoff_height});        
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        drone->log("At home, now entered Land Mode.");
        drone->land();
        drone->disarmSync();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        pos = drone->getLocalPosition();
        
        if ((pos-goal).norm() < 0.10)
            return "AT HOME";

        drone->setLocalPosition(goal[0], goal[1], goal[2], orientation[2]);
        
        return "";
    }

private:
    Eigen::Vector3d home_pos, pos, orientation, goal;
    float takeoff_height;
    Drone* drone;
};