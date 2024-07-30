#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

class LandState : public fsm::State {
public:
    LandState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("Entering landing state.");

        Eigen::Vector3d pos = drone->getLocalPosition();
        drone->log("x: " + std::to_string(pos[0]));
        drone->log("y: " + std::to_string(pos[1]));
        drone->log("z: " + std::to_string(pos[2]));
    }

    std::string act(fsm::Blackboard &blackboard) override {
        

        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return "SEG FAULT";

        drone->setLocalVelocity(0.0f, 0.0f, +0.3f, 0.0f);
        drone->log("Arming state is:" + std::to_string(drone->getArmingState()));
        drone->log("Ground speed is: ");

        if (drone->getGroundSpeed() == 0.0f){
            drone->log("Velocity is below 0.1f, landing completed.");
            return "LAND COMPLETED";
        }

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;

        drone->toPositionSync();
    }
};