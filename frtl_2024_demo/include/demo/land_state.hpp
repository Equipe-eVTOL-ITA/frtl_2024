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
    }

    std::string act(fsm::Blackboard &blackboard) override {
        

        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return "SEG FAULT";

        drone->land();
        drone->log("Landing");
        drone->log("Arming state is:");

        if (drone->getArmingState() == DronePX4::ARMING_STATE::DISARMED){
            drone->log("The landing has completed.");
            return "LAND COMPLETED";
        }

        return "";
    }
};