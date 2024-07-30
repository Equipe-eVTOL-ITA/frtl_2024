#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

#include <opencv2/highgui.hpp>

class TakeoffState : public fsm::State {
public:
    TakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("Taking off");

        drone->toOffboardSync();
        drone->arm();
        
        Eigen::Vector3d pos = drone->getLocalPosition(),
                        orientation = drone->getOrientation();

        this->initial_x = pos[0];
        this->initial_y = pos[1];
        this->initial_yaw = orientation[0];
        drone->log("Arming state is:" + std::to_string(drone->getArmingState()));
        drone->log("x: " + std::to_string(pos[0]));
        drone->log("y: " + std::to_string(pos[1]));
        drone->log("z: " + std::to_string(pos[2]));
    }

    std::string act(fsm::Blackboard &blackboard) override {

        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return "SEG FAULT";

        float* z =  blackboard.get<float>("height");
        if (z == nullptr) return "SEG FAULT";

        Eigen::Vector3d pos  = drone->getLocalPosition(),
                        goal = Eigen::Vector3d({this->initial_x, this->initial_y, *z});

        if ((pos-goal).norm() < 0.15)
            return "TAKEOFF COMPLETED";

        drone->setLocalPosition(this->initial_x, this->initial_y, *z, this->initial_yaw);
        
        return "";
    }

    float initial_x, initial_y, initial_yaw;
};