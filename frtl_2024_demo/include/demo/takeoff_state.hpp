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
        
        drone->setHomePosition();
        drone->toOffboardSync();
        drone->arm();
        
        Eigen::Vector3d pos = drone->getLocalPosition(),
                        orientation = drone->getOrientation();

        this->initial_x_ = pos[0];
        this->initial_y_ = pos[1];
        this->initial_yaw_ = orientation[0];
        this->target_height_ = *blackboard.get<float>("height");
        blackboard.set<float>("takeoff_height", pos[2]);

        drone->log("Arming state is:" + std::to_string(drone->getArmingState()));
        drone->log("x: " + std::to_string(pos[0]));
        drone->log("y: " + std::to_string(pos[1]));
        drone->log("z: " + std::to_string(pos[2]));
    }

    std::string act(fsm::Blackboard &blackboard) override {

        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return "SEG FAULT";

        Eigen::Vector3d pos  = drone->getLocalPosition(),
                        goal = Eigen::Vector3d({this->initial_x_, this->initial_y_, this->target_height_});

        if ((pos-goal).norm() < 0.10)
            return "TAKEOFF COMPLETED";

        drone->setLocalPosition(this->initial_x_, this->initial_y_, this->target_height_, this->initial_yaw_);
        
        return "";
    }

private:
    float initial_x_, initial_y_, target_height_, initial_yaw_;
};