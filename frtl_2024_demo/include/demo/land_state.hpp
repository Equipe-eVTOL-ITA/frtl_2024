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

        Eigen::Vector3d pos = drone->getLocalPosition(),
                        orientation = drone->getOrientation();

        this->initial_x_ = pos[0];
        this->initial_y_ = pos[1];
        this->initial_yaw_ = orientation[0];
        this->landing_height_ = *blackboard.get<float>("takeoff_height") - 0.05;

        drone->log("x: " + std::to_string(pos[0]));
        drone->log("y: " + std::to_string(pos[1]));
        drone->log("z: " + std::to_string(pos[2]));
    }

    std::string act(fsm::Blackboard &blackboard) override {
        

        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return "SEG FAULT";

        Eigen::Vector3d pos  = drone->getLocalPosition(),
                        goal = Eigen::Vector3d({this->initial_x_, this->initial_y_, this->landing_height_});

        if ((pos-goal).norm() < 0.10){
            usleep(2*1e6); //wait 2 seconds
            drone->disarm();
            return "LANDING COMPLETED";
        }

        drone->setLocalPosition(this->initial_x_, this->initial_y_, this->landing_height_, this->initial_yaw_);
        
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;

        drone->toPositionSync();
    }

private:
    float initial_x_, initial_y_, landing_height_, initial_yaw_;
};