#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

#include <opencv2/highgui.hpp>

class InitialTakeoffState : public fsm::State {
public:
    InitialTakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone_ = blackboard.get<Drone>("drone");
        if (drone_ == nullptr) return;

        drone_->log("STATE: INITIAL TAKEOFF");

        drone_->setHomePosition();
        drone_->toOffboardSync();
        drone_->arm();
        
        Eigen::Vector3d pos = drone_->getLocalPosition(),
                        orientation = drone_->getOrientation();

        blackboard.set<Eigen::Vector3d>("home_position", pos);
        
        this->initial_x_ = pos[0];
        this->initial_y_ = pos[1];
        this->initial_yaw_ = orientation[0];
        this->target_height_ = *blackboard.get<float>("takeoff_height") + pos[2];

        drone_->log("Home at: " + std::to_string(pos[0])
                    + " " + std::to_string(pos[2]) + " " + std::to_string(pos[2]));
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        Eigen::Vector3d pos  = drone_->getLocalPosition(),
                        goal = Eigen::Vector3d({this->initial_x_, this->initial_y_, this->target_height_});

        if ((pos-goal).norm() < 0.10)
            return "INITIAL TAKEOFF COMPLETED";

        drone_->setLocalPosition(this->initial_x_, this->initial_y_, this->target_height_, this->initial_yaw_);
        
        return "";
    }

private:
    float initial_x_, initial_y_, target_height_, initial_yaw_;
    Drone* drone_;
};