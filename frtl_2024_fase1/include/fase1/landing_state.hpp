#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

#include "base.hpp"

class LandingState : public fsm::State {
public:
    LandingState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = blackboard.get<Drone>("drone");
        if (drone_ == nullptr) return;
        drone_->log("STATE: LANDING");

        landing_height_ = *blackboard.get<float>("landing_height");

        pos_ = drone_->getLocalPosition();
        Eigen::Vector3d orientation = drone_->getOrientation();
        initial_yaw_ = orientation[2];

        goal_ = Eigen::Vector3d({pos_[0], pos_[1], landing_height_});
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;
        pos_ = drone_->getLocalPosition();

        if ((pos_-goal_).norm() < 0.10){
            return "LANDED";
        }

        drone_->setLocalPosition(goal_[0], goal_[1], goal_[2], this->initial_yaw_);
        
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        //Descend for 3s at 0.4m/s to make sure it is landed
        for (int i = 0 ;i < 30; i++){
            drone_->setLocalVelocity(0.0, 0.0, 0.4);
            usleep(1e5);
        }
        drone_->log("Landed at height: " + std::to_string(pos_[2]));
    }

private:
    Drone* drone_;
    float landing_height_, initial_yaw_;
    Eigen::Vector3d pos_, goal_;
};