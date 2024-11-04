#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class ReturnHomeState : public fsm::State {
public:
    ReturnHomeState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = blackboard.get<Drone>("drone");
        if (drone_ == nullptr) return;
        drone_->log("STATE: RETURN HOME");

        home_pos_ = *blackboard.get<Eigen::Vector3d>("home_position");
        pos_ = drone_->getLocalPosition();
        orientation_ = drone_->getOrientation();

        goal_ = Eigen::Vector3d({home_pos_[0], home_pos_[1], pos_[2] - 0.1});
        
        drone_->log("Going to home at: " + std::to_string(goal_[0]) + " " + std::to_string(goal_[1]));
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        goal_[2] = home_pos_[2];
        drone_->setLocalPositionSync(goal_[0], goal_[1], goal_[2], orientation_[2]);
        
        drone_->log("At home, now entered Land Mode for precaution.");
        drone_->land();
        drone_->disarmSync();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        pos_ = drone_->getLocalPosition();
        
        if ((pos_-goal_).norm() < 0.10)
            return "AT HOME";

        drone_->setLocalPosition(goal_[0], goal_[1], goal_[2], orientation_[2]);
        
        return "";
    }

private:
    Eigen::Vector3d home_pos_, pos_, orientation_, goal_;
    Drone* drone_;
};