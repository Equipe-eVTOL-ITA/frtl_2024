#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

#include <opencv2/highgui.hpp>

class TakeoffState : public fsm::State {
public:
    TakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone_ = blackboard.get<Drone>("drone");
        if (drone_ == nullptr) return;

        drone_->log("STATE: TAKEOFF");

        std::vector<Base> bases_obj = *blackboard.get<std::vector<Base>>("bases");
        drone_->log("Finished bases: " + std::to_string(bases_obj[0].is_visited) + " " + std::to_string(bases_obj[1].is_visited) + " "
                    + std::to_string(bases_obj[2].is_visited) + " "+ std::to_string(bases_obj[3].is_visited) + " "
                    + std::to_string(bases_obj[4].is_visited));

        finished_bases_ = *blackboard.get<bool>("finished_bases");
        drone_->log("Are bases finished? " + std::to_string(finished_bases_));

        float takeoff_height = *blackboard.get<float>("takeoff_height");
        
        pos_ = drone_->getLocalPosition();
        orientation_ = drone_->getOrientation();
        goal_ = Eigen::Vector3d({pos_[0], pos_[1], takeoff_height});
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        pos_  = drone_->getLocalPosition();

        if ((pos_-goal_).norm() < 0.10){
            if (finished_bases_)
                return "FINISHED BASES";
            else
                return "NEXT BASE";
        }

        drone_->setLocalPosition(goal_[0], goal_[1], goal_[2], orientation_[0]);
        
        return "";
    }

private:
    bool finished_bases_;
    Drone* drone_;
    Eigen::Vector3d pos_, goal_, orientation_;
};