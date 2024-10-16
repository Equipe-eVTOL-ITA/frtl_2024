#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>
#include "Base.hpp"

class VisitBasesState : public fsm::State {
public:
    VisitBasesState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone_ = blackboard.get<Drone>("drone");
        if (drone_ == nullptr) return;
        drone_->log("STATE: VISIT BASES");
    
        Eigen::Vector3d pos = drone_->getLocalPosition(),
                        orientation = drone_->getOrientation();

        bases = blackboard.get<std::vector<Base>>("bases");
        for (Base& base : *bases){
            if (!base.is_visited){
                this->base_to_visit_ = &base;
                break;
            }
        }

        this->initial_z_ = pos[2];
        this->initial_yaw_ = orientation[2];
        this->target_x_ = this->base_to_visit_->coordinates[0];
        this->target_y_ = this->base_to_visit_->coordinates[1];

        drone_->log("Visiting base at: " + std::to_string(this->target_x_) + " " + std::to_string(this->target_y_)); 
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        Eigen::Vector3d pos  = drone_->getLocalPosition(),
                        goal = Eigen::Vector3d({this->target_x_, this->target_y_, this->initial_z_});

        if ((pos-goal).norm() < 0.10)
            return "ARRIVED AT BASE";

        drone_->setLocalPosition(goal[0], goal[1], goal[2], this->initial_yaw_);
        
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {

        blackboard.set<float>("landing_height", this->base_to_visit_->coordinates[2]);
        this->base_to_visit_->is_visited = true;

        bool finished_bases = true;
        for (Base& base : *bases){
            if (!base.is_visited){
                finished_bases = false;
                break;
            }
        }
        blackboard.set<bool>("finished_bases", finished_bases);
    }

private:
    std::vector<Base>* bases;
    Drone* drone_;
    Base* base_to_visit_;
    float initial_z_, initial_yaw_, target_x_, target_y_;
};