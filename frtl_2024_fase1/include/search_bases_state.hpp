#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

#include <iostream>
#include <vector>
#include "base.hpp"

class SearchBasesState : public fsm::State {
public:
    SearchBasesState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = blackboard.get<Drone>("drone");
        if (drone_ == nullptr) return;

        drone_->log("STATE: SEARCH BASES");

        Eigen::Vector3d real_home = *blackboard.get<Eigen::Vector3d>("home_position"),
                        fictual_home = Eigen::Vector3d({2.2, 8.0, -0.242});

        std::vector<Base> bases;

        // Takeoff platform: (2.2, 8.0, -0.6)
        bases.push_back({Eigen::Vector3d({8.0, 8.0, -1.0})}); //suspended landing platform 1
        bases.push_back({Eigen::Vector3d({2.0, 5.0, -1.505})}); //suspended landing platform 2
        bases.push_back({Eigen::Vector3d({5.0, 4.0, -0.005})}); //landing platform 1
        bases.push_back({Eigen::Vector3d({7.0, 6.0, -0.005})}); //landing platform 2
        bases.push_back({Eigen::Vector3d({3.0, 2.0, -0.005})}); //landing platform 3

        //Correcting coordinates based on spawn point
        for (Base& base : bases){
            base.coordinates = base.coordinates - fictual_home + real_home;
        }

        blackboard.set<std::vector<Base>>("bases", bases);

        //Circle variables
        this->x_center_ = 5.0;
        this->y_center_ = 5.0;
        this->x_center_ += - fictual_home[0] + real_home[0];
        this->y_center_ += - fictual_home[1] + real_home[1];
        this->r_ = 2.0;
        this->w_ = 0.4;
        this->timeout_ = 2 * acos(-1) / this->w_;
        this->start_time_ = this->drone_->getTime();
        
        this->orient_ = drone_->getOrientation();
        this->pos_ = drone_->getLocalPosition();

        drone_->log("Moving to initial search location");
        drone_->setLocalPositionSync(this->x_center_ + this->r_, this->y_center_, this->pos_[2], this->orient_[0]);
        drone_->log("Starting search movement");
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        drone_->log("Going back to initial location");
        drone_->setLocalPositionSync(this->pos_[0], this->pos_[1], this->pos_[2], this->orient_[0]);
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        double dt = this->drone_->getTime() - this->start_time_;
        if (dt > this->timeout_)
            return "BASES FOUND";

        float x = r_ * cos(w_ * dt) + x_center_;
        float y = r_ * sin(w_ * dt) + y_center_;

        this->drone_->setLocalPosition(x, y, this->pos_[2], this->orient_[0]);
        
        return "";
    }
    
private:
    float r_, w_, timeout_, start_time_, x_center_, y_center_;
    Drone* drone_;
    Eigen::Vector3d pos_, orient_;
};