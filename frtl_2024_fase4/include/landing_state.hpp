#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

class LandingState : public fsm::State {
public:
    LandingState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: LANDING");

        pos = drone->getLocalPosition();

        drone->land();
        sleep(3);
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;
        airspeed = drone->getAirSpeed();

        if (airspeed < 0.05){
            return "LANDED";
        }

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        //Descend for 3s at 0.4m/s to make sure it is landed
        for (int i = 0 ;i < 30; i++){
            drone->setLocalVelocity(0.0, 0.0, 0.4, 0.0);
            usleep(1e5);
        }

        drone->toOffboardSync();
        drone->armSync();    
    }

private:
    float airspeed;
    Drone* drone;
    Eigen::Vector3d pos, goal;
};