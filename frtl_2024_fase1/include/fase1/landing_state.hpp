#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "Base.hpp"

class LandingState : public fsm::State {
public:
    LandingState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: LANDING");

        pos = drone->getLocalPosition();

        for (int i = 0; i < 10; i++){
            drone->land();
            sleep(0.1);
        }
        sleep(2);
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

        //Descend for 1s at 0.4m/s to make sure it is landed
        for (int i = 0 ;i < 10; i++){
            drone->setLocalVelocity(0.0, 0.0, 0.4, 0.0);
            usleep(1e5);
        }

        //Publish base coordinates
        pos = drone->getLocalPosition();
        std::vector<Base> bases = *blackboard.get<std::vector<Base>>("bases");
        bases.push_back({pos, true});
        blackboard.set<std::vector<Base>>("bases", bases);

        drone->toOffboardSync();
        drone->armSync();    
    }

private:
    float airspeed;
    Drone* drone;
    Eigen::Vector3d pos;
};