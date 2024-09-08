#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

class TakeoffState : public fsm::State {
public:
    TakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;

        drone->log("STATE: TAKEOFF");
        
        pos = drone->getLocalPosition();
        orientation = drone->getOrientation();
        
        float takeoff_height = *blackboard.get<float>("takeoff height");

        goal = Eigen::Vector3d({pos[0], pos[1], takeoff_height});
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        pos  = drone->getLocalPosition();

        if ((pos-goal).norm() < 0.15)
            return "TAKEOFF COMPLETED";

        drone->setLocalPosition(goal[0], goal[1], goal[2], orientation[2]);
        
        usleep(5e04); // Throttle to approx 20 Hz
        return "";
    }

private:
    Eigen::Vector3d pos, orientation, goal;
    Drone* drone;
};