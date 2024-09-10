#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

class InitialTakeoffState : public fsm::State {
public:
    InitialTakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;

        drone->log("STATE: INITIAL TAKEOFF");

        drone->setHomePosition();
        drone->toOffboardSync();
        drone->armSync();
        
        pos = drone->getLocalPosition();
        orientation = drone->getOrientation();

        blackboard.set<Eigen::Vector3d>("home position", pos);
        
        float* takeoff_height = blackboard.get<float>("takeoff height");
        *takeoff_height += pos[2];

        drone->log("Home at: " + std::to_string(pos[0])
                    + " " + std::to_string(pos[1]) + " " + std::to_string(pos[2]));

        goal = Eigen::Vector3d({pos[0], pos[1], *takeoff_height});
        drone->log("Goal is height: " + std::to_string(goal[2]));
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        pos  = drone->getLocalPosition();

        if ((pos-goal).norm() < 0.10)
            return "INITIAL TAKEOFF COMPLETED";

        drone->setLocalPosition(goal[0], goal[1], goal[2], orientation[2]);

        usleep(5e04); // Throttle to approx 20 Hz
        
        return "";
    }

private:
    Eigen::Vector3d pos, orientation, goal;
    Drone* drone;
};