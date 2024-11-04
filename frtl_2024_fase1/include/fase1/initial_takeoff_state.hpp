#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "ArenaPoint.hpp"

class InitialTakeoffState : public fsm::State {
public:
    InitialTakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: INITIAL TAKEOFF");

        drone->toOffboardSync();
        drone->armSync();
        
        pos = drone->getLocalPosition();
        orientation = drone->getOrientation();
        float target_height = *blackboard.get<float>("takeoff_height");

        drone->log("Home at: " + std::to_string(pos[0])
                    + " " + std::to_string(pos[1]) + " " + std::to_string(pos[2]));

        goal = Eigen::Vector3d({pos[0], pos[1], target_height});
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        pos  = drone->getLocalPosition();

        if ((pos-goal).norm() < 0.15){
            return "INITIAL TAKEOFF COMPLETED";
        }

        goal_diff = goal-pos;
        if (goal_diff.norm() > max_velocity){
            goal_diff = goal_diff.normalized() * max_velocity;
        }
        goal = goal_diff + pos;
        drone->setLocalPosition(goal[0], goal[1], goal[2], orientation[2]);
        
        usleep(5e04);  // Control frequency to approximately 20Hz
        return "";
    }

private:
    const float max_velocity = 1.0;
    Eigen::Vector3d pos, orientation, goal, goal_diff;
    Drone* drone;
};