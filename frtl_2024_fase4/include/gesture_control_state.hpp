#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

#include <iostream>
#include <string>
#include <array>
#include "pid_controller.hpp"

class GestureControlState : public fsm::State {
public:
    GestureControlState() : fsm::State(), pid(1.0, 0.1, 0.05, 0.5) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;

        drone->log("STATE: GESTURE CONTROL");

        this->control_speed = *blackboard.get<float>("control speed");
        drone->log("control speed: " + std::to_string(this->control_speed));
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        gestures = drone->getHandGestures();
        hand_x = (drone->getHandLocation())[0];

        if (hand_x != previous_hand_x){
            this->yaw_rate = pid.compute(hand_x);
            if (i%10==0) drone->log("yaw rate: " + std::to_string(this->yaw_rate));

            if (gestures.size() > 1){
                if (gestures[1] == "Open_Palm"){
                    return "LAND NOW";
                }
                handleGesture(gestures[1]);
            }
            else{
                drone->setLocalVelocity(0.0, 0.0, 0.0, this->yaw_rate);
            }
        }
        else{
            drone->setOffboardControlMode(DronePX4::CONTROLLER_TYPE::VELOCITY);
        }

        previous_hand_x = hand_x;
        return "";
    }
    
private:
    int i = 0;
    PidController pid;
    float hand_x = 0.0;
    float previous_hand_x = 0.0;
    Drone* drone;
    Eigen::Vector3d pos;
    std::vector<std::string> gestures;
    float yaw_rate = 0.0;
    float control_speed = 0.0;

    void handleGesture(const std::string& gesture) {
        if (gesture == "Closed_Fist") {
            drone->setLocalVelocity(-this->control_speed, 0.0, 0.0, this->yaw_rate); //Closed fist -> Backward
        } else if (gesture == "Pointing_Up") {
            drone->setLocalVelocity(this->control_speed, 0.0, 0.0, this->yaw_rate); //Pointing up -> Forward
        } else if (gesture == "Victory") {
            drone->setLocalVelocity(0.0, this->control_speed, 0.0, this->yaw_rate); //Victory -> Right
        } else if (gesture == "ILoveYou") {
            drone->setLocalVelocity(0.0, -this->control_speed, 0.0, this->yaw_rate); //I Love You -> Left
        } else if (gesture == "Thumb_Down") {
            drone->setLocalVelocity(0.0, 0.0, this->control_speed, this->yaw_rate); //Thumb down -> Down
        } else if (gesture == "Thumb_Up") {
            drone->setLocalVelocity(0.0, 0.0, -this->control_speed, this->yaw_rate); //Thumb up -> Up
        }
    }
};
