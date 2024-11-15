#include <Eigen/Eigen>
#include <vector>
#include <limits>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "Base.hpp"
#include "PidController.hpp"

#include <deque>
#include <iostream>
#include <string>
#include <array>

class CentralizeBaseState : public fsm::State {
public:
    CentralizeBaseState() : fsm::State(), x_pid(0.9, 0.0, 0.05, 0.5), y_pid(0.9, 0.0, 0.05, 0.5)  {}

    void on_enter(fsm::Blackboard &blackboard) override {
        
        drone = blackboard.get<Drone>("drone");
        drone->log("STATE: CENTALIZE BASE");

    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        bboxes = drone->getBoundingBox();

        // Find the bounding box closest to the center
        double min_distance = std::numeric_limits<double>::max();
        if (!bboxes.empty()) {
            updateBBoxesBuffer(bboxes[0].center_x);
            for (const auto &bbox : bboxes) {
                double distance = (Eigen::Vector2d(bbox.center_x, bbox.center_y) - image_center).norm();
                if (distance < min_distance) {
                    min_distance = distance;
                    closest_bbox = bbox;
                }
            }
        }
        else{
            drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
            updateBBoxesBuffer(0.0);
            return "";
        }

        //BBox detection is the same for the last 5s
        if (allElementsEqual(bboxes_buffer))
        {
            drone->log("Last 5s had no detections!");
            return "LOST BASE";
        }
        //Good to do PID Control
        else
        {
            x_rate = x_pid.compute(closest_bbox.center_y);
            y_rate = y_pid.compute(closest_bbox.center_x);
            drone->setLocalVelocity(x_rate, -y_rate, 0.0);
        }

        if (min_distance < 0.03){
            return "CENTRALIZED BASE";
        }

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        x_rate = 0.0;
        y_rate = 0.0;
        bboxes_buffer.clear();
    }

private:
    Drone* drone;
    std::vector<DronePX4::BoundingBox> bboxes;
    DronePX4::BoundingBox closest_bbox;
    std::deque<float> bboxes_buffer;

    Eigen::Vector2d image_center = Eigen::Vector2d({0.5, 0.5});
    PidController x_pid, y_pid;
    float x_rate = 0.0, y_rate = 0.0;


    void updateBBoxesBuffer(float new_center_x)
    {
        if (bboxes_buffer.size() >= 100){
            bboxes_buffer.pop_front();
        }
        bboxes_buffer.push_back(new_center_x);
    }

    bool allElementsEqual(const std::deque<float>& buffer)
    {
        if (buffer.empty()) return true;
        float first_value = buffer[0];
        for (const auto& value : buffer){
            if (value != first_value) {
                return false;
            }
        }
        return buffer.size() >= 99; // 5 seconds
    }
};