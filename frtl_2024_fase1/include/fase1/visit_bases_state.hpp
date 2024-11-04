#include <Eigen/Eigen>
#include <vector>
#include <limits>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "Base.hpp"
#include "PidController.hpp"

class VisitBasesState : public fsm::State {
public:
    VisitBasesState() : fsm::State(), x_pid(0.5, 0.0, 0.05, 0.5), y_pid(0.5, 0.0, 0.05, 0.5)  {}

    void on_enter(fsm::Blackboard &blackboard) override {
        
        drone = blackboard.get<Drone>("drone");
        drone->log("STATE: VISIT BASES");

        Base* base_to_visit = blackboard.get<Base>("base_to_visit");
        
        drone->setLocalPositionSync(base_to_visit->coordinates.x(), base_to_visit->coordinates.y(), drone->getLocalPosition().z());
        drone->log("Arrived at estimated base coordinatte.");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        bboxes = drone->getBoundingBox();

        // Find the bounding box closest to the center
        double min_distance = std::numeric_limits<double>::max();

        if (!bboxes.empty() && !previous_bboxes.empty() && bboxes[0].center_x != previous_bboxes[0].center_x) {
            for (const auto &bbox : bboxes) {
                double distance = (Eigen::Vector2d(bbox.center_x, bbox.center_y) - image_center).norm();
                if (distance < min_distance) {
                    min_distance = distance;
                    closest_bbox = bbox;
                }
            }
        }
        else{
            previous_bboxes = bboxes;
            drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
            return "";
        }

        // PID control to centralize the closest bounding box
        if (min_distance != std::numeric_limits<double>::max()) {
            x_rate = x_pid.compute(closest_bbox.center_x);
            y_rate = y_pid.compute(closest_bbox.center_y);

            drone->setLocalVelocity(x_rate, -y_rate, 0.0);

        }

        if (min_distance < 0.05){
            return "ARRIVED AT BASE";
        }

        usleep(5e04);  // Control frequency to approximately 20Hz
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void) blackboard;
    }

private:
    Eigen::Vector2d image_center = Eigen::Vector2d({0.5, 0.5});
    Drone* drone;
    PidController x_pid, y_pid;
    float x_rate = 0.0, y_rate = 0.0;
    std::vector<DronePX4::BoundingBox> bboxes, previous_bboxes;
    DronePX4::BoundingBox closest_bbox;
};