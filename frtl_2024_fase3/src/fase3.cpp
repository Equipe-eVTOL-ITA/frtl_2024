#include "fase3/initial_takeoff_state.hpp"
#include "fase3/read_state.hpp"
#include "fase3/return_home_state.hpp"
#include "fase3/go_to_base_state.hpp"
#include "fase3/centralize_base_state.hpp"
#include "fase3/Base.hpp"
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <iostream>


class Fase3FSM : public fsm::FSM {
public:
    Fase3FSM() : fsm::FSM({"ERROR", "FINISHED"}) {

        this->blackboard_set<Drone>("drone", new Drone());
        Drone* drone = blackboard_get<Drone>("drone");

        const Eigen::Vector3d fictual_home = Eigen::Vector3d({1.2, -1.10, -0.55});
        drone->setHomePosition(fictual_home);

        const Eigen::Vector3d home_pos = drone->getLocalPosition(); 
        const Eigen::Vector3d orientation = drone->getOrientation();

        std::vector<Base> bases;
        float takeoff_height = -1.5;

        const Eigen::Vector3d base_A = Eigen::Vector3d({1.0, -4.0, -1.55 + takeoff_height});
        const Eigen::Vector3d base_B = Eigen::Vector3d({2.0, -7.0, -0.10 + takeoff_height});
        const Eigen::Vector3d base_C = Eigen::Vector3d({4.0, -5.0, -0.10 + takeoff_height});
        const Eigen::Vector3d base_D = Eigen::Vector3d({6.0, -3.0, -0.10 + takeoff_height});
        const Eigen::Vector3d base_E = Eigen::Vector3d({7.0, -1.0, -1.05 + takeoff_height});

        bases.push_back({base_A, "A", false});
        bases.push_back({base_B, "B", false});
        bases.push_back({base_C, "C", false});
        bases.push_back({base_D, "D", false});
        bases.push_back({base_E, "E", false});


        this->blackboard_set<std::vector<Base>>("bases", bases);
        this->blackboard_set<Eigen::Vector3d>("home_position", home_pos);
        this->blackboard_set<bool>("finished_bases", false);
        this->blackboard_set<float>("initial_yaw", orientation[2]);
        this->blackboard_set<float>("takeoff_height", takeoff_height);


        this->add_state("INITIAL TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("GO TO BASE", std::make_unique<GoToBaseState>());
        this->add_state("CENTRALIZE BASE", std::make_unique<CentralizeBaseState>());
        this->add_state("READ", std::make_unique<ReadState>());
        this->add_state("RETURN HOME", std::make_unique<ReturnHomeState>());

        // Initial Takeoff transitions
        this->add_transitions("INITIAL TAKEOFF", {{"INITIAL TAKEOFF COMPLETED", "GO TO BASE"},{"SEG FAULT", "ERROR"}});

        // GO TO BASE transitions
        this->add_transitions("GO TO BASE", {{"AT APPROXIMATE BASE", "CENTRALIZE BASE"},{"SEG FAULT", "ERROR"}});

        // CENTRALIZE BASE transitions
        this->add_transitions("CENTRALIZE BASE", {{"CENTRALIZED BASE", "READ"},{"SEG FAULT", "ERROR"}});
        this->add_transitions("CENTRALIZE BASE", {{"LOST BASE", "GO TO BASE"}, {"SEG FAULT", "ERROR"}});

        // READ transitions
        this->add_transitions("READ", {{"GOT QR CODE", "GO TO BASE"},{"SEG FAULT", "ERROR"}});

        // Return Home transitions
        this->add_transitions("RETURN HOME", {{"AT HOME", "FINISHED"},{"SEG FAULT", "ERROR"}});
    }
};

class NodeFSM : public rclcpp::Node {
public:
    NodeFSM() : rclcpp::Node("fase3_node"), my_fsm() {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // Run at approximately 20 Hz
            std::bind(&NodeFSM::executeFSM, this));
    }

    void executeFSM() {
        if (rclcpp::ok() && !my_fsm.is_finished()) {
            my_fsm.execute();
        } else {
            rclcpp::shutdown();
        }
    }

private:
    Fase3FSM my_fsm;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    auto my_node = std::make_shared<NodeFSM>();
    rclcpp::spin(my_node);

    return 0;
}
