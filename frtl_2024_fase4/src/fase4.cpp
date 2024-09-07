#include "initial_takeoff_state.hpp"
#include "landing_state.hpp"
#include "return_home_state.hpp"
#include "search_state.hpp"
#include "takeoff_state.hpp"
#include "gesture_control_state.hpp"
#include <rclcpp/rclcpp.hpp>
 
#include <memory>
#include <iostream>


class Fase1FSM : public fsm::FSM {
public:
    Fase1FSM() : fsm::FSM({"ERROR", "FINISHED"}) {

        this->blackboard_set<Drone>("drone", new Drone());

        this->blackboard_set<float>("takeoff height", -1.3);
        this->blackboard_set<float>("control speed", 0.5);
        this->blackboard_set<float>("yaw speed", 0.2);


        this->add_state("INITIAL TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("SEARCH", std::make_unique<SearchState>());
        this->add_state("GESTURE CONTROL", std::make_unique<GestureControlState>());
        this->add_state("LANDING", std::make_unique<LandingState>());
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("RETURN HOME", std::make_unique<ReturnHomeState>());

        // Initial Takeoff transitions
        this->add_transitions("INITIAL TAKEOFF", {{"INITIAL TAKEOFF COMPLETED", "SEARCH"},{"SEG FAULT", "ERROR"}});

        // Search transitions
        this->add_transitions("SEARCH", {{"HAND FOUND", "GESTURE CONTROL"},{"SEG FAULT", "ERROR"}});

        // Gesture Control transitions
        this->add_transitions("GESTURE CONTROL", {{"LAND NOW", "LANDING"},{"SEG FAULT", "ERROR"}});
        this->add_transitions("GESTURE CONTROL", {{"GO HOME", "RETURN HOME"},{"SEG FAULT", "ERROR"}});

        // Landing transitions
        this->add_transitions("LANDING", {{"LANDED", "TAKEOFF"},{"SEG FAULT", "ERROR"}});

        // Takeoff transitions
        this->add_transitions("TAKEOFF", {{"TAKEOFF COMPLETED", "GESTURE CONTROL"},{"SEG FAULT", "ERROR"}});

        // Return Home transitions
        this->add_transitions("RETURN HOME", {{"AT HOME", "FINISHED"},{"SEG FAULT", "ERROR"}});
        
    }
};

class NodeFSM : public rclcpp::Node {
public:
    NodeFSM() : rclcpp::Node("fase4_node") {}
    Fase1FSM my_fsm;
};

int main(int argc, const char * argv[]){
    rclcpp::init(argc,argv);

    auto my_node = std::make_shared<NodeFSM>();
    while (rclcpp::ok() && !my_node->my_fsm.is_finished()) {
        my_node->my_fsm.execute();
        rclcpp::spin_some(my_node);
    }

    std::cout << my_node->my_fsm.get_fsm_outcome() << std::endl;
    rclcpp::shutdown();
    
    return 0;
}


