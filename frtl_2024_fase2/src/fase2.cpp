#include "fase2/initial_takeoff_state.hpp"
#include "fase2/landing_state.hpp"
#include "fase2/return_home_state.hpp"
#include "fase2/search_bases_state.hpp"
#include "fase2/takeoff_state.hpp"
#include "fase2/visit_bases_state.hpp"
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <iostream>


class Fase2FSM : public fsm::FSM {
public:
    Fase2FSM() : fsm::FSM({"ERROR", "FINISHED"}) {

        this->blackboard_set<Drone>("drone", new Drone());
        //Drone* drone = blackboard_get<Drone>("drone");

        this->blackboard_set<float>("takeoff_height", -3.0);

        this->add_state("INITIAL TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("SEARCH BASES", std::make_unique<SearchBasesState>());
        this->add_state("VISIT BASE", std::make_unique<VisitBasesState>());
        this->add_state("LANDING", std::make_unique<LandingState>());
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("RETURN HOME", std::make_unique<ReturnHomeState>());

        // Initial Takeoff transitions
        this->add_transitions("INITIAL TAKEOFF", {{"INITIAL TAKEOFF COMPLETED", "SEARCH BASES"},{"SEG FAULT", "ERROR"}});

        // Search Bases transitions
        this->add_transitions("SEARCH BASES", {{"BASES FOUND", "VISIT BASE"},{"SEG FAULT", "ERROR"}});
        this->add_transitions("SEARCH BASES", {{"SEARCH ENDED", "RETURN HOME"},{"SEG FAULT", "ERROR"}});

        // Visit Base transitions
        this->add_transitions("VISIT BASE", {{"ARRIVED AT BASE", "LANDING"},{"SEG FAULT", "ERROR"}});

        // Landing transitions
        this->add_transitions("LANDING", {{"LANDED", "TAKEOFF"},{"SEG FAULT", "ERROR"}});

        // Takeoff transitions
        this->add_transitions("TAKEOFF", {{"NEXT BASE", "SEARCH BASES"},{"SEG FAULT", "ERROR"}});
        this->add_transitions("TAKEOFF", {{"FINISHED BASES", "RETURN HOME"},{"SEG FAULT", "ERROR"}});

        // Return Home transitions
        this->add_transitions("RETURN HOME", {{"AT HOME", "FINISHED"},{"SEG FAULT", "ERROR"}});
        
    }
};

class NodeFSM : public rclcpp::Node {
public:
    NodeFSM() : rclcpp::Node("fase2_node") {}
    Fase2FSM my_fsm;
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


