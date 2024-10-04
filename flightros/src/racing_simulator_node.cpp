#include "flightros/racing_simulator.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "racing_simulator");
    RacingSimulator simulator;
    ros::spin();
    return 0;
}