#include <iostream>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

int main() {
    try {
        gtsam::NonlinearFactorGraph graph;
        std::cout << "GTSAM is installed." << std::endl;
    } catch (std::exception& e) {
        std::cout << "GTSAM is not installed." << std::endl;
    }
    return 0;
}