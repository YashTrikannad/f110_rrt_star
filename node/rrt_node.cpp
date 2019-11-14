// This file contains the node definition for RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Author: Yash Trikannad

#include "f110_rrt_star/rrt.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "f110_rrt_star");
    ros::NodeHandle nh;
    RRT rrt(nh);
    ros::spin();
    return 0;
}
