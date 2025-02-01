#include "lane_filter_visualizer/lidar_lane_detector.h"

/*
차선 검출하는 코드
*/

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

