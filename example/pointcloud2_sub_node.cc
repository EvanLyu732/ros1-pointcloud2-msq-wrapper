#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "pointcloud_msq.h"
#include <iostream>

using namespace ros1_pointcloud2_msq;

void print_pointcloud2(const sensor_msgs::PointCloud2 &msg) {
    std::cout << "---------------------------" << '\n';
    std::cout << " msg.height: " << msg.height << '\n';
    std::cout << " msg.width: " << msg.width << '\n';
    std::cout << " msg.row_step: " << msg.row_step << '\n';
    std::cout << " msg.is_bigendian: " << msg.is_bigendian << '\n';
    std::cout << " msg.data: ";

    for (const auto& data: msg.data) {
        std::cout << data;
    }
    std::cout << std::endl;
    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud2_sub_node");
    ros::NodeHandle n;
    int rate = 10;

    auto sub_msq_wrapper = create_msq_wrapper("/home/ubuntu/todo.txt", &print_pointcloud2);
//    auto sub_msq_wrapper = create_msq_wrapper("/demo_channel", &print_pointcloud2);
    sub_msq_wrapper->c_ptr->loop(rate);
    return 0;
}