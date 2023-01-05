#include <ros/ros.h>
#include <vector>
#include <utility>
#include <chrono>
#include <sensor_msgs/PointCloud2.h>
#include "pointcloud_msq.h"

using namespace ros1_pointcloud2_msq;

sensor_msgs::PointCloud2 generate_random_pointcloud() {
    sensor_msgs::PointCloud2 msg;

    /* minimum pointcloud2 msg */
    msg.height = 1;
    msg.width = 1;
    msg.row_step = 1;
    msg.is_bigendian = false;
    msg.is_dense = true;

    static int send_count = 0;
    std::vector<int> data;

    if (send_count < 100) {
        data.push_back(send_count);
        msg.data.push_back(send_count);
        msg.data.push_back(send_count++);

    } else {
        send_count = 0;
    }


    return msg;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud2_pub_node");
    ros::NodeHandle n;

    /* create a server msg queue wrapper */
    PointCloud2CallBack cb;
    auto pub_msq_wrapper = create_msq_wrapper("/home/ubuntu/todo.txt", cb);

    while (true) {
        /* Generate Msg */
        auto msg = generate_random_pointcloud();
        ROS_INFO("sizeof sending msg %d", sizeof(msg));
        pub_msq_wrapper->s_ptr->enqueue(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}
