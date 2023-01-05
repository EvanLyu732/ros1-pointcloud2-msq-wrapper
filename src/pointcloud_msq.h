#pragma once

#include <cstdint>
#include <memory>
#include <ratio>
#include <stdexcept>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/types.h>

#ifndef SENSOR_MSGS_MESSAGE_POINTCLOUD2_H
#include "PointCloud2.h"
#endif

#ifndef SENSOR_MSGS_MESSAGE_POINTCLOUD2_H
#include "sensor_msgs/PointCloud2.h"
#endif

#include <chrono>
#include <stdexcept>
#include <string>
#include <cstring>
#include <thread>
#include <utility>

#include "ros/ros.h"


/*
Usage:
    // server side
    auto wrapper_ptr = std::make_shared<::ros1_pointcloud2_msq::msq_wrapper>("/var/msq/192.168.1.23", void*);
    pcl::toRosMsg(pointcloud_ptr,msg);
    // ...
    // Tf transform 
    // ...
    wrapper_ptr->server->enqueue(msg);


    // client side
    auto wrapper_ptr = std::make_shared<::ros1_pointcloud2_msq::msq_wrapper>("/var/msq/192.168.1.23", void*);
    wrapper_ptr->client->set_cb();
    wrapper_ptr->client->loop(50);
*/


namespace ros1_pointcloud2_msq {

    using sensor_msgs::PointCloud2;

    using PointCloud2CallBack = void (*)(const sensor_msgs::PointCloud2 &msg);

    /* the unix message queue wrapper */
    struct msq_wrapper;
    /* message for sending to message queue */
    struct PointCloud2Buf;

    static std::shared_ptr<msq_wrapper> create_msq_wrapper(const char *channel, PointCloud2CallBack cb) noexcept {
        return std::make_shared<msq_wrapper>(channel, cb);
    }

    struct PointCloud2Buf {
        long mType;
        ::sensor_msgs::PointCloud2 data;
    };


    /* unix default message queue wrapper for sending point cloud */
    struct msq_wrapper : public std::enable_shared_from_this<msq_wrapper> {
        struct server;
        struct client;

        using channel_id = int;

    public:
        /* sturct size */
        size_t buf_size;
        server *s_ptr;
        client *c_ptr;

    public:
        /* Server is used for sending */
        struct server {

            server(const char *channel_path) {
                /* initial msq_queue*/
                key_t k = ftok(channel_path, 'b');
                channel_id id_ = msgget(k, 0666 | IPC_CREAT);
                if (id_ <= 0) {
                    std::string error = "msg wrapper error create channel " + std::string(channel_path);
                    throw std::runtime_error(std::move(error));
                } else {
                    ROS_INFO("create pub channel %d success", id_);
                }
            }

            void enqueue(const ::sensor_msgs::PointCloud2& msg) {
                struct PointCloud2Buf buf;
                buf.data = std::move(msg);
                buf.mType = 0;

                int res = msgsnd(id_, &buf, sizeof(buf), 0);
                if (res < 0) {
                    // log & abort
                    ROS_ERROR("send pointcloud2 msg failed, reason: %s", strerror(errno));
                }
            }

        private:
            channel_id id_;
        };


        /* Client is used for receving */
        struct client {
            client(const char *channel_path, PointCloud2CallBack cb) : cb_(cb) {
                key_t k = ftok(channel_path, 'b');
                channel_id id_ = msgget(k, 0666 | IPC_CREAT);
                if (id_ > 0) {
                    ROS_INFO("create sub channel %d success", id_);
                }
            }

            void loop(uint8_t rate) {
                struct PointCloud2Buf buf;

                while (true) {
                    ssize_t s = msgrcv(id_, &buf, sizeof(struct PointCloud2Buf), 0, 0);
                    if (s > 0) {
                        const ::sensor_msgs::PointCloud2 msg = std::move(buf.data);
                        if (cb_) {
                            cb_(msg);
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(rate));
                }
            }

            void set_cb(PointCloud2CallBack cb) {
                cb_ = cb;
            }

        private:
            PointCloud2CallBack cb_;
            channel_id id_;
        };

    public:
        msq_wrapper(const char *channel_path, PointCloud2CallBack cb) {
            struct PointCloud2Buf m;
            buf_size = sizeof(m.data);
            s_ptr = new server(channel_path);
            c_ptr = new client(channel_path, cb);
        }

        ~msq_wrapper() {
            if (s_ptr) delete s_ptr;
            if (c_ptr) delete c_ptr;
        }
    };
}// namespace ros1_pointcloud2_msq
