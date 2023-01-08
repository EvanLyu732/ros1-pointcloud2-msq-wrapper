# ros1-pointcloud2-msq-wrapper


## 简述：
  提供了一个基于System V的消息队列对于ros1消息sensor_msgs::PointCloud2的封装

## 背景：
  由于ros1主从机模式下（x86为主，arm64为从）存在着网络丢包现象。比如，在arm64上启动基于[一经雷达sdk](https://github.com/ZVISION-lidar/zvision_sdk)的驱动封装， 再在arm64上启动ros1订阅程序。此时由于x86为主，roscore是在x86上启动的。因此
  arm64会与x86进行一次网络传输。导致雷达丢包（TODO：增加具体日志）。因此在不大改动（内嵌代码，更换ros等其他方式）的情况下，此时考虑的解决方式则是通过ipc（共享内存，unix domain socket等其他方式）进行消息传输。本项目提供了提供了一个基于unix默认的消息队列对于ros1消息sensor_msgs::PointCloud2的封装。
  
## 示例：

```cpp

```
 
  
  
