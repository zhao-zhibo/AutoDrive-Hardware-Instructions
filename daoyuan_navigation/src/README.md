E70 定位程序  iau_ros_location
=======================

代码功能：

1.车辆轮速以及档位信息接收；
2.基于导远组合导航协议的定位信息接收；
3.轨迹录制；

代码结构：

* include
    * LoopThread.h
* src
    * collect_node:路径采集程序文件
    * pos_dy_node: 轮速/档位信息接收以及
* ZCANBusSocket: