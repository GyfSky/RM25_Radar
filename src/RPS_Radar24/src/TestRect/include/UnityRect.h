//
// Created by plusseven on 24-3-23.
//

#ifndef JSONCPP_TEST_UNITYRECT_H
#define JSONCPP_TEST_UNITYRECT_H

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/rect.hpp"
#include "interfaces/msg/image_byte.hpp"
#include "../../General/include/General.h"

class UnityRect {
private:
    // ros::NodeHandle nh;
    // ros::Subscriber sub_rect;
    rclcpp::Subscription<interfaces::msg::Rect>::SharedPtr sub_rect;
public: 
    cv::Rect rect;
    UnityRect() = default;
    void getRect(const interfaces::msg::Rect::ConstPtr &rosRect_ptr);//
    void init(int argc,char *argv[]);
};


#endif //JSONCPP_TEST_UNITYRECT_H
