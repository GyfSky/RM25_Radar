//
// Created by plusseven on 24-3-23.
//

#include "../include/UnityRect.h"

void UnityRect::getRect(const interfaces::msg::Rect::ConstPtr &rosRect_ptr){
    rect.x = rosRect_ptr->x;
    rect.y = rosRect_ptr->y;
    rect.width = rosRect_ptr->w;
    rect.height = rosRect_ptr->h;
    std::cout << "rect: " << rect << std::endl;
}

void UnityRect::init(int argc,char *argv[]){
    // ros::init(argc, argv, "unityRect_listener");
    rclcpp::init(argc, argv);
    // ros::NodeHandle nh;
    auto nh = rclcpp::Node::make_shared("unityRect_listener"); 
    // sub_rect = nh.subscribe<RADAR24_ROS::Rect>("rect",1,&UnityRect::getRect,this);
    sub_rect=nh->create_subscription<interfaces::msg::Rect>("rect", 1, std::bind(&UnityRect::getRect, this, std::placeholders::_1));
}