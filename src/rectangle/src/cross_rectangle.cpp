#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core.hpp>

#include "PID_controller.h"

#define ALTITUDE_TAKEOFF 5
float K[] = {277.191356, 0.0, 320.5, 0.0, 277.191356, 240.5, 0.0, 0.0, 1.0};
mavros_msgs::State current_mode;
PID_Controller *pos_control;
mavros_msgs::PositionTarget vel;
ros::Publisher local_pub;
bool land_flag = false;
bool task = false;
bool final_approach = false;
bool flag_init_position = false;
geometry_msgs::PoseStamped pose_mav_info;
geometry_msgs::PoseStamped pose_start_to_through;
geometry_msgs::PoseStamped init_position_take_off;

void uav_control_vel(mavros_msgs::PositionTarget& vel_set){
    vel_set.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    vel_set.type_mask = 
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_PX |
            mavros_msgs::PositionTarget::IGNORE_PY |
            mavros_msgs::PositionTarget::IGNORE_PZ |
            mavros_msgs::PositionTarget::IGNORE_YAW;
    local_pub.publish(vel_set);
}

void uav_control_pos(mavros_msgs::PositionTarget& pos_set){
    pos_set.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    pos_set.type_mask = 
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_PX |
            mavros_msgs::PositionTarget::IGNORE_PY |
            mavros_msgs::PositionTarget::IGNORE_PZ |
            mavros_msgs::PositionTarget::IGNORE_YAW;
    local_pub.publish(pos_set);
}

void image_cb(const sensor_msgs::Image::ConstPtr& msg){
    int center_x;
    int center_y;
    cv::Point2d center;
    cv::Point2d original_center;
    original_center.x = K[2]/2;
    center.x = K[2]/2;
    original_center.y = K[5]/2;
    center.y = K[5]/2;
    cv_bridge::CvImagePtr cv_ptr;
    static double error_y = 0;
    static double error_z = 0;
    static double error_posix_x = 0;
    static double error_posix_y = 0;
    static double biggest_area = 0.0;
    int largest_contour_index = -1;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //图像预处理
    //转换为灰度图，因为阈值分割通常在单通道图像上进行
    cv::Mat gray_image;
    cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
    cv::Mat blurred_image;
    cv::GaussianBlur(gray_image, blurred_image, cv::Size(5, 5), 0);
    cv::Mat binary_image;
    // 二值化：cv::threshold(输入图像, 输出图像, 阈值, 最大值, 方法);
    cv::threshold(blurred_image, binary_image, 50, 255, cv::THRESH_BINARY_INV);
    //cv::adaptiveThreshold(blurred_image, binary_image, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);
    //cv::inRange(blurred_image, cv::Scalar(50), cv::Scalar(60), binary_image);

    //寻找轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    // cv::findContours(输入二值图, 输出轮廓, 层次结构, 模式, 方法);
    // cv::RETR_EXTERNAL: 只检测最外层的轮廓
    // cv::CHAIN_APPROX_SIMPLE: 压缩水平、垂直和对角线段，只保留其端点
    cv::findContours(binary_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty())
    {
        double max_area = 10.0;
        largest_contour_index = -1;

        for (size_t i = 0; i < contours.size(); i++)
        {
            std::vector<cv::Point> approxContour;
            double perimeter = cv::arcLength(contours[i], true);
            double area = cv::contourArea(contours[i]);
            cv::approxPolyDP(contours[i], approxContour, 0.02 * perimeter, true);
            if (area > 1000 && area > max_area) // 示例：过滤掉面积小于100的轮廓
            {
                if (approxContour.size() == 4)
                {
                    max_area = area;
                    largest_contour_index = i;
                    ROS_INFO("目前面积：%f",max_area);
                }
            }
            if (max_area > 153683/4){
                pose_start_to_through = pose_mav_info;
                final_approach = true;
            }
        }

        if (largest_contour_index != -1)
        {
            // 计算最大轮廓的边界矩形框
            cv::Rect bounding_box = cv::boundingRect(contours[largest_contour_index]);

            center.x = bounding_box.x + bounding_box.width/2;
            center.y = bounding_box.y + bounding_box.height/2;
            cv::circle(cv_ptr->image, center, 5, cv::Scalar(0, 0, 255), -1);

            // 在原始彩色图像上绘制矩形框
            // cv::rectangle(图像, 矩形, 颜色(BGR), 线宽);
            cv::rectangle(cv_ptr->image, bounding_box, cv::Scalar(0, 255, 0), 2);

            // (可选) 绘制找到的轮廓本身
            // cv::drawContours(cv_ptr->image, contours, largest_contour_index, cv::Scalar(0, 0, 255), 2);
        }
    }

    //<-------PID计算------->
    //<像素误差计算>
    error_posix_x = center.x - original_center.x;
    error_posix_y = center.y - original_center.y;
    //<转换到机体坐标系计算>
    error_y = error_posix_x * 0.01;
    error_z = error_posix_y * 0.01;
    ROS_INFO("PID误差,y = %.3f,z = %.3f",error_y,error_z);
    //<PID控制>
    vel = pos_control->control(0, error_y, error_z);
    vel.velocity.x = 0.3;
    if(task && !final_approach){
        // ROS_INFO("无人机速度,x = %.3f,y = %.3f,z = %.3f",vel.velocity.x,vel.velocity.y,vel.velocity.z);
        if(largest_contour_index == -1)
        {
            ROS_INFO("None Frame");
            {
                vel.velocity.x = 0.5;
                vel.velocity.y = 0;
                vel.velocity.z = 0;
            }
        }
        uav_control_vel(vel);
    }
    else if(final_approach)
    {
        // ROS_INFO("Final_task");
        if (pose_mav_info.pose.position.x - pose_start_to_through.pose.position.x < 5)
        {
            vel.velocity.x = 1;
            vel.velocity.y = 0;
            vel.velocity.z = 0;
            uav_control_vel(vel);
        }
        else
        {
            if(pose_mav_info.pose.position.z < 0.3)
            {
                land_flag = true;
            }
            else
            {
                vel.velocity.x = 0;
                vel.velocity.y = 0;
                vel.velocity.z = -0.2;
                uav_control_vel(vel);
            }
        }
    }
    
    // 显示带有边界框的原始图像
    cv::circle(cv_ptr->image, original_center, 5, cv::Scalar(255, 0, 0), -1);
    cv::imshow("OPENCV_WINDOW_ORIGINAL", cv_ptr->image);
    // 显示二值化后的图像，方便调试
    cv::imshow("OPENCV_WINDOW_BINARY", binary_image);
    cv::waitKey(3);
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (flag_init_position ==false && (msg->pose.position.z!=0)) //如果还未初始化 且 z坐标不为0>>>
    {
		init_position_take_off.pose.position.x = msg->pose.position.x;
	    init_position_take_off.pose.position.y = msg->pose.position.y;
	    init_position_take_off.pose.position.z = msg->pose.position.z;
        flag_init_position = true; //>>>将“当地位置坐标”设为“初始坐标”并且将“初始化flag”设为true
    }
    pose_mav_info.pose.position.x = msg->pose.position.x;
    pose_mav_info.pose.position.y = msg->pose.position.y;
    pose_mav_info.pose.position.z = msg->pose.position.z;
}
    

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_mode = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "cross_t_node");//初始化一个ROS节点
    ros::NodeHandle nh;
    ros::Rate rate(20);
    
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = ALTITUDE_TAKEOFF;
    pos_control = new PID_Controller();

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    ros::Time last_request = ros::Time::now();//更新请求时间
    mavros_msgs::CommandBool arm_cmd;//mavros_msgs::CommandBool：ROS消息类型，用于解锁或锁定无人机
    arm_cmd.request.value = true;//将arm_cmd.request.value设置为true，表示解锁无人机

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                    ("/iris_0/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                    ("/iris_0/mavros/state", 10, state_cb);
    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>
                    ("/iris_0/usb_cam/image_raw", 1, image_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                    ("/iris_0/mavros/setpoint_position/local", 10);
    local_pub = nh.advertise<mavros_msgs::PositionTarget>
                    ("/iris_0/mavros/setpoint_raw/local",10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                    ("/iris_0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                    ("/iris_0/mavros/set_mode");
    
    while(ros::ok())
    {

        if( !current_mode.armed && (ros::Time::now() - last_request > ros::Duration(2.0))) //内层条件：检查是否解锁无人机并尝试解锁
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
                flag_init_position = false;
            }
            last_request = ros::Time::now();
        }

        if( current_mode.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0))){ //内层条件：检查是否为起飞模式并尝试切换
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
                flag_init_position = false;
            }
            last_request = ros::Time::now();
        }
        
        if(!current_mode.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) 
        {
            ROS_INFO("Vehicle takeoff");
            last_request = ros::Time::now();
            flag_init_position = false;
        }
        local_pos_pub.publish(pose);
        if(fabs(pose_mav_info.pose.position.z - init_position_take_off.pose.position.z - ALTITUDE_TAKEOFF)<0.5)
        {	
            if(ros::Time::now() - last_request > ros::Duration(2.0))
            {
                ROS_INFO("Vehicle Stable");
                task = true;
                break;
            }
        }
        // //超过10秒说明高度定位不准，直接进入任务
        // if(ros::Time::now() - last_request > ros::Duration(10.0))
        // {
        //     ROS_INFO("Takeoff timeout, start the task ");
        //     task = true;
        //     break;
        // }
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok())
    {
        if(land_flag == true)
        {
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Land");
                
            }
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }
    
    delete pos_control;
}