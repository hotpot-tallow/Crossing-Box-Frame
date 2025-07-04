//
// Created by xyl on 24-5-15.
//

#include "PID_controller.h"

PID_Controller::~PID_Controller() {
    delete x_pid_controller;
    delete y_pid_controller;
    delete yaw_pid_controller;
    delete z_pid_controller;
}

PID_Controller::PID_Controller() {
    x_pid_controller = new PID<double>();
    y_pid_controller = new PID<double>();
    z_pid_controller = new PID<double>();
    yaw_pid_controller = new PID<double>();

    double temp;
    ros::param::get("pos_kp_x",temp);//加载x轴PID控制器参数
    x_pid_controller->set_kp(temp);
    ros::param::get("pos_ki_x",temp);
    x_pid_controller->set_ki(temp);
    ros::param::get("pos_kd_x",temp);
    x_pid_controller->set_kd(temp);
    ros::param::get("pos_ol_x",temp);
    x_pid_controller->set_ol(temp);
    ros::param::get("pos_sp_x",temp);
    x_pid_controller->set_sp(temp);

    ros::param::get("pos_kp_y",temp);//加载y轴PID控制器参数
    y_pid_controller->set_kp(temp);
    ros::param::get("pos_ki_y",temp);
    y_pid_controller->set_ki(temp);
    ros::param::get("pos_kd_y",temp);
    y_pid_controller->set_kd(temp);
    ros::param::get("pos_ol_y",temp);
    y_pid_controller->set_ol(temp);
    ros::param::get("pos_sp_y",temp);
    y_pid_controller->set_sp(temp);

    ros::param::get("height_kp",temp);//加载高度控制器参数
    z_pid_controller->set_kp(temp);
    ros::param::get("height_ki",temp);
    z_pid_controller->set_ki(temp);
    ros::param::get("height_kd",temp);
    z_pid_controller->set_kd(temp);
    ros::param::get("height_ol",temp);
    z_pid_controller->set_ol(temp);
    ros::param::get("height_sp",temp);
    z_pid_controller->set_sp(temp);

    ros::param::get("yaw_kp",temp);//加载偏航角控制器参数
    yaw_pid_controller->set_kp(temp);
    ros::param::get("yaw_ki",temp);
    yaw_pid_controller->set_ki(temp);
    ros::param::get("yaw_kd",temp);
    yaw_pid_controller->set_kd(temp);
    ros::param::get("yaw_ol",temp);
    yaw_pid_controller->set_ol(temp);
    ros::param::get("yaw_sp",temp);
    yaw_pid_controller->set_sp(temp);

    ros::param::get("offset_docking_x",temp);//对接补偿x轴
    offset_docking_x = temp;
    ros::param::get("offset_docking_y",temp);//对接补偿y轴
    offset_docking_y = temp;
    ros::param::get("offset_docking_z",temp);//对接补偿z轴
    offset_docking_z = temp;
}

mavros_msgs::PositionTarget PID_Controller::control(double error_x,double error_y,double error_z) 
{

    static uint8_t control_step = 0;

    double velocity_x = 0,velocity_y = 0,velocity_z = 0,velocity_yaw = 0;

    //修正偏差
    error_x += offset_docking_x;
    error_y += offset_docking_y;
    error_z += offset_docking_z;
    velocity_y = y_pid_controller->PID_caculate(0,error_y);
    velocity_x = x_pid_controller->PID_caculate(0,error_x);
    velocity_z = z_pid_controller->PID_caculate(0,error_z);
    //  switch (control_step)
    //  {
    //    case 0:
    //    {
    //         //x方向速度为向东，y方向速度向北
    //         velocity_x = x_pid_controller->PID_caculate(0,error_x);
    //         velocity_y = y_pid_controller->PID_caculate(0,error_y);
    //         //velocity_z = z_pid_controller->PID_caculate(0,error_z);
    //         if(abs(error_x) < 0.1 && abs(error_y) < 0.1)
    //             {
    //                 control_step ++;
    //             }
    //         break;
                
    //    }
    //    case 1:
    //    {
    //        velocity_yaw = yaw_pid_controller->PID_caculate(0,error_yaw);
    //        break;
    //    }
    //    default:
    //        break;
    // }
    mavros_msgs::PositionTarget set_speed;
    set_speed.velocity.x = velocity_x;
    set_speed.velocity.y = velocity_y;
    set_speed.velocity.z = velocity_z;

    // set_speed.angular.z = velocity_yaw;
    return set_speed;
}
