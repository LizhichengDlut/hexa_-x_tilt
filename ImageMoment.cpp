/************************************************
增加发布速度的话题/mavros/setpoint_raw/local，
引入接收相对相机相对Apriltag码的话题/tag_detections
引入相机参数话题/camera/camera_info
实现从相机坐标系到像素坐标系的变换，根据Apriltag码大小
得出四个角点的像素坐标。引入图像距，计算得到期望
速度值，进行发布
Date:2022/04/12
Editor:Lizhicheng
***************************************************/

#include "ros/ros.h"

#include "mavros_msgs/PositionTarget.h"
#include<vector>

// #include "apriltags_ros/AprilTagDetectionArray.h"
#include "apriltag2_ros/Apriltags.h"

// #include "sensor_msgs/CameraInfo.h"
// #include <geometry_msgs/PoseStamped.h>

#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>
#include <cmath>

// void tagCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& tag)
// {
//     last_msg_=tag;
//     center_x = last_msg_->detections[0].pose.pose.pose.position.x;      //可视作Apriltag码中心点在相机坐标系下的位置
//     center_y = last_msg_->detections[0].pose.pose.pose.position.y;
//     center_z = last_msg_->detections[0].pose.pose.pose.position.z;

//     //使用相机内参矩阵将相机坐标系下的点转换为像素坐标

// }


ros::Publisher vel_pub;




double sq(double a)
{
    return a * a;
}

void cornerCallback(const apriltag2_ros::Apriltags& msg)
{



mavros_msgs::PositionTarget velmsg;
velmsg.header.stamp = ros::Time::now();
static int seq = 1;
velmsg.header.seq = seq++;
velmsg.header.frame_id = 1;
velmsg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
velmsg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX +
                    mavros_msgs::PositionTarget::IGNORE_PY +
                    mavros_msgs::PositionTarget::IGNORE_PZ +
                    mavros_msgs::PositionTarget::IGNORE_AFX +
                    mavros_msgs::PositionTarget::IGNORE_AFY +
                    mavros_msgs::PositionTarget::IGNORE_AFZ +
                    mavros_msgs::PositionTarget::FORCE +
                    mavros_msgs::PositionTarget::IGNORE_YAW +
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;


        cv::Mat r1 = cv::Mat(3, 1, CV_64FC1);
        cv::Mat r2 = cv::Mat(3, 1, CV_64FC1);
        cv::Mat r3 = cv::Mat(3, 1, CV_64FC1);
        cv::Mat r4 = cv::Mat(3, 1, CV_64FC1);

        tf::Matrix3x3 R_rp;

        cv::Matx33d mK(205.46963709898583, 0.0, 320.5, 0.0, 205.46963709898583, 180.5, 0.0, 0.0, 1.0);
                                // fx, 0.0, cx, 
                                // 0.0, fy, cy, 
                                // 0.0, 0.0, 1.0

        // cv::Vec4f distParam(params_.D.at<double>(0), params_.D.at<double>(1), params_.D.at<double>(2), params_.D.at<double>(3));
        cv::Vec4f distParam(0.0, 0.0, 0.0, 0.0);

    // cv::Vec3d rvec, tvec;
    //相机内参

        // geometry_msgs::PoseStamped vpdata_pose;  //20180425 image moment in the virtual plane

    //接收角点像素坐标
        r1.at<double>(0, 0) = msg.apriltags[0].corners[0].x; //
        r1.at<double>(1, 0) = msg.apriltags[0].corners[0].y; //
        r1.at<double>(2, 0) = 1;
        r2.at<double>(0, 0) = msg.apriltags[0].corners[1].x; //
        r2.at<double>(1, 0) = msg.apriltags[0].corners[1].y; //
        r2.at<double>(2, 0) = 1;
        r3.at<double>(0, 0) = msg.apriltags[0].corners[2].x; //
        r3.at<double>(1, 0) = msg.apriltags[0].corners[2].y; //
        r3.at<double>(2, 0) = 1;
        r4.at<double>(0, 0) = msg.apriltags[0].corners[3].x; //
        r4.at<double>(1, 0) = msg.apriltags[0].corners[3].y; //
        r4.at<double>(2, 0) = 1;


        ROS_INFO("r1 [0]%f",msg.apriltags[0].corners[0].x);
        ROS_INFO("r1 [1]%f",r1.at<double>(1,0));

        ROS_INFO("r2 [0]%f",msg.apriltags[0].corners[1].x);
        ROS_INFO("r2 [1]%f",r2.at<double>(1,0));
    
        ROS_INFO("r3 [0]%f",msg.apriltags[0].corners[2].x);
        ROS_INFO("r3 [1]%f",r3.at<double>(1,0));
    
        ROS_INFO("r4 [0]%f",msg.apriltags[0].corners[3].x);
        ROS_INFO("r4 [1]%f",r4.at<double>(1,0));


        cv::Matx33d mK_inv = mK.inv();//求逆//A矩阵相机固有参数矩阵
        //cv::Vec3d _r1 ;
        //cv::Vec3d  _r1= (mK_inv * r1);//标准化图像坐标系
        // cv::Vec3d _r2 = mK_inv * r2;
        // cv::Vec3d _r3 = mK_inv * r3;
        // cv::Vec3d _r4 = mK_inv * r4;

        cv::Mat  _r1= (mK_inv * r1);//标准化图像坐标系
        cv::Mat _r2 = mK_inv * r2;
        cv::Mat _r3 = mK_inv * r3;
        cv::Mat _r4 = mK_inv * r4;

        /*virtual image plane with full rotation*/

        // R_rp.setRPY(0,0,0);
        // tf::Vector3 swap_p;
        // double beta;
        // // swap_p.setValue(_r1[0], _r1[1], 1);//标准化
        // swap_p.setValue(_r1.at<double>(0,0), _r1.at<double>(1,0), 1);//标准化
        // // swap_p = R_rp * swap_p;//R_rp为实际平面到虚平面的旋转矩阵
        // beta = (double)1.0 / (swap_p.z());
        // // _r1[0] = beta * swap_p.x();
        // // _r1[1] = beta * swap_p.y();//m_v
        // _r1.at<double>(0,0) = beta * swap_p.x();
        // _r1.at<double>(1,0) = beta * swap_p.y();//m_v

        // // swap_p.setValue(_r2[0], _r2[1], 1);
        // swap_p.setValue(_r2.at<double>(0,0), _r2.at<double>(1,0), 1);
        // // swap_p = R_rp * swap_p;
        // beta = (double)1.0 / (swap_p.z());
        // // _r2[0] = beta * swap_p.x();
        // // _r2[1] = beta * swap_p.y();
        // _r2.at<double>(0,0)= beta * swap_p.x();
        // _r2.at<double>(1,0)= beta * swap_p.y();

        // // swap_p.setValue(_r3[0], _r3[1], 1);
        // swap_p.setValue(_r3.at<double>(0,0), _r3.at<double>(1,0), 1);
        // // swap_p = R_rp * swap_p;
        // beta = (double)1.0 / (swap_p.z());
        // // _r3[0] = beta * swap_p.x();
        // // _r3[1] = beta * swap_p.y();
        // _r3.at<double>(0,0) = beta * swap_p.x();
        // _r3.at<double>(1,0) = beta * swap_p.y();

        // // swap_p.setValue(_r4[0], _r4[1], 1);
        // swap_p.setValue(_r4.at<double>(0,0), _r4.at<double>(1,0), 1);
        // // swap_p = R_rp * swap_p;
        // beta = (double)1.0 / (swap_p.z());
        // // _r4[0] = beta * swap_p.x();
        // // _r4[1] = beta * swap_p.y();
        // _r4.at<double>(0,0) = beta * swap_p.x();
        // _r4.at<double>(1,0) = beta * swap_p.y();


        double cake = 0;
        double ug = 0,vg = 0;
        double az_u = 0, az_v = 0;
        double az = 0;
        double dug = 0, dvg = 0, daz = 0.125;
        
            ug = (_r1.at<double>(0, 0) + _r2.at<double>(0, 0) + _r3.at<double>(0, 0) + _r4.at<double>(0, 0)) * 0.25;//gravity center
            vg = (_r1.at<double>(1, 0) + _r2.at<double>(1, 0) + _r3.at<double>(1, 0) + _r4.at<double>(1, 0)) * 0.25;

            az_u = (sq(ug - _r1.at<double>(0, 0)) + sq(ug - _r2.at<double>(0, 0)) + sq(ug - _r3.at<double>(0, 0)) + sq(ug - _r4.at<double>(0, 0)));
            az_v = (sq(vg - _r1.at<double>(1, 0)) + sq(vg - _r2.at<double>(1, 0)) + sq(vg - _r3.at<double>(1, 0)) + sq(vg - _r4.at<double>(1, 0)));

        

        // ug = (_r1[0] + _r2[0] + _r3[0] + _r4[0]) * 0.25;
        // vg = (_r1[1] + _r2[1] + _r3[1] + _r4[1]) * 0.25;

        // az_u = (sq(ug - _r1[0]) + sq(ug - _r2[0]) + sq(ug - _r3[0]) + sq(ug - _r4[0]));
        // az_v = (sq(vg - _r1[1]) + sq(vg - _r2[1]) + sq(vg - _r3[1]) + sq(vg - _r4[1]));


        az = az_u + az_v;
        /* 1M  height */
        // daz = 0.055778;

        /* M  height */
        // daz = 0.087153125;
        //

        cake = sqrt(daz / az);
        // vpdata_pose.header.stamp = ros::Time::now();  //what is this? 
        // vpdata_pose.pose.position.x = (cake * (dvg - vg));
        // vpdata_pose.pose.position.y = (cake * (dug - ug));
        // vpdata_pose.pose.position.z = (1 - cake);
        // vpdata_pose.pose.orientation = msg.apriltags[0].pose.orientation;//

        velmsg.velocity.x=  10*(cake * (dvg - vg));//发布速度
        velmsg.velocity.y=  10*(cake * (dug - ug));
        velmsg.velocity.z=   (1 - cake);


        vel_pub.publish(velmsg);
        ROS_INFO("velmsg.x is %f",velmsg.velocity.x);
        ROS_INFO("velmsg.y is %f",velmsg.velocity.y);
        ROS_INFO("velmsg.z is %f",velmsg.velocity.z);

        // pub_vppose.publish(vpdata_pose);

}


// void camCallback(const sensor_msgs::CameraInfo::ConstPtr& cam_info)
// {

// // # Intrinsic camera matrix for the raw (distorted) images.
// // #         [fx  0 cx]
// // # K = [ 0 fy cy]
// // #         [ 0  0  1]
// // # Projects 3D points in the camera coordinate frame to 2D pixel
// // # coordinates using the focal lengths (fx, fy) and principal point
// // # (cx, cy).
    
//     // fx = cam_info->K[0];  //相机内参矩阵，使用焦距（fx,fy）及主点坐标(cx,cy), 单位为像素
//     // fy = cam_info->K[4];
//     // cx = cam_info->K[2];
//     // cy = cam_info->K[5];
//     // height = cam_info->height;
//     // width = cam_info->width;

//     //cgo3_camera K: [205.46963709898583, 0.0, 320.5, 0.0, 205.46963709898583, 180.5, 0.0, 0.0, 1.0]
//     //                             D: [0.0, 0.0, 0.0, 0.0, 0.0]

//     // cv::Vec4f distParam(params_.D.at<double>(0), params_.D.at<double>(1), params_.D.at<double>(2), params_.D.at<double>(3)); // all 0?
//     // cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
//     // cv::Matx33f r;
//     // cv::Rodrigues(rvec, r);

        

// }




int main(int argc, char **argv)
{
    ros::init(argc, argv, "ImageMoment");
    // uint32 height,width;
    // float64 fx,fy,cx,cy;
    //float64 center_x,center.y,center.z;


    tf::Matrix3x3 R_rp;

    cv::Matx33f mK(205.46963709898583, 0.0, 320.5, 0.0, 205.46963709898583, 180.5, 0.0, 0.0, 1.0);
                                // fx, 0.0, cx, 
                                // 0.0, fy, cy, 
                                // 0.0, 0.0, 1.0

    // cv::Vec4f distParam(params_.D.at<double>(0), params_.D.at<double>(1), params_.D.at<double>(2), params_.D.at<double>(3));
    cv::Vec4f distParam(0.0, 0.0, 0.0, 0.0);

    // cv::Vec3d rvec, tvec;
    //相机内参

    // geometry_msgs::PoseStamped vpdata_pose;  //20180425 image moment in the virtual plane

    // apriltags_ros::AprilTagDetectionArrayConstPtr last_msg_;

    ros::NodeHandle nh;

    vel_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 6);
    // ros::Subscriber tag_sub = nh.subscribe("/tag_detections",1000,tagCallback);
    ros::Subscriber corners_sub = nh.subscribe("/apriltag_detector/tags" , 1, cornerCallback);
    // ros::Subscriber cam_info_sub = nh.subscribe("/camera/camera_info",1000,camCallback);
    //ros::Publisher pub_vppose = nh.advertise<geometry_msgs::PoseStamped>("setpoint/relative_pos", 5);




    ros::Rate loop_rate(10);

    // mavros_msgs::PositionTarget msg;
    // msg.header.stamp = ros::Time::now();
    // static int seq = 1;
    // msg.header.seq = seq++;
    // msg.header.frame_id = 1;
    // msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX +
    //                 mavros_msgs::PositionTarget::IGNORE_PY +
    //                 mavros_msgs::PositionTarget::IGNORE_PZ +
    //                 mavros_msgs::PositionTarget::IGNORE_AFX +
    //                 mavros_msgs::PositionTarget::IGNORE_AFY +
    //                 mavros_msgs::PositionTarget::IGNORE_AFZ +
    //                 mavros_msgs::PositionTarget::FORCE +
    //                 mavros_msgs::PositionTarget::IGNORE_YAW +
    //                 mavros_msgs::PositionTarget::IGNORE_YAW_RATE;



    while (ros::ok())
    {

        // msg.velocity.x=     ;//发布速度
        // msg.velocity.y=    ;
        // msg.velocity.z=    ;


        // vel_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();


    }


    return 0;
}
