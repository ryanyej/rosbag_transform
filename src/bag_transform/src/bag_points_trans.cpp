#include <dirent.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cmath>
#include <vector>
#include <time.h>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "geometry_msgs/Quaternion.h"
#include <dirent.h>
#include <tf/transform_datatypes.h>
#include<string>
#include "sensor_msgs/PointCloud2.h"

std::string path="/home/user/old/";
std::string new_path="/home/user/new/";

ros::Publisher no_frame_id_lidar;
Eigen::Matrix4f vec_pose_to_matrix(std::vector<double> &vec_pose)
{
    ///generate transform matrix according to current pose
    Eigen::AngleAxisf current_rotation_x(vec_pose[3], Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf current_rotation_y(vec_pose[4], Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf current_rotation_z(vec_pose[5], Eigen::Vector3f::UnitZ());
    Eigen::Translation3f current_translation(vec_pose[0], vec_pose[1], vec_pose[2]);
    Eigen::Matrix4f transform_matrix =
        (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();

    return transform_matrix;
}

void callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
                sensor_msgs::PointCloud2 pub_point;
                pub_point=*msg;
                pub_point.width=0; 
                pub_point.data={};
                no_frame_id_lidar.publish(pub_point);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_transform", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    // ros::Subscriber lidar_receive=nh.subscribe<sensor_msgs::PointCloud2>("/mems_sensing/lidar/front/zvision_lidar_points",10,&callback);
    // no_frame_id_lidar=nh.advertise<sensor_msgs::PointCloud2>("/mems_sensing/lidar/ryan/pointcloud",10);
    // tf::Quaternion quat;
    // geometry_msgs::Quaternion msg;
    // msg.x=0.00133468628804;
    // msg.y=0.0154031408186;
    // msg.z= -2.05618878181e-05;
    // msg.w=0.999880473578; 
    // tf::quaternionMsgToTF(msg, quat);
    // float roll, pitch, yaw,x,y,z;
    // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    std::vector<double> transform_from_lidar_to_kit,tranform_from_kit_to_left_lidar,transform_from_left_lidar_to_base_link;
    transform_from_lidar_to_kit.resize(6);
    tranform_from_kit_to_left_lidar.resize(6);
    transform_from_left_lidar_to_base_link.resize(6);
    double lidar2kit[6]={0.0,0.0,0.0,-0.11,0.1,0.0};
    double kit2left_lidar[6]={-0.055,-0.595,0.45,0.050,0.4712389,0.0};
    double left_lidar2_base_link[6]={1.871,0.575,1.24,0.0,-0.0525344,-0.0038397};
    transform_from_lidar_to_kit.insert(transform_from_lidar_to_kit.begin(),lidar2kit,lidar2kit+6);
    tranform_from_kit_to_left_lidar.insert(tranform_from_kit_to_left_lidar.begin(),kit2left_lidar,kit2left_lidar+6);
    transform_from_left_lidar_to_base_link.insert(transform_from_left_lidar_to_base_link.begin(),left_lidar2_base_link,left_lidar2_base_link+6);
    Eigen::Matrix4f mat1=vec_pose_to_matrix(transform_from_lidar_to_kit);
    Eigen::Matrix4f mat2=vec_pose_to_matrix(tranform_from_kit_to_left_lidar);
    Eigen::Matrix4f mat3=vec_pose_to_matrix(transform_from_left_lidar_to_base_link);
    mat1=mat1.inverse().eval();
    // Eigen::Matrix4f trans_matrix=mat3*mat2*mat1;
    // Eigen::Transform<float, 3, Eigen::Affine> affine (trans_matrix);
    // pcl::getTranslationAndEulerAngles (affine, x, y, z, roll, pitch, yaw);
    // ROS_INFO("X=%f,Y=%f,Z=%f,ROLL=%f,pitch=%f,yaw=%f.",x,y,z,roll,pitch,yaw);
    DIR *pDir;
    struct dirent* ptr;

    const char* ccpath=path.c_str();
    if(!(pDir=opendir(ccpath)))
    {
        std::cout<<"Folder doesn't Exist!"<<std::endl;
        return -1;
    }

    while((ptr=readdir(pDir))!=0)
    {

            if(strstr(ptr->d_name,".bag")!=NULL)
            {
            rosbag::Bag i_bag,o_bag;
            std::cout<<path+ptr->d_name<<std::endl;
            i_bag.open(path+ptr->d_name,rosbag::bagmode::Read);
            o_bag.open(new_path+ptr->d_name,rosbag::bagmode::Write);
            // p.reset(new pcl::visualization::PCLVisualizer(argc, argv, "Online PointCloud2 Viewer"));
            std::vector<std::string>topics;
            topics.push_back(std::string("/rslidar_points"));
            rosbag::View view(i_bag,rosbag::TopicQuery(topics));
            for(auto m:view)
            {
                sensor_msgs::PointCloud2::ConstPtr mems=m.instantiate<sensor_msgs::PointCloud2>();
                if(mems==nullptr)
                {
                    std::cout<<"mems null."<<std::endl;
                }
                else
                {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
                    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr1(new pcl::PointCloud<pcl::PointXYZI>);
	                // pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr2(new pcl::PointCloud<pcl::PointXYZI>);
                    pcl::fromROSMsg(*mems,*out_cloud_ptr);
                    pcl::transformPointCloud(*out_cloud_ptr, *out_cloud_ptr1, mat1);
                    // pcl::transformPointCloud(*out_cloud_ptr1, *out_cloud_ptr2, mat2);
                    out_cloud_ptr1->header.frame_id = "base_link";
                    sensor_msgs::PointCloud2 t_rosCloud;
                    pcl::toROSMsg(*out_cloud_ptr1,t_rosCloud);
                    o_bag.write("rslidar_points",mems->header.stamp,t_rosCloud);
                }
            }
            ROS_INFO("Finished transform %s",ptr->d_name);
    }
    else
    {
        closedir(pDir);
        return (0);
    }
    }
   closedir(pDir);
ros::spin();
    return (0);
}
