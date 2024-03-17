#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "include/CustomMsg.h"

ros::Publisher point_cloud_pub;
void livoxPointCloudCallback(const livox_ros_driver2::CustomMsg& input_cloud)
{ 
    // 创建一个空的 sensor_msgs::PointCloud2 对象 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 output_cloud;

    for (const auto& point : input_cloud.points) 
    {
        pcl::PointXYZ pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = point.z;
        cloud->push_back(pcl_point);
    }
   
    pcl::toROSMsg(*cloud, output_cloud);

    output_cloud.header = input_cloud.header;

    // 设置header的信息 

 //  output_cloud.header = input_cloud.header;
 //  output_cloud.header.frame_id = "livox_frame";

 //  // 设置点云数据的宽度和高度 

 //  output_cloud.width = input_cloud.point_num;
 //  output_cloud.height = 1;

 //  // 设置点云数据的fields
 //  sensor_msgs::PointField offset_time_field, x_field, y_field, z_field, reflectivity_field, tag_field, line_field; 
 //  offset_time_field.name = "offset_time";
 //  offset_time_field.offset = 0;
 //  offset_time_field.datatype = sensor_msgs::PointField::UINT32;
 //  offset_time_field.count = 1;

 //  x_field.name = "x";
 //  x_field.offset = 4;
 //  x_field.datatype = sensor_msgs::PointField::FLOAT32;
 //  // 字段中每个点的元素数量为 1
 //  x_field.count = 1;

 //  y_field.name = "y";
 //  y_field.offset = 8;
 //  y_field.datatype = sensor_msgs::PointField::FLOAT32;
 //  y_field.count = 1;

 //  z_field.name = "z";
 //  z_field.offset = 12;
 //  z_field.datatype = sensor_msgs::PointField::FLOAT32;
 //  z_field.count = 1;

 //  reflectivity_field.name = "reflectivity";
 //  reflectivity_field.offset = 16;
 //  reflectivity_field.datatype = sensor_msgs::PointField::UINT8;
 //  reflectivity_field.count = 1;

 //  tag_field.name = "tag";
 //  tag_field.offset = 17;
 //  tag_field.datatype = sensor_msgs::PointField::UINT8;
 //  tag_field.count = 1;

 //  line_field.name = "tag";
 //  line_field.offset = 18;
 //  line_field.datatype = sensor_msgs::PointField::UINT8;
 //  line_field.count = 1;

 //  output_cloud.fields.push_back(offset_time_field);
 //  output_cloud.fields.push_back(x_field);
 //  output_cloud.fields.push_back(y_field);
 //  output_cloud.fields.push_back(z_field);
 //  output_cloud.fields.push_back(reflectivity_field);
 //  output_cloud.fields.push_back(tag_field);
 //  output_cloud.fields.push_back(line_field);
 //  

 //  // 设置点云数据的is_bigendian、point_step和row_step 
 //  output_cloud.is_bigendian = false;
 //  // 每个点的字节数，对应一个u32 三个float32 和三个无符号8位整型 
 //  output_cloud.point_step = 19;
 //  output_cloud.row_step = output_cloud.point_step * input_cloud.point_num;  // 点云数据总字节数
 //  // 设置点云数据 
 //  output_cloud.data.resize(output_cloud.row_step);
 //  memcpy(output_cloud.data.data(), input_cloud.points.data(), output_cloud.row_step); 

    // 发布PointCloud2类型的点云 
    point_cloud_pub.publish(output_cloud);
    ROS_INFO_STREAM("topic livox_PointCloud2 pub");
} 

int main(int argc, char** argv)
{ 
    // 初始化ROS节点
    ros::init(argc, argv, "point_cloud_converter"); 
    ros::NodeHandle nh; 
    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("livox_PointCloud2", 10);         // 发布
    ros::Subscriber livox_pc_sub = nh.subscribe("livox/lidar", 10, livoxPointCloudCallback);  // 订阅

     tf2_ros::StaticTransformBroadcaster static_broadcaster;

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "livox_frame";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    static_broadcaster.sendTransform(transformStamped);
    
    // 循环处理事件ros::spin(); 
    ros::spin();
    return 0; 
}