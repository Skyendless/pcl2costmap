#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//关于平面分割的头文件
#include <pcl/sample_consensus/model_types.h>   
#include <pcl/sample_consensus/method_types.h>   
#include <pcl/segmentation/sac_segmentation.h>  
 
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/filters/statistical_outlier_removal.h>

ros::Publisher groundPoint_pub;//输出地面的点云信息
ros::Publisher nogroundPoint_pub;//输出非地面的点云信息

double L = 0.15;
double max_L = -3.0;
double max_s = 3.5;
//double H = -0.4;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr & input)
{
  //std::string function_name = "my_pcl_ransac_ground_filter";
  //ROS_INFO("[%s]:Callback function!",function_name.c_str());
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr GroundPoint_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr NoGroundPoint_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  // 将点云格式为sensor_msgs/PointCloud2 格式转为 pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input,*cloud);

  //过滤
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_gound(new pcl::PointCloud<pcl::PointXYZ>);

    //首先对大概的地面点云进行y轴提取
    //先过滤掉太高和太远的,离群点以及降采样提高速度
    for (int i = 0; i < cloud->points.size(); i++)
    {
        if (cloud->points[i].y >= max_L && cloud->points[i].z <= max_s)
        {
            cloud_filter->points.push_back(cloud->points[i]);
        }
    }
    /* voxel filter */
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    double resolution = 0.03;   
    voxel_filter.setLeafSize(resolution, resolution, resolution);       // resolution
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp1(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter.setInputCloud(cloud_filter);
    voxel_filter.filter(*tmp1);
    tmp1->swap(*cloud_filter); 

    //statistical filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statistical_filter;
    statistical_filter.setMeanK(50);
    statistical_filter.setStddevMulThresh(1.0);
    statistical_filter.setInputCloud(cloud_filter);
    statistical_filter.filter(*tmp);
    tmp->swap(*cloud_filter); 

//然后在粗略的分割一下地面和非地面
  for (int i = 0; i < cloud_filter->points.size(); i++)
  {
    if (cloud_filter->points[i].y >= L)
    {
        GroundPoint_ptr->points.push_back(cloud_filter->points[i]);
    }
    else
    {
        NoGroundPoint_ptr->points.push_back(cloud_filter->points[i]);
    }
    
  }
  /*** 开始ransac算法 
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);//申明模型的参数
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);//申明存储模型的内点的索引
  pcl::SACSegmentation<pcl::PointXYZ> seg;// 创建一个分割方法
  seg.setOptimizeCoefficients (true);// 这一句可以选择最优化参数的因子
  seg.setModelType (pcl::SACMODEL_PLANE);   //平面模型
  seg.setMethodType (pcl::SAC_RANSAC);    //分割平面模型所使用的分割方法
  seg.setDistanceThreshold (0.01);        //设置最小的阀值距离
  seg.setInputCloud (cloud_no_gound);   //设置输入的点云
  seg.segment (*inliers,*coefficients);  // 执行ransac算法

  //pcl::PointCloud<pcl::PointXYZ>::Ptr GroundPoint_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr NoGroundPoint_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  
  double A = coefficients->values[0];
  double B = coefficients->values[1];
  double C = coefficients->values[2];
  double D = coefficients->values[2];

//pragma omp for
  double threshold = 0.01;
  for (int i = 0; i < cloud_filter->points.size(); i++)
  {
      double X = cloud_filter->points[i].x;
      double Y = cloud_filter->points[i].y;
      double Z = cloud_filter->points[i].z;
      double height = abs(A*X + B*Y + C*Z + D) / sqrt(A*A + B*B + C*C); 
      //如果垂直距离小于0.01 那么就认为是地面
      //&& (cloud->points[i].z < -0.2) )并且z小于-0.2
      if ((height <= threshold) && (cloud_filter->points[i].y >= L) )
      {
          GroundPoint_ptr->points.push_back(cloud_filter->points[i]);
      }
      else
      {
          NoGroundPoint_ptr->points.push_back(cloud_filter->points[i]);
      }
  }
  */

  //publish ground ptr and noground ptr
  sensor_msgs::PointCloud2 groundPoint_cloud_msg;
  sensor_msgs::PointCloud2 nogroundPoint_cloud_msg;
  pcl::toROSMsg(*GroundPoint_ptr, groundPoint_cloud_msg);
  pcl::toROSMsg(*NoGroundPoint_ptr, nogroundPoint_cloud_msg);
  groundPoint_cloud_msg.header = input->header;
  nogroundPoint_cloud_msg.header = input->header;
  groundPoint_pub.publish(groundPoint_cloud_msg);
  nogroundPoint_pub.publish(nogroundPoint_cloud_msg);
}
 
int main (int argc, char** argv)
{
 
    ros::init (argc, argv, "pcl2costmap");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);
    groundPoint_pub = nh.advertise<sensor_msgs::PointCloud2>("/ransac_groundPoint",100);
    nogroundPoint_pub = nh.advertise<sensor_msgs::PointCloud2>("/ransac_nogroundPoint",100);
  
  
    while (ros::ok())
    {   
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;    
}
