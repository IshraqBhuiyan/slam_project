#include <pcl/common/angles.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/random_sample.h>
#include <nav_msgs/Odometry.h>
#include <pcl/search/kdtree.h>
#include <Eigen/Core>
#include <cmath>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/filter.h>


typedef pcl::PointNormal PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

PointCloudT::Ptr last_scan;
PointCloudT::Ptr built_map;
PointCloudT::Ptr full_map;

ros::Publisher odom_pub, built_map_pub, debug_pc_pub;
ros::Time stamp;

Eigen::Affine3d base_to_odom;
Eigen::Affine3d world_to_odom;
pcl::VoxelGrid<PointT> grid;
const float leaf = 0.005f;

tf::TransformListener* tf_listener;

int global_iters;

void icp_slam_iter(pcl::PointCloud<PointT>::Ptr &curr_scan);

void icp_global_map();

void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &input_cloud){
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::RandomSample<PointT> rand_filt;
  tf::StampedTransform vel_to_base;
  pcl::NormalEstimationOMP<PointT, PointT> nest;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  stamp = input_cloud->header.stamp;

  try {
    tf_listener->waitForTransform("/base_link",
                                 (*input_cloud).header.frame_id.c_str(),
                                 (*input_cloud).header.stamp, ros::Duration(1.0));
    tf_listener->lookupTransform("/base_link",
                                (*input_cloud).header.frame_id.c_str(),
                                (*input_cloud).header.stamp, vel_to_base);
  } catch (...) {
    return;
  }

  pcl::PCLPointCloud2::Ptr temp_cloud(new pcl::PCLPointCloud2);
  pcl::PointCloud<PointT>::Ptr cloudtemp(new pcl::PointCloud<PointT>);
  pcl_conversions::toPCL(*input_cloud, *temp_cloud);
  pcl::fromPCLPointCloud2(*temp_cloud, *cloudtemp);
  pcl_ros::transformPointCloud(*cloudtemp, *cloud, vel_to_base);
  cloud->header.frame_id = "base_link";

  // Uniformly randomly subsample pointcloud to have every 20 points
  rand_filt.setSample(input_cloud->width*input_cloud->height/10);
  rand_filt.filter(*cloud);

  nest.setSearchMethod(tree);
  nest.setKSearch(5);
  nest.setInputCloud(cloud);
  nest.compute(*cloud);

  if(last_scan->size() == 0){
    *last_scan = *cloud;
  }else{
    icp_slam_iter(cloud);
  }
}

void icp_slam_iter(pcl::PointCloud<PointT>::Ptr &curr_scan){
  sensor_msgs::PointCloud2 pub_cloud;
  pcl::toROSMsg(*last_scan, pub_cloud);
  pub_cloud.header.frame_id = "base_link";
  debug_pc_pub.publish(pub_cloud);

  pcl::IterativeClosestPointWithNormals<PointT, PointT> icp;
  pcl::PointCloud<PointT> transf_cloud;
  icp.setMaximumIterations(40);
  icp.setMaxCorrespondenceDistance(0.05);
  //ROS_INFO("0");
  icp.setInputSource(last_scan);
  icp.setInputTarget(curr_scan);
  //ROS_INFO("1");
  icp.align(transf_cloud);

  if(icp.hasConverged()){
    *last_scan = *curr_scan;
    //ROS_INFO("2");
    global_iters++;
    Eigen::Matrix4d relative_tf_m = icp.getFinalTransformation().cast<double>();
    Eigen::Affine3d relative_tf;
    relative_tf.matrix() = relative_tf_m.inverse();
    base_to_odom = relative_tf* base_to_odom;
    pcl::transformPointCloud<PointT>(*curr_scan, transf_cloud, base_to_odom.cast<float>());
    *built_map += transf_cloud;
    grid.setLeafSize(leaf,leaf,leaf);
    grid.setInputCloud(built_map);
    grid.filter(*built_map);
    //ROS_INFO("3");

    sensor_msgs::PointCloud2 pub_cloud;
    pcl::toROSMsg(*built_map, pub_cloud);
    pub_cloud.header.frame_id = "map";
    built_map_pub.publish(pub_cloud);
    //ROS_INFO("4");

    nav_msgs::Odometry odom;
    odom.header.frame_id = "map";
    tf::Pose tf_pose;
    tf::poseEigenToTF(base_to_odom, tf_pose);
    //ROS_INFO_STREAM("base to odom\n" << base_to_odom.matrix());
    tf::poseTFToMsg(tf_pose, odom.pose.pose);
    odom_pub.publish(odom);
    //ROS_INFO("5");

    static tf::TransformBroadcaster br;
    tf::Transform tf_odom;
    tf::transformEigenToTF(base_to_odom, tf_odom);
    br.sendTransform(tf::StampedTransform(tf_odom, ros::Time::now(), "map", "base_link"));
    //ROS_INFO("Sent transform for odom to base_link");
    if(global_iters>=10){
      //icp_global_map();
      global_iters=0;
    }

  }

}

void icp_global_map(){
  //ROS_INFO("Got here too");
  pcl::PointCloud<PointT>::Ptr map_down(new PointCloudT);
  PointCloudT::Ptr built_map_down(new PointCloudT);
  *built_map_down = *built_map;
  *map_down = *full_map;
  pcl::RandomSample<PointT> rand_filt;
  rand_filt.setSample(built_map->size()/4);
  rand_filt.filter(*built_map_down);
  rand_filt.setSample(full_map->size()/10);
  rand_filt.filter(*map_down);
  /*grid.setLeafSize(leaf*10,leaf*10,leaf*10);
  grid.setInputCloud(built_map);
  grid.filter(*built_map_down);
  grid.setInputCloud(full_map);
  grid.filter(*map_down);*/
  //ROS_INFO("6");
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*built_map_down, *built_map_down, indices);
  pcl::removeNaNNormalsFromPointCloud(*built_map_down,*built_map_down, indices);
  pcl::removeNaNFromPointCloud(*map_down, *map_down, indices);
  pcl::removeNaNNormalsFromPointCloud(*map_down,*map_down, indices);
  
  pcl::IterativeClosestPointWithNormals<PointT, PointT> icp;
  pcl::PointCloud<PointT> transf_cloud;
  icp.setMaximumIterations(10);
  icp.setMaxCorrespondenceDistance(0.05);
  icp.setInputTarget(map_down);
  icp.setInputSource(built_map_down);
  //ROS_INFO("7");
  icp.align(transf_cloud, world_to_odom.matrix().cast<float>());
  //ROS_INFO("8");

  if(icp.hasConverged()){
    //ROS_INFO("9");
    static tf::TransformBroadcaster br2;
    world_to_odom.matrix() = icp.getFinalTransformation().cast<double>().inverse();
    tf::Transform tf_odom;
    tf::transformEigenToTF(world_to_odom, tf_odom);
    br2.sendTransform(tf::StampedTransform(tf_odom, ros::Time::now(), "map", "odom"));
    //ROS_INFO("10");
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "icp_slam");
  ros::NodeHandle nh("~");
  std::string pc_topic, template_path;
  base_to_odom.matrix().setIdentity();
  world_to_odom.matrix().setIdentity();

  tf_listener = new tf::TransformListener();

  if (!(nh.getParam("pc_topic", pc_topic))) {
    ROS_ERROR("pc_topic failed to get param");
  }
  
  if (!(nh.getParam("template", template_path))) {
    ROS_ERROR("Didn't get path for full object mesh");
  }

  full_map = boost::make_shared<PointCloudT>();
  built_map = boost::make_shared<PointCloudT>();
  last_scan = boost::make_shared<PointCloudT>();
  if (pcl::io::loadPCDFile<PointT>(template_path, *full_map) == -1) {
     ROS_ERROR_STREAM("Couldn't read full pcd file ");
     return -1;
  }
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*full_map, *full_map, indices);
  pcl::removeNaNNormalsFromPointCloud(*full_map,*full_map, indices);
  pcl::NormalEstimationOMP<PointT, PointT> nest;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  nest.setSearchMethod(tree);
  nest.setKSearch(15);
  nest.setInputCloud(full_map);
  nest.compute(*full_map);
  pcl::removeNaNFromPointCloud(*full_map, *full_map, indices);
  pcl::removeNaNNormalsFromPointCloud(*full_map,*full_map, indices);
  full_map->is_dense=false;

  built_map_pub = nh.advertise<sensor_msgs::PointCloud2>("built_map", 1);
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
  debug_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("debug_cloud", 1);

  ros::Subscriber pc_sub = nh.subscribe(pc_topic.c_str(), 1, pointcloud_callback);

  ros::spin();
}