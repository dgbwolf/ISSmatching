/*

Functions that are used to extract keypoints

*/

#ifndef KEYPOINTEXTRACTION_H
#define KEYPOINTEXTRACTION_H


#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//Uniform downsampling (no evaluation regarding keypoint quality)
pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_uniform(pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud, double uniform_radius);

//Voxel downsampling  (no evaluation regarding keypoint quality)
pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud, float leafsize = 0.005f);

//Compute ISS keypoints
pcl::PointCloud<pcl::PointXYZ>::Ptr computeIssKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud);


#endif