/*

Functions that are used to compute 3D descriptors

*/

#ifndef DESCRIPTORCOMPUTATION_H
#define DESCRIPTORCOMPUTATION_H

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


//compute shot descriptors
pcl::PointCloud<pcl::SHOT352>::Ptr computeShotDescriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr inputkeypoints, double normal_search_radius = 50, double shot_search_radius = 50);

#endif