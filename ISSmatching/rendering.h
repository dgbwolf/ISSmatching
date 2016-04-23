/*

Functions that are used to render different things.

*/


#ifndef RENDERING_H
#define RENDERING_H

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


//Render Pointcloud and Keypoints
void rendercloudandkeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr inputpointcloud, pcl::PointCloud<pcl::PointXYZ>::Ptr inputkeypoints);

//render pointclouds and correspondances
void rendercorrespondances(pcl::PointCloud<pcl::PointXYZ>::Ptr object, pcl::PointCloud<pcl::PointXYZ>::Ptr object_keypoints, pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints, pcl::CorrespondencesPtr correspondences);

#endif