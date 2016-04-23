/*

Functions that are used to detect object poses

*/

#ifndef OBJECT_EXTRACTION_H
#define OBJECT_EXTRACTION_H

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/shot.h>
#include <pcl/correspondence.h>

//correspondance search
pcl::CorrespondencesPtr correspondecesearch(pcl::PointCloud<pcl::SHOT352>::Ptr sceneDescriptors, pcl::PointCloud<pcl::SHOT352>::Ptr modelDescriptors);

//ransac rejection and print most plausible transform
void ransac_reject(pcl::CorrespondencesPtr rawcorrespondeces, pcl::PointCloud<pcl::PointXYZ>::Ptr object, pcl::PointCloud<pcl::PointXYZ>::Ptr scene);

//align two pointclouds
pcl::PointCloud<pcl::PointXYZ>::Ptr align(pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::SHOT352>::Ptr scene_features, pcl::PointCloud<pcl::PointXYZ>::Ptr object, pcl::PointCloud<pcl::SHOT352>::Ptr object_features);

#endif