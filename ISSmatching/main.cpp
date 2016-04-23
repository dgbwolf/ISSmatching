//most of the includes are probably not needed anymore here
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/keypoint.h>
#include <pcl/point_cloud.h>
#include <pcl/common/eigen.h>
#include <pcl/point_representation.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/features/shot.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/time.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/features/board.h>

#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <iostream>

#include "keypointextraction.h"
#include "descriptorcomputation.h"
#include "rendering.h"
#include "object_extraction.h"


void main() {

	//load input clouds
	std::cout << "loading scene pointcloud: ";
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile<pcl::PointXYZ>("scene.pcd", *scene);
	std::cout << "done" << "\n";

	std::cout << "loading object pointcloud: ";
	pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile<pcl::PointXYZ>("object.pcd", *object);
	std::cout << "done" << "\n";
	
	// transform one of the clouds, so that there is a transformation even if both clouds are originally aligned
	std::cout << "transform cloud: " << "\n";
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << 0.3, 0, 0;
	transform.rotate(Eigen::AngleAxisf(1.3, Eigen::Vector3f::UnitY()));
	pcl::transformPointCloud(*object, *object, transform);

	//calculate keypoints
	std::cout << "calculating keypoints scene: ";
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	scene_keypoints = downsample_uniform(scene, 0.1);
	std::cout << "calculating keypoints object: ";
	pcl::PointCloud<pcl::PointXYZ>::Ptr object_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	object_keypoints = downsample_uniform(object,0.01);

	//render the clouds and keypoints
	std::cout << "render pointclouds: ";
	rendercloudandkeypoints(scene, scene_keypoints);
	rendercloudandkeypoints(object, object_keypoints);
	std::cout << "done" << "\n";

	//compute descriptors
	std::cout << "compute descpriptors scene: ";
	pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors = computeShotDescriptors(scene_keypoints);
	std::cout << "compute descpriptors object: ";
	pcl::PointCloud<pcl::SHOT352>::Ptr object_descriptors = computeShotDescriptors(object_keypoints);

	//correspondance search
	std::cout << "correspondance search: " << "\n";
	pcl::CorrespondencesPtr correspondences = correspondecesearch(object_descriptors, scene_descriptors);
	std::cout << "render clouds and correspondance: " << "\n";
	rendercorrespondances(object, object_keypoints, scene, scene_keypoints, correspondences);

	//reject outliers using ransac (atm this also outputs the most probable transformation)
	ransac_reject(correspondences, object_keypoints, scene_keypoints);
	
	//don't close console window until input
	std::cin.ignore();
	
}