#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/shot.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/time.h>


//compute shot descriptors
pcl::PointCloud<pcl::SHOT352>::Ptr computeShotDescriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr inputkeypoints, double normal_search_radius = 50, double shot_search_radius = 50) {

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;

	normalEstimation.setInputCloud(inputkeypoints);
	normalEstimation.setRadiusSearch(normal_search_radius);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
	shot.setInputCloud(inputkeypoints);
	shot.setInputNormals(normals);
	shot.setRadiusSearch(shot_search_radius);
	//Compute descriptors and meassure time
	{
		pcl::ScopeTime t("Descriptor Computation");
		shot.compute(*descriptors);
	}
	return descriptors;
}
