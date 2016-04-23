#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/eigen.h>
#include <pcl/point_representation.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/common/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/uniform_sampling.h>

#include <iostream>


//Uniform downsampling
pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_uniform(pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud, double uniform_radius) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
	uniform_sampling.setInputCloud(inputcloud);
	uniform_sampling.setRadiusSearch(uniform_radius);
	//Downsampling and time measurement
	{
		pcl::ScopeTime t("Downsampling - uniform");
		uniform_sampling.filter(*cloud_keypoints);
	}
	std::cout << "Model total points: " << inputcloud->size() << "; Selected Keypoints: " << cloud_keypoints->size() << std::endl;

	return cloud_keypoints;


}

//Voxel downsampling  
pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud, float leafsize = 0.005f) {
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	grid.setLeafSize(leafsize, leafsize, leafsize);
	grid.setInputCloud(inputcloud);

	//Downsampling and time measurement
	{
		pcl::ScopeTime t("Downsampling - voxel");
		grid.filter(*cloud_keypoints);
	}
	std::cout << "Model total points: " << inputcloud->size() << "; Selected Keypoints: " << cloud_keypoints->size() << std::endl;

	return cloud_keypoints;
}

double _computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!pcl_isfinite((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}


//Compute iss keypoints
pcl::PointCloud<pcl::PointXYZ>::Ptr computeIssKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud) {
	double iss_salient_radius_;
	double iss_non_max_radius_;

	double iss_normal_radius_;
	double iss_border_radius_;

	double iss_gamma_21_(0.3);
	double iss_gamma_32_(0.9);
	double iss_min_neighbors_(5);
	int iss_threads_(4);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());


	double model_resolution = _computeCloudResolution(inputcloud);
	std::cout << "model resolution: " << model_resolution << "\n";


	iss_salient_radius_ = 6 * model_resolution;
	iss_non_max_radius_ = 4 * model_resolution;
	iss_normal_radius_ = 4 * model_resolution;
	iss_border_radius_ = 1 * model_resolution;

	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;

	iss_detector.setSearchMethod(tree);
	iss_detector.setSalientRadius(iss_salient_radius_);
	iss_detector.setNonMaxRadius(iss_non_max_radius_);

	iss_detector.setNormalRadius(iss_normal_radius_);
	iss_detector.setBorderRadius(iss_border_radius_);

	iss_detector.setThreshold21(iss_gamma_21_);
	iss_detector.setThreshold32(iss_gamma_32_);
	iss_detector.setMinNeighbors(iss_min_neighbors_);
	iss_detector.setNumberOfThreads(iss_threads_);
	iss_detector.setInputCloud(inputcloud);
	//Keypointdetection and time measurement
	{
		pcl::ScopeTime t("Keypoint Search");
		iss_detector.compute(*cloud_keypoints);
	}

	std::cout << "Model total points: " << inputcloud->size() << "; Selected Keypoints: " << cloud_keypoints->size() << std::endl;
	return cloud_keypoints;
}