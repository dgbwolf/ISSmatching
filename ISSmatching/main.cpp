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

#include <iostream>
// compute Point Cloud resolution
double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
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

//compute shot descriptors
pcl::PointCloud<pcl::SHOT352>::Ptr computeShotDescriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr inputkeypoints) {

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;

	normalEstimation.setInputCloud(inputkeypoints);
	normalEstimation.setRadiusSearch(0.3);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
	shot.setInputCloud(inputkeypoints);
	shot.setInputNormals(normals);

	shot.setRadiusSearch(0.3);
	shot.compute(*descriptors);

	return descriptors;
}

//Compute iss keypoints
pcl::PointCloud<pcl::PointXYZ>::Ptr computeIssKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr inputpointcloud) {
	double iss_salient_radius_;
	double iss_non_max_radius_;

	double iss_normal_radius_;
	double iss_border_radius_;

	double iss_gamma_21_(0.975);
	double iss_gamma_32_(0.975);
	double iss_min_neighbors_(5);
	int iss_threads_(4);

	pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());


	double model_resolution = computeCloudResolution(inputpointcloud);
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
	iss_detector.setInputCloud(inputpointcloud);
	iss_detector.compute(*model_keypoints);

	return model_keypoints;
}

//Render Pointcloud and Keypoints
void rendercloudandkeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr inputpointcloud, pcl::PointCloud<pcl::PointXYZ>::Ptr inputkeypoints) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer());
	viewer->setBackgroundColor(255, 255, 255);
	viewer->addPointCloud(inputpointcloud, "model");
	viewer->addPointCloud(inputkeypoints, "keypoints");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "keypoints");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (double)0, (double)0, (double)0, "model");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (double)255, (double)0, (double)0, "keypoints");
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	
}

//correspondance search
pcl::CorrespondencesPtr correspondecesearch(pcl::PointCloud<pcl::SHOT352>::Ptr sceneDescriptors, pcl::PointCloud<pcl::SHOT352>::Ptr modelDescriptors) {



	// A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
	pcl::KdTreeFLANN<pcl::SHOT352> matching;
	matching.setInputCloud(modelDescriptors);
	// A Correspondence object stores the indices of the query and the match,
	// and the distance/weight.
	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());

	// Check every descriptor computed for the scene.
	for (size_t i = 0; i < sceneDescriptors->size(); ++i)
	{
		std::vector<int> neighbors(1);
		std::vector<float> squaredDistances(1);
		// Ignore NaNs.
		if (pcl_isfinite(sceneDescriptors->at(i).descriptor[0]))
		{
			// Find the nearest neighbor (in descriptor space)...
			int neighborCount = matching.nearestKSearch(sceneDescriptors->at(i), 1, neighbors, squaredDistances);
			// ...and add a new correspondence if the distance is less than a threshold
			// (SHOT distances are between 0 and 1, other descriptors use different metrics).
			if (neighborCount == 1 && squaredDistances[0] < 0.25f)
			{
				pcl::Correspondence correspondence(neighbors[0], static_cast<int>(i), squaredDistances[0]);
				correspondences->push_back(correspondence);
			}
		}
	}
	std::cout << "Found " << correspondences->size() << " correspondences." << std::endl;

	return correspondences;
}

//render pointclouds and correspondances
void rendercorrespondances(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, pcl::CorrespondencesPtr correspondences) {
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> correspondence_viewer(new pcl::visualization::PCLVisualizer());
	correspondence_viewer->setBackgroundColor(255, 255, 255);
	correspondence_viewer->addPointCloud(cloud1, "cloud1");
	correspondence_viewer->addPointCloud(cloud2, "cloud2");
	correspondence_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud1");
	correspondence_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud2");
	correspondence_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (double)0, (double)0, (double)0, "cloud1");
	correspondence_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (double)0, (double)255, (double)0, "cloud2");
	
	correspondence_viewer->addCorrespondences<pcl::PointXYZ>(cloud1, cloud2, *correspondences);
	correspondence_viewer->initCameraParameters();
	while (!correspondence_viewer->wasStopped())
	{
		correspondence_viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}
void main() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>());
	std::cout << "loading pointcloud: " << "\n";
	pcl::io::loadPCDFile<pcl::PointXYZ>("input.pcd", *model);

	std::cout << "compute keypoints: " << "\n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints = computeIssKeypoints(model);

	std::cout << "save keypoints: " << "\n";
	pcl::io::savePCDFileASCII("keypoints.pcd", *model_keypoints);

	std::cout << "render cloud and keypoints: " << "\n";
	rendercloudandkeypoints(model, model_keypoints);

	std::cout << "compute descriptors: " << "\n";
	pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors = computeShotDescriptors(model_keypoints);

	std::cout << "transform cloud: " << "\n";
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << 0.5, 0, 0;
	transform.rotate(Eigen::AngleAxisf(0.3, Eigen::Vector3f::UnitZ()));

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*model, *transformed_cloud, transform);

	std::cout << "compute transformed keypoints: " << "\n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_transformed_keypoints = computeIssKeypoints(transformed_cloud);
	rendercloudandkeypoints(transformed_cloud, model_transformed_keypoints);

	std::cout << "compute transformed descriptors " << "\n";
	pcl::PointCloud<pcl::SHOT352>::Ptr model_transformed_descriptors = computeShotDescriptors(model_transformed_keypoints);

	std::cout << "correspondance search: " << "\n";
	pcl::CorrespondencesPtr correspondences = correspondecesearch(model_descriptors, model_transformed_descriptors);

	std::cout << "render clouds and correspondance: " << "\n";
	rendercorrespondances(model, transformed_cloud, correspondences);

	//don't close console window until input
	std::cin.ignore();
}