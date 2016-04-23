
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <iostream>

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


//render pointclouds and correspondances
void rendercorrespondances(pcl::PointCloud<pcl::PointXYZ>::Ptr object, pcl::PointCloud<pcl::PointXYZ>::Ptr object_keypoints, pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints,pcl::CorrespondencesPtr correspondences) {

	boost::shared_ptr<pcl::visualization::PCLVisualizer> correspondence_viewer(new pcl::visualization::PCLVisualizer());
	correspondence_viewer->setBackgroundColor(255, 255, 255);
	correspondence_viewer->addPointCloud(object, "object");
	correspondence_viewer->addPointCloud(scene, "scene");
	correspondence_viewer->addPointCloud(object_keypoints, "object_keypoints");
	correspondence_viewer->addPointCloud(scene_keypoints, "scene_keypoints");
	correspondence_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "object");
	correspondence_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene");
	correspondence_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (double)0, (double)0, (double)0, "object");
	correspondence_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (double)0, (double)0, (double)1, "scene");
	correspondence_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "object_keypoints");
	correspondence_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "scene_keypoints");
	correspondence_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (double)1, (double)0, (double)0, "object_keypoints");
	correspondence_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, (double)1, (double)0, (double)0, "scene_keypoints");
	//correspondence_viewer->addCorrespondences<pcl::PointXYZ>(object, scene, *correspondences);

	for (size_t j = 0; j < correspondences->size(); ++j)
	{
		std::stringstream ss_line;
		ss_line << "correspondence_line" << j;
		//std::cout << "draw line: " << j << "between:" << object->points.at((*correspondences)[j].index_query) << "and " << scene->points.at((*correspondences)[j].index_match)<<"\n";
	
		pcl::PointXYZ& model_point = scene_keypoints->points.at((*correspondences)[j].index_query);
		pcl::PointXYZ& scene_point = object_keypoints->points.at((*correspondences)[j].index_match);

		//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
		correspondence_viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(model_point, scene_point, 0, 255, 0, ss_line.str());
	}
	


	correspondence_viewer->initCameraParameters();
	while (!correspondence_viewer->wasStopped())
	{
		correspondence_viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}
