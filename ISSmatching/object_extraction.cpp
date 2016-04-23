#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/common/time.h>



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

//ransac rejection and print most plausible transform
void ransac_reject(pcl::CorrespondencesPtr rawcorrespondeces, pcl::PointCloud<pcl::PointXYZ>::Ptr object, pcl::PointCloud<pcl::PointXYZ>::Ptr scene) {

	pcl::CorrespondencesPtr filteredcorrespondences(new pcl::Correspondences());
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejecter;
	rejecter.setInputSource(scene);
	rejecter.setInputTarget(object);
	rejecter.setInputCorrespondences(rawcorrespondeces);
	rejecter.setMaximumIterations(500);
	rejecter.setInlierThreshold(2);
	rejecter.setRefineModel(false);
	rejecter.getRemainingCorrespondences(*rawcorrespondeces, *filteredcorrespondences);
	Eigen::Matrix4f transformation = rejecter.getBestTransformation();
	pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
	pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
	pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
	pcl::console::print_info("\n");
	pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
	pcl::console::print_info("\n");
}

//align two pointclouds
pcl::PointCloud<pcl::PointXYZ>::Ptr align(pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::SHOT352>::Ptr scene_features, pcl::PointCloud<pcl::PointXYZ>::Ptr object, pcl::PointCloud<pcl::SHOT352>::Ptr object_features) {


	pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::SHOT352> align;
	pcl::PointCloud<pcl::PointXYZ>::Ptr object_aligned(new pcl::PointCloud<pcl::PointXYZ>());

	align.setInputSource(object);
	align.setSourceFeatures(object_features);
	align.setInputTarget(scene);
	align.setTargetFeatures(scene_features);
	align.setMaximumIterations(1000); // Number of RANSAC iterations
	align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
	align.setCorrespondenceRandomness(5); // Number of nearest features to use
	align.setSimilarityThreshold(0.5f); // Polygonal edge length similarity threshold
	align.setMaxCorrespondenceDistance(0.08f); // Inlier threshold
	align.setInlierFraction(0.25f); // Required inlier fraction for accepting a pose hypothesis
	//perform alignment and measure time
	{
		pcl::ScopeTime t("Alignment");
		align.align(*object_aligned);
	}

	// Print results
	printf("\n");
	Eigen::Matrix4f transformation = align.getFinalTransformation();
	pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
	pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
	pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
	pcl::console::print_info("\n");
	pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
	pcl::console::print_info("\n");
	pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), object->size());

	return object_aligned;
}
