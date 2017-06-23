#include <pcl/io/pcd_io.h>
#include <pcl/recognition/cg/hough_3d.h>
 
#include <iostream>
#include <fstream>
#include <algorithm>
#include <iterator>
 
int
main(int argc, char** argv)
{
	// Objects for storing the keypoints of the scene and the model.
	pcl::PointCloud<pcl::PointXYZ>::Ptr sceneKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr modelKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
	// Objects for storing the Reference Frames.
	pcl::PointCloud<pcl::ReferenceFrame>::Ptr sceneRF(new pcl::PointCloud<pcl::ReferenceFrame>);
	pcl::PointCloud<pcl::ReferenceFrame>::Ptr modelRF(new pcl::PointCloud<pcl::ReferenceFrame>);
	// Objects for storing the unclustered and clustered correspondences.
	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
	std::vector<pcl::Correspondences> clusteredCorrespondences;
	// Object for storing the transformations (rotation plus translation).
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;
 
	// Read the keypoints from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *sceneKeypoints) != 0)
	{
		return -1;
	}

        std::cout << "Found " << sceneKeypoints->size() << " sceneKeypoints." << std::endl;

 
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *modelKeypoints) != 0)
	{
		return -1;
	}

        std::cout << "Found " << modelKeypoints->size() << " modelKeypoints." << std::endl;

	// Read the Reference Frames from disk.
	if (pcl::io::loadPCDFile<pcl::ReferenceFrame>(argv[3], *sceneRF) != 0)
	{
		return -1;
	}

        std::cout << "Found " << sceneRF->size() << " sceneRF." << std::endl;

	if (pcl::io::loadPCDFile<pcl::ReferenceFrame>(argv[4], *modelRF) != 0)
	{
		return -1;
	}
 
        std::cout << "Found " << modelRF->size() << " modelRF." << std::endl;
      

        // Read the Correspondences from disk.

        std::ifstream INFILE(argv[5]);

        std::vector<float> myLines;
        std::copy(std::istream_iterator<float>(INFILE),
        std::istream_iterator<float>(),
        std::back_inserter(myLines)); 


        for (size_t i=0; i<myLines.size(); i+= 3)
        {
               pcl::Correspondence correspondence(static_cast<int>(myLines[i]), static_cast<int>(myLines[i+1]), static_cast<float>(myLines[i+2]));
               correspondences->push_back(correspondence);
        }

        std::cout << "Found " << correspondences->size() << " correspondences." << std::endl;


	// Note: here you would compute the correspondences and the Reference Frames.
	// It has been omitted here for simplicity.
 
	// Object for correspondence grouping.
	pcl::Hough3DGrouping<pcl::PointXYZ, pcl::PointXYZ, pcl::ReferenceFrame, pcl::ReferenceFrame> grouping;
	grouping.setInputCloud(modelKeypoints);
	grouping.setInputRf(modelRF);
	grouping.setSceneCloud(sceneKeypoints);
	grouping.setSceneRf(sceneRF);
	grouping.setModelSceneCorrespondences(correspondences);
	// Minimum cluster size. Default is 3 (as at least 3 correspondences
	// are needed to compute the 6 DoF pose).
	grouping.setHoughThreshold(3);
	// Size of each bin in the Hough space.
	grouping.setHoughBinSize(3);
	// If true, the vote casting procedure will interpolate the score
	// between neighboring bins in the Hough space.
	grouping.setUseInterpolation(true);
	// If true, the vote casting procedure will use the correspondence's
	// weighted distance to compute the Hough voting score.
	grouping.setUseDistanceWeight(false);
 
	grouping.recognize(transformations, clusteredCorrespondences);
 
	std::cout << "Model instances found: " << transformations.size() << std::endl << std::endl;
	for (size_t i = 0; i < transformations.size(); i++)
	{
		std::cout << "Instance " << (i + 1) << ":" << std::endl;
		std::cout << "\tHas " << clusteredCorrespondences[i].size() << " correspondences." << std::endl << std::endl;
 
		Eigen::Matrix3f rotation = transformations[i].block<3, 3>(0, 0);
		Eigen::Vector3f translation = transformations[i].block<3, 1>(0, 3);
		printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		printf("\t\tR = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		std::cout << std::endl;
		printf("\t\tt = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
	} 
}
