#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>

#include <iostream>

#include <fstream>
#include <algorithm>
#include <iterator>
 
int
main(int argc, char** argv)
{
	// Object for storing the SHOT descriptors for the scene.
	pcl::PointCloud<pcl::SHOT352>::Ptr sceneDescriptors(new pcl::PointCloud<pcl::SHOT352>());
	// Object for storing the SHOT descriptors for the model.
	pcl::PointCloud<pcl::SHOT352>::Ptr modelDescriptors(new pcl::PointCloud<pcl::SHOT352>());
 
	// Read the already computed descriptors from disk.
	if (pcl::io::loadPCDFile<pcl::SHOT352>(argv[1], *sceneDescriptors) != 0)
	{
		return -1;
	}
	if (pcl::io::loadPCDFile<pcl::SHOT352>(argv[2], *modelDescriptors) != 0)
	{
		return -1;
	}
 
	// A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
	pcl::KdTreeFLANN<pcl::SHOT352> matching;
	matching.setInputCloud(modelDescriptors);
	// A Correspondence object stores the indices of the query and the match,
	// and the distance/weight.
	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
 
	// Check every descriptor computed for the scene.
	//for (size_t i = 0; i <seneDescriptors->size() ; ++i)

std::vector<float> Corresp;

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

Corresp.push_back(neighbors[0]);
Corresp.push_back(i);
Corresp.push_back(squaredDistances[0]);


			}
		}
	}


	std::cout << "Found " << correspondences->size() << " correspondences." << std::endl;



        std::ofstream FILE("Correspondences.txt");

        for (size_t i=0; i<3*correspondences->size(); i+= 3)
        {
              FILE << Corresp[i] << std::endl;
              FILE << Corresp[i+1] << std::endl;
              FILE << Corresp[i+2] << std::endl;
        }

/*

std::ifstream INFILE("saveFile.txt");

std::vector<float> myLines;
std::copy(std::istream_iterator<float>(INFILE),
          std::istream_iterator<float>(),
          std::back_inserter(myLines)); 


pcl::CorrespondencesPtr correspondences_New(new pcl::Correspondences());

for (size_t i=0; i<3*correspondences->size(); i+= 3)
{
pcl::Correspondence correspondence(static_cast<int>(myLines[i]), static_cast<int>(myLines[i+1]), static_cast<float>(myLines[i+2]));
correspondences_New->push_back(correspondence);
}

*/


}
