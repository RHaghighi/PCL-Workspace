#include <pcl/io/pcd_io.h>
#include <pcl/features/board.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/uniform_sampling.h>
 
int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Objects for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr sceneNormals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr modelNormals(new pcl::PointCloud<pcl::Normal>);
	// Objects for storing the keypoints.
	pcl::PointCloud<pcl::PointXYZ>::Ptr sceneKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr modelKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
	// Objects for storing the Reference Frames.
	pcl::PointCloud<pcl::ReferenceFrame>::Ptr sceneRF(new pcl::PointCloud<pcl::ReferenceFrame>);
	pcl::PointCloud<pcl::ReferenceFrame>::Ptr modelRF(new pcl::PointCloud<pcl::ReferenceFrame>);
 
	// Read the scene and model clouds from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *sceneCloud) != 0)
	{
		return -1;
	}
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *modelCloud) != 0)
	{
		return -1;
	}

  //
  //  Compute Normals
  //
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
  norm_est.setKSearch (10);
  norm_est.setInputCloud (modelCloud);
  norm_est.compute (*modelNormals);

  norm_est.setInputCloud (sceneCloud);
  norm_est.compute (*sceneNormals);

  //
  //  Downsample Clouds to Extract keypoints
  //

  pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;

  pcl::PointCloud<int> sampled_indices_scene;
  uniform_sampling.setInputCloud (sceneCloud);
  uniform_sampling.setRadiusSearch (0.03f);
  uniform_sampling.compute (sampled_indices_scene);

  pcl::copyPointCloud (*sceneCloud, sampled_indices_scene.points, *sceneKeypoints);

    pcl::io::savePCDFileASCII ("sceneKeypoints.pcd", *sceneKeypoints);

  std::cout << "Scene total points: " << sceneCloud->size () << "; Selected Keypoints: " << sceneKeypoints->size () << std::endl;



  pcl::PointCloud<int> sampled_indices_model;
  uniform_sampling.setInputCloud (modelCloud);
  uniform_sampling.setRadiusSearch (0.01f);
  uniform_sampling.compute (sampled_indices_model);

  pcl::copyPointCloud (*modelCloud, sampled_indices_model.points, *modelKeypoints);

  pcl::io::savePCDFileASCII ("modelKeypoints.pcd", *modelKeypoints);

  std::cout << "Model total points: " << modelCloud->size () << "; Selected Keypoints: " << modelKeypoints->size () << std::endl;


 
	// BOARD RF estimation object.
	pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame> rf;
	// Search radius (maximum distance of the points used to estimate the X and Y axes
	// of the BOARD Reference Frame for a given point).
	rf.setRadiusSearch(0.02f);
	// Check if support is complete, or has missing regions because it is too close to mesh borders.
	rf.setFindHoles(true);

 
	rf.setInputCloud(sceneKeypoints);
	rf.setInputNormals(sceneNormals);
	rf.setSearchSurface(sceneCloud);
	rf.compute(*sceneRF);

        pcl::io::savePCDFileASCII ("scene_RF.pcd", *sceneRF);

        std::cerr << "saved " << sceneRF->size() << " Reference Frames" << std::endl;


         
	rf.setInputCloud(modelKeypoints);
	rf.setInputNormals(modelNormals);
	rf.setSearchSurface(modelCloud);
	rf.compute(*modelRF);

        pcl::io::savePCDFileASCII ("model_RF.pcd", *modelRF);

        std::cerr << "saved " << modelRF->size() << " Reference Frames" << std::endl;


}

