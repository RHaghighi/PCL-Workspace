#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) !=0)
{
  PCL_ERROR("Couldn't Find the File");
  return(-1);
}  

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filter);
  
  std::cerr << "Point cloud data: " << cloud_filter->points.size () << " points" << std::endl;


  ////////
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;


  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filter);
  ne.setKSearch (50);
  ne.compute (*normals);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg; 
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.5);
  seg.setMaxIterations (100000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.2);

/*
  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }


  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

*/

 //Visualization of Original Cloud
  pcl::visualization::PCLVisualizer viewer;

  viewer.addPointCloud(cloud_filter);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>), cloud_l (new pcl::PointCloud<pcl::PointXYZ>);


pcl::PointCloud<pcl::Normal>::Ptr normals_p (new pcl::PointCloud<pcl::Normal>), normals_f (new pcl::PointCloud<pcl::Normal>);


    // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = (int) cloud_filter->points.size ();



  // While 30% of the original cloud is still there
  while (cloud_filter->points.size () > 0.17 * nr_points)
  {

 
    seg.setInputCloud (cloud_filter);
    seg.setInputNormals (normals);
    seg.segment (*inliers, *coefficients);


    // Extract the inliers
    extract.setInputCloud (cloud_filter);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    *cloud_l += *cloud_p;

    // Extract the inliers
    extract_normals.setInputCloud (normals);
    extract_normals.setIndices (inliers);
    extract_normals.setNegative (false);
    extract_normals.filter (*normals_p);

    if (cloud_p->size()<20)
       break;

    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;


    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filter.swap (cloud_f);

    // Create the filtering object
    extract_normals.setNegative (true);
    extract_normals.filter (*normals_f);
    normals.swap (normals_f);

    i++;

  }

  // Visualization of both point clouds
  pcl::visualization::PCLVisualizer viewer2;
  pcl::visualization::PCLVisualizer viewer3;

  viewer2.addPointCloud(cloud_l);
  viewer3.addPointCloud(cloud_filter);

while (! viewer.wasStopped ())
{
   viewer.spinOnce();
}


  return (0);
}
