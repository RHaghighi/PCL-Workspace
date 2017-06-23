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

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);

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

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = (int) cloud_filter->points.size ();

float thresh=0.005;

  // While 30% of the original cloud is still there
  while (cloud_filter->points.size () > 0.17 * nr_points)
  {

    seg.setDistanceThreshold (thresh);
    seg.setInputCloud (cloud_filter);
    seg.segment (*inliers, *coefficients);

    thresh=thresh-0.001;

    // Extract the inliers
    extract.setInputCloud (cloud_filter);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);

    if (cloud_p->size()==0)
       break;

    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;


    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filter.swap (cloud_f);
    i++;

  }

  // Visualization of both point clouds
  pcl::visualization::PCLVisualizer viewer2;
  pcl::visualization::PCLVisualizer viewer3;

  viewer2.addPointCloud(cloud_p);
  viewer3.addPointCloud(cloud_filter);

while (! viewer.wasStopped ())
{
   viewer.spinOnce();
}
  return (0);
}
