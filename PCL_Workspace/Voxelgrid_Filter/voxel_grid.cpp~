#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;


int
main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);

  pcl::visualization::PCLVisualizer viewer ("PCL Visualizer");

  pcl::visualization::PCLVisualizer viewer2 ("Cloud Viewer");
  

  cout << "Enter File Name:";
  string filename;
  cin >> filename; 

  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (filename, *cloud)==-1)
  {
   PCL_ERROR("Couldn't read file");
   return(-1);
  }

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

 viewer.addPointCloud (cloud,"cloud");


  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << endl;
 
  string filename1;
  string exten = ".pcd";
  filename1 =filename;
  filename1.replace(filename1.find(".pcd"),exten.length(),"_downsampled.pcd"); 

  pcl::io::savePCDFileASCII (filename1, *cloud_filtered);

  viewer2.addPointCloud (cloud_filtered, "cloud");

  while (! viewer.wasStopped ())
  {
   viewer.spinOnce ();
  }

  return (0);
}
