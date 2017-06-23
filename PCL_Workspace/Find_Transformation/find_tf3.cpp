#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/registration/icp.h>

int 
  main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_tf1 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_tf2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_modified (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_modified (new pcl::PointCloud<pcl::PointXYZ>);

  // Read PCD files from disk.
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *model) != 0)
    {
	return -1;
    }
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[2], *scene) != 0)
   {
	return -1;
   }

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[3], *model_modified) != 0)
    {
	return -1;
    }
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[4], *scene_modified) != 0)
   {
	return -1;
   }


  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setLeafSize (0.01f, 0.01f, 0.01f);


  sor.setInputCloud (model);
  sor.filter (*model);

  sor.setInputCloud (scene);
  sor.filter (*scene);


  Eigen::Matrix4f Transform_Model = Eigen::Matrix4f::Identity();

Transform_Model (0,0) = 0.716639;
Transform_Model (0,1) = -0.298942;
Transform_Model (0,2) = 0.630129;
Transform_Model (0,3) = -0.723870;
Transform_Model (1,0) = -0.617316;
Transform_Model (1,1) = 0.148582;
Transform_Model (1,2) = 0.772557;
Transform_Model (1,3) = -0.739746;
Transform_Model (2,0) = 0.324576;
Transform_Model (2,1) = 0.942633;
Transform_Model (2,2) = 0.078063;
Transform_Model (2,3) = 0.931368;
Transform_Model (3,0) = 0.000000;
Transform_Model (3,1) = 0.000000;
Transform_Model (3,2) = 0.000000;
Transform_Model (3,3) = 1.000000;

  Eigen::Vector3f point1_model(0.0690597,0.121807,0.726103);
  Eigen::Vector3f point2_model(0.0680728,0.0246748,0.637091);
  //Eigen::Vector3f point3_model(0.0164271,-0.107744,0.751862);


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  cloud->width = 2;
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);

  cloud->points[0].x = point1_model(0);
  cloud->points[0].y = point1_model(1);
  cloud->points[0].z = point1_model(2);

  cloud->points[1].x = point2_model(0);
  cloud->points[1].y = point2_model(1);
  cloud->points[1].z = point2_model(2);

  //cloud->points[2].x = point3_model(0);
  //cloud->points[2].y = point3_model(1);
  //cloud->points[2].z = point3_model(2);


  Eigen::Vector3f point1_scene(-0.227579,-0.166513,1.08712);
  Eigen::Vector3f point2_scene(-0.213298,-0.180041,0.954568);
  //Eigen::Vector3f point3_scene(-0.299508,-0.327723,1.00388);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);

  cloud1->width = 2;
  cloud1->height = 1;
  cloud1->is_dense = false;
  cloud1->points.resize (cloud1->width * cloud1->height);

  cloud1->points[0].x = point1_scene(0);
  cloud1->points[0].y = point1_scene(1);
  cloud1->points[0].z = point1_scene(2);

  cloud1->points[1].x = point2_scene(0);
  cloud1->points[1].y = point2_scene(1);
  cloud1->points[1].z = point2_scene(2);

  //cloud1->points[2].x = point3_scene(0);
  //cloud1->points[2].y = point3_scene(1);
  //cloud1->points[2].z = point3_scene(2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudf  (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::transformPointCloud (*cloud, *cloudf, Transform_Model);

  pcl::transformPointCloud (*model, *model_tf1, Transform_Model);

 //pcl::transformPointCloud (*model_modified, *model_modified, Transform_Model);



  // ICP object.
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> reg;
  //reg.setTransformationEpsilon (1e-12);
  //reg.setMaxCorrespondenceDistance (0.1);
  //reg.setMaximumIterations (100000);

  reg.setInputSource(model_modified);
  reg.setInputTarget(scene_modified);

  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity ();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src  (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr reg_result  = model_modified;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt  = scene_modified;
  
  for (int i = 0; i < 10; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    *cloud_src = *reg_result;

    // Estimate
    reg.setInputSource (cloud_src);
    reg.align (*reg_result);

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

std::cout  << reg.getFinalTransformation ()  << std::endl;

  }


  //pcl::transformPointCloud (*model_tf1, *model_tf1, Ti);


  std::cout << "Ti = "  << Ti  <<  std::endl; 


  /// VISUALIZATION Clouds

  boost::shared_ptr<pcl::visualization::PCLVisualizer> model_viewer (new pcl::visualization::PCLVisualizer ("model Cluster viewer"));
  boost::shared_ptr<pcl::visualization::PCLVisualizer> scene_viewer (new pcl::visualization::PCLVisualizer ("scene Cluster viewer"));

  //boost::shared_ptr<pcl::visualization::PCLVisualizer> model_transformed (new pcl::visualization::PCLVisualizer ("model Cluster viewer"));

  model_viewer->setBackgroundColor (255, 255, 255);
  scene_viewer->setBackgroundColor (255, 255, 255);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> model_rgb(model, 0, 0, 255);
  model_viewer->addPointCloud<pcl::PointXYZRGB> (model, model_rgb, "model Cluster viewer");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> scene_rgb(scene, 0, 255, 0);
  scene_viewer->addPointCloud<pcl::PointXYZRGB> (scene, scene_rgb, "scene Cluster viewer");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> model_tf1_rgb(model_tf1, 255, 0, 0);
  scene_viewer->addPointCloud<pcl::PointXYZRGB> (model_tf1, model_tf1_rgb,"model Cluster viewer0");


  scene_viewer->addPointCloud<pcl::PointXYZRGB> (model, model_rgb, "model Cluster viewer1");


  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_rgb(cloud, 0, 255, 0);
  scene_viewer->addPointCloud<pcl::PointXYZ> (cloud, cloud_rgb, "model Cluster viewer5");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud1_rgb(cloud1, 255, 0, 0);
  scene_viewer->addPointCloud<pcl::PointXYZ> (cloud1, cloud1_rgb, "scene Cluster viewer5");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloudf_rgb(cloudf, 0, 0, 255);
  scene_viewer->addPointCloud<pcl::PointXYZ> (cloudf, cloudf_rgb, "cloudf Cluster viewer5");

  model_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "model Cluster viewer");
  scene_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "model Cluster viewer0");
  scene_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "model Cluster viewer1");

  scene_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene Cluster viewer");

  //scene_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model Cluster viewer5");
  //scene_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene Cluster viewer5");
  //scene_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloudf Cluster viewer5");
 

  while (!model_viewer->wasStopped ())
     {
	model_viewer->spinOnce (100);
	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
     }

 return(0);
}
