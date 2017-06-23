#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h> 
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/filters/filter.h>
#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/common/common.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// PCL viewer //
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1 (new pcl::visualization::PCLVisualizer ("3D Viewer1"));



PointCloudT::Ptr clicked_points_3d (new PointCloudT);
PointCloudT::Ptr cloud (new PointCloudT);

pcl::visualization::PointCloudColorHandlerCustom<PointT> red (clicked_points_3d, 255, 0, 0);

std::string str;

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

bool
loadCloud (const std::string &filename, PointCloudT &cloud)
{
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  if (loadPCDFile (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void cloud_cb_ (const PointCloudT::ConstPtr &callback_cloud, PointCloudT::Ptr& cloud,
    bool* new_cloud_available_flag)
{
  *cloud = *callback_cloud;
  *new_cloud_available_flag = true;

}


void
pointPickingOccured (const pcl::visualization::PointPickingEvent &eventp, void* args)
{

viewer->removeShape(str);
/*
    float x,y,z;
    eventp.getPoint(x,y,z);
    //pointIndex = event.getPointIndex();
    std::cout <<"Point No. " << eventp.getPointIndex() <<" ";
    std::cout << "X: " << x << " Y: " << y << " Z: " << z << std::endl;

    //viewer.updateSphere(cloud.points[pointIndex], 0.03, 255, 0, 0, "pt");
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
*/

  //viewer->removePointCloud("colorcloud");
  //viewer->removePointCloud ("input_cloud");
  //viewer->addPointCloud (cloud, "input_cloud");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "input_cloud");
  

struct callback_args* data = (struct callback_args *)args;
  if (eventp.getPointIndex () == -1)
    return;
  PointT *current_point (new PointT);
  eventp.getPoint(current_point->x, current_point->y, current_point->z);


 clicked_points_3d->points.push_back(*current_point);
  // Draw clicked points in red:
  //pcl::visualization::PointCloudColorHandlerCustom<PointT> red (clicked_points_3d, 255, 0, 0);

  timeval curTime;
  gettimeofday(&curTime, NULL);
  int milli = curTime.tv_usec / 1000;

  char buffer [80];
  strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec));

  char currentTime[84] = "";
  sprintf(currentTime, "%s:%d", buffer, milli);
  str=currentTime;

  //std::cout << str << "   ";


  //viewer->removePointCloud(str);
  //viewer->addPointCloud(clicked_points_3d, red, "colorcloud");
  //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "colorcloud");
  std::cout << "current point:  "  << current_point->x << " " << current_point->y << " " << current_point->z << std::endl;

  viewer->addSphere (*current_point, 0.001, 255, 0, 0, str);


  //clicked_points_3d->points.pop_back();
  delete current_point;

 // viewer->removePointCloud ("input_cloud");
 // viewer->addPointCloud (cloud, "input_cloud");
 // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "input_cloud");

}



int main (int argc, char** argv)
{

 // Parse the command line arguments for .pcd and .ply files
  std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");

  //PointCloudT::Ptr cloud (new PointCloudT);

  if (!loadCloud (argv[pcd_file_indices[0]], *cloud)) 
    return (-1);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
  std::cout << "cloud->width: " << cloud->width  << "  cloud->height: " << cloud->height  << std::endl; 

  // Display pointcloud:
  viewer->addPointCloud (cloud, "input_cloud");

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "input_cloud");
  viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters ();
  viewer->registerPointPickingCallback(pointPickingOccured, (void*)&viewer);
  viewer->setCameraPosition(0.2, 0.2, 0.2, 0, 0, 1, 0, -1, 0);


   while (!viewer->wasStopped())
  {        

      viewer->spinOnce();

  }


  return 0;
}
