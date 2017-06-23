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
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1 (new pcl::visualization::PCLVisualizer ("3D Viewer1"));



PointCloudT::Ptr clicked_points_3d (new PointCloudT);
PointCloudT::Ptr cloud (new PointCloudT);

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

//struct callback_args{
  // structure used to pass arguments to the callback function
 // PointCloudT::Ptr clicked_points_3d;
 // pcl::visualization::PCLVisualizer::Ptr viewerPtr;
//};

/*
void mouseEventOccurred(const pcl::visualization::MouseEvent &eventm, void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);

    if (eventm.getButton() == pcl::visualization::MouseEvent::LeftButton  &&
      eventm.getType() == pcl::visualization::MouseEvent::MouseButtonPress)
      std::cout << "Left mouse button pressed at position (" << eventm.getX() << ", " << eventm.getY() << ")" << std::endl;
 
}
*/

void
pointPickingOccured (const pcl::visualization::PointPickingEvent &eventp, void* args)
{

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

  struct callback_args* data = (struct callback_args *)args;
  if (eventp.getPointIndex () == -1)
    return;
  PointT *current_point (new PointT);
  eventp.getPoint(current_point->x, current_point->y, current_point->z);


 clicked_points_3d->points.push_back(*current_point);
  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (clicked_points_3d, 255, 0, 0);

  timeval curTime;
  gettimeofday(&curTime, NULL);
  int milli = curTime.tv_usec / 1000;

  char buffer [80];
  strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec));

  char currentTime[84] = "";
  sprintf(currentTime, "%s:%d", buffer, milli);
  std::string str(currentTime);

  //std::cout << str << "   ";


  //viewer->removePointCloud(str);
  viewer1->addPointCloud(clicked_points_3d, red, str);
  viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, str);
  std::cout << "current point:  "  << current_point->x << " " << current_point->y << " " << current_point->z << std::endl;

  delete current_point;

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
  //pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);

      viewer->setSize (450, 450);
      viewer1->setSize (450, 450);
  viewer->addPointCloud (cloud, "input_cloud");

  
/*
  // Add point picking callback to viewer:
  struct callback_args cb_args;
  PointCloudT::Ptr clicked_points_3d (new PointCloudT);
  cb_args.clicked_points_3d = clicked_points_3d;
  cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer);
  viewer.registerPointPickingCallback (pointPickingOccured, (void*)&cb_args);
*/

  viewer1->addPointCloud (cloud, "input_cloud1");
  viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input_cloud1");
  viewer1->addCoordinateSystem (0.1);
  viewer1->initCameraParameters ();
  viewer1->setCameraPosition(0.2, 0.2, 0.2, 0, 0, 1, 0, -1, 0);

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "input_cloud");
  viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters ();
  //viewer.registerMouseCallback (mouseEventOccurred, (void*)&viewer);
  viewer->registerPointPickingCallback(pointPickingOccured, (void*)&viewer);
  viewer->setCameraPosition(0.2, 0.2, 0.2, 0, 0, 1, 0, -1, 0);

  std::vector<pcl::visualization::Camera> param;

  //std::cout << param;


   while (!viewer->wasStopped())
  {
      viewer->spinOnce();
      viewer1->spinOnce();
      viewer->setSize (450, 450);
      viewer1->setSize (450, 450);

      viewer->getCameras(param);
      viewer1->setCameraPosition(param[0].pos[0], param[0].pos[1], param[0].pos[2], 0,0,1,param[0].view[0], param[0].view[1], param[0].view[2]);
  }


  return 0;
}
