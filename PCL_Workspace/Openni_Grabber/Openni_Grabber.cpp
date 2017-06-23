#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>


using namespace std;

boost::shared_ptr<pcl::visualization::CloudViewer> viewer (new pcl::visualization::CloudViewer("3D viewer"));

pcl::Grabber* openniGrabber;


void
grabberCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
{

     if (!viewer->wasStopped())
         viewer->showCloud(cloud);
}

/*
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGBA> ());


*cloudOut=*cloud;

for (size_t i=0;i<cloudOut->size();++i)
{
    cloudOut->at(i).y=-cloudOut->at(i).y;
    
    viewer->showCloud(cloudOut);
}
*/



int
main (int argc, char** argv)
{


    openniGrabber = new pcl::OpenNIGrabber();

   if (openniGrabber == 0)
	 return -1;


   boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(&grabberCallback,_1);
//     auto g = [](const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud){};
  //   boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(&g,-1);

		openniGrabber->registerCallback(f);

openniGrabber->start();

while (! viewer->wasStopped())
      boost::this_thread::sleep(boost::posix_time::seconds(1));

}
