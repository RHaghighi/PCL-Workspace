/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <pcl/common/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>


typedef pcl::PointXYZRGBA PointT;

double init_time=pcl::getTime();

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_regist (new pcl::PointCloud<pcl::PointXYZRGBA>);

class OpenNIOrganizedEdgeDetection
{
  private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<PointT> cloud_;
    boost::mutex cloud_mutex;
	bool i;

  public:
    OpenNIOrganizedEdgeDetection ()
      : viewer (new pcl::visualization::PCLVisualizer ("PCL Organized Edge Detection"))
    {

    }
    ~OpenNIOrganizedEdgeDetection ()
    {
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer>
    initCloudViewer (pcl::PointCloud<PointT>::ConstPtr cloud)
    {
      viewer->setSize (640, 480);
      viewer->addPointCloud<PointT> (cloud, "cloud");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
      //viewer->addCoordinateSystem (0.2,0, "global");
      viewer->initCameraParameters ();
      //viewer->registerKeyboardCallback (&OpenNIOrganizedEdgeDetection::keyboard_callback, *this);


      viewer->resetCameraViewpoint ("cloud");

      return (viewer);
    }
/*
    void
    keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
    {
      double opacity;  
	  i=true;
          viewer->getPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, "nan boundary edges");
          viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0-opacity, "nan boundary edges");
    }
*/
    void
    cloud_cb_ (const pcl::PointCloud<PointT>::ConstPtr& cloud)
    {
      if (!viewer->wasStopped ())
      {
        cloud_mutex.lock ();
        cloud_ = *cloud;
        cloud_mutex.unlock ();

      double start_time=pcl::getTime();

      if (start_time-init_time<1.5)
         *cloud_regist=*cloud;
        
      }
    }

    void
    run ()
    {
	//i=true;
      pcl::Grabber* interface = new pcl::OpenNIGrabber ();

      boost::function<void(const pcl::PointCloud<PointT>::ConstPtr&)> f = boost::bind (&OpenNIOrganizedEdgeDetection::cloud_cb_, this, _1);

      // Make and initialize a cloud viewer
      pcl::PointCloud<PointT>::Ptr init_cloud_ptr (new pcl::PointCloud<PointT>);
      viewer = initCloudViewer (init_cloud_ptr);
      boost::signals2::connection c = interface->registerCallback (f);

      interface->start ();
      
	pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
	std::vector<int> mapping;
	

	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;

  	
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::VoxelGrid<pcl::PointXYZRGBA> sor;  
        sor.setLeafSize (0.1f, 0.1f, 0.1f);
    


      while (!viewer->wasStopped ())
      {
        viewer->spinOnce ();

        if (cloud_mutex.try_lock ())
        {


            sor.setInputCloud (cloud_.makeShared());
            sor.filter (*cloud_filtered);   

            sor.setInputCloud (cloud_regist);
            sor.filter (*cloud_regist);           
  
            pcl::removeNaNFromPointCloud(*cloud_filtered,*cloud_filtered, mapping);
            pcl::removeNaNFromPointCloud(*cloud_regist,*cloud_regist, mapping);

            for (size_t i=0;i<10;++i)
              {
            icp.setInputSource(cloud_filtered);
            icp.setInputTarget(cloud_regist);
            icp.align(*cloud_filtered);
               }

            *cloud_regist += *cloud_filtered;

         if (!viewer->updatePointCloud (cloud_regist))
           viewer->addPointCloud (cloud_regist);

       
	  
          
          cloud_mutex.unlock ();
        }
      }

      interface->stop ();
    }
};

void
usage (char ** argv)
{
  std::cout << "usage: " << argv[0] << " <device_id> <options>\n\n";

  openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance ();
  if (driver.getNumberDevices () > 0)
  {
    for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
    {
      cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
              << ", connected: " << driver.getBus (deviceIdx) << " @ " << driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << endl;
      cout << "device_id may be #1, #2, ... for the first second etc device in the list or" << endl
           << "                 bus@address for the device connected to a specific usb-bus / address combination (works only in Linux) or" << endl
           << "                 <serial-number> (only in Linux and for devices which provide serial numbers)"  << endl;
    }
  }
  else
    cout << "No devices connected." << endl;
}

int
main (int argc, char ** argv)
{
  std::string arg;
  if (argc > 1)
    arg = std::string (argv[1]);
  
  if (arg == "--help" || arg == "-h")
  {
    usage (argv);
    return 1;
  }
  
  std::cout << "Press <1> to re-grab the point cloud." << std::endl;
  std::cout << "<Q,q> quit" << std::endl;


  pcl::OpenNIGrabber grabber ("");
  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgb> ())
  {
    OpenNIOrganizedEdgeDetection app;
    app.run ();
  }
  else
    PCL_ERROR ("The input device does not provide a PointT mode.\n");

  return (0);
}
