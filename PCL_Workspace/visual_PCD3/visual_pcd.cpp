#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


int 
main (int argc, char** argv) 
{ 

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>); 

        // Visualization 
        //pcl::visualization::PCLVisualizer viewer ("PCL visualizer"); 
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")); 
        
 // Fill in the cloud data 
        cloud->width = 500; 
        cloud->height = 1; 
        cloud->is_dense = false; 
        cloud->points.resize (cloud->width * cloud->height); 

        for (size_t i = 0; i < cloud->points.size (); ++i){ 
                cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f); 
                cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f); 
                cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f); 

                //  Random color for each point 
                cloud->points[i].r = 255 *(1024 * rand () / (RAND_MAX + 1.0f)); 
                cloud->points[i].g = 255 *(1024 * rand () / (RAND_MAX + 1.0f)); 
                cloud->points[i].b = 255 *(1024 * rand () / (RAND_MAX + 1.0f)); 
        } 

        viewer->addPointCloud (cloud, "cloud"); 


        viewer->addCoordinateSystem(1.0, 0, 0, 0, 0); 
        viewer->setBackgroundColor (0.05, 0.05, 0.05, 0);	// Setting background to a dark grey 
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud"); 

        while (!viewer->wasStopped ()) { 
                viewer->spinOnce (); 
        } 
        return (0); 
}
