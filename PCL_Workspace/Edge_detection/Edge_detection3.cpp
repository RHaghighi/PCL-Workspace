#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>


int
main(int argc, char** argv)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	std::cout << "cloud.size () " << cloud->size () << std::endl;
 
	// Note: you would usually perform downsampling now. It has been omitted here
	// for simplicity, but be aware that computation can take a long 


        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
 
        pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (0.02f, 0.02f, 0.02f);
        sor.filter (*cloud_filtered);

	std::cout << "cloud_filtered.size () " << cloud_filtered->size () << std::endl;

/*
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGBA> outrem;
	// build the filter
	outrem.setInputCloud(cloud_filtered);
	outrem.setRadiusSearch(0.08);
	outrem.setMinNeighborsInRadius (.05);
	// apply filter
	outrem.filter (*cloud_filtered);

	std::cout << "cloud_filtered.size () " << cloud_filtered->size () << std::endl;
*/


	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud_filtered);
	normalEstimation.setRadiusSearch(0.035);
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	std::vector<int> index;

	pcl::removeNaNNormalsFromPointCloud (*normals, *normals, index);



	for (size_t i = 0; i < normals->size(); ++i)
		 cloud_filtered1->points.push_back (cloud_filtered->points[index.at(i)]); 

	cloud_filtered1->width = cloud_filtered1->points.size ();
	cloud_filtered1->height = 1;
	cloud_filtered1->is_dense = true;

	std::cout << "cloud_filtered1.size () " << cloud_filtered1->size () << std::endl;

	pcl::search::Search<pcl::PointXYZRGBA>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBA> > (new pcl::search::KdTree<pcl::PointXYZRGBA>);
	pcl::RegionGrowing<pcl::PointXYZRGBA, pcl::Normal> reg;
	reg.setMinClusterSize (30);
	reg.setMaxClusterSize (1000000);
	reg.setSearchMethod (tree);
	reg.setNumberOfNeighbours (20);
	reg.setInputCloud (cloud_filtered1);
	//reg.setIndices (indices);
	reg.setInputNormals (normals);
	reg.setSmoothnessThreshold (4.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold (1.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract (clusters);

	std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;



	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
	pcl::visualization::CloudViewer viewer1 ("Cluster viewer");
	viewer1.showCloud(colored_cloud);

	while (!viewer1.wasStopped ())
	{
	}


/*
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Transformed Cloud"));

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> cloud_h (cloud_filtered, 0, 0, 255);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> segment_h (segment, 255, 0, 0);


        viewer->setBackgroundColor (0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZRGBA> (cloud_filtered, cloud_h, "Transformed Cloud1");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.3, "Transformed Cloud1");


	viewer->addPointCloud<pcl::PointXYZRGBA> (segment,segment_h, "Transformed Cloud2");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Transformed Cloud2");



	while (! viewer->wasStopped ())
	{
	viewer->spinOnce ();
	}
*/

return(0);
        
}
