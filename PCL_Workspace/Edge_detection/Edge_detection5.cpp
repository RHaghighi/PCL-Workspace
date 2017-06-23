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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

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


	// PassThrough Filter
	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setInputCloud (cloud);
	//pass.setFilterFieldName ("x");
	//pass.setFilterLimits (-0.5, 0.5);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-0.7, 0.7);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.5);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered);

	// Voxel Grid Filter 
        pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
        sor.setInputCloud (cloud_filtered);
        sor.setLeafSize (0.008f, 0.008f, 0.008f);
        sor.filter (*cloud_filtered);

	std::cout << "cloud_filtered.size () " << cloud_filtered->size () << std::endl;

	// Outlier Removal Filter
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> outrem;
	// build the filter
	outrem.setInputCloud(cloud_filtered);
	outrem.setMeanK (5);
	outrem.setStddevMulThresh (0.5);
	// apply filter
	outrem.filter (*cloud_filtered);

	std::cout << "cloud_filtered.size () " << cloud_filtered->size () << std::endl;



	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud_filtered);
	normalEstimation.setRadiusSearch(0.03);
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




        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Transformed Cloud"));

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> cloud_h (cloud_filtered1, 0, 0, 255);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::Normal> normal_h (normals, 255, 0, 0);


        viewer->setBackgroundColor (0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZRGBA> (cloud_filtered1, cloud_h, "Transformed Cloud1");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Transformed Cloud1");


	viewer->addPointCloudNormals<pcl::PointXYZRGBA,pcl::Normal> (cloud_filtered1,normals,8,0.05, "Transformed Cloud2");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Transformed Cloud2");




	while (! viewer->wasStopped ())
	{
	viewer->spinOnce ();
	}


return(0);
        
}
