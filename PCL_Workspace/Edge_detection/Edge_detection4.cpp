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
#include <math.h>
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
 
        pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
        vg.setInputCloud (cloud);
        vg.setLeafSize (0.008f, 0.008f, 0.008f);
        vg.filter (*cloud_filtered);

	std::cout << "cloud_filtered.size () " << cloud_filtered->size () << std::endl;



	// Statistical Outlier Removal
	pcl::StatisticalOutlierRemoval <pcl::PointXYZRGBA> sor;
	sor.setInputCloud (cloud_filtered);
	sor.setMeanK (50);	
	sor.setStddevMulThresh (1.0);
	sor.filter (*cloud_filtered);

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

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segment(new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtreeN;
	kdtreeN.setInputCloud (cloud_filtered1);

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	std::vector<float> normaldifference;
	float radius = 0.015f;
	//float knn = 20;
	float normaldiffsum = 0;

	Eigen::Vector3f result;

	
	for (size_t i = 0; i < normals->size(); ++i) 
	{
	normaldiffsum = 0;
		if ( kdtreeN.radiusSearch (cloud_filtered1->at(i), radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
		//if ( kdtreeN.nearestKSearch (cloud_filtered1->at(i), knn, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
		{
		for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j)
		{
		Eigen::Vector3f result;
		Eigen::Map<Eigen::Vector3f> normali (static_cast<float*> (normals->points[i].normal));
		Eigen::Map<Eigen::Vector3f> normalj (static_cast<float*> (normals->points[j].normal));
		result = normali.cross (normalj);
		normaldiffsum+=sqrt(result.dot (result)); 
		}
		}
	normaldifference.push_back(normaldiffsum);
	//std::cout << "pointIdxRadiusSearch.size () " << pointIdxRadiusSearch.size () << std::endl;
	}

	float normalmax;
	normalmax = normaldifference.at(0);

	for (size_t i = 1; i < normaldifference.size(); ++i) 
	{
		if (normaldifference.at(i)>normalmax)
			normalmax = normaldifference.at(i);
	}


	for (size_t i = 0; i < normaldifference.size(); ++i) 
		normaldifference.at(i) = normaldifference.at(i)/normalmax;



	for (size_t i = 0; i < normaldifference.size(); ++i) 
	{
		if (normaldifference.at(i)>.7)
		{
			segment->points.push_back (cloud_filtered1->points[i]);
		}	
	}

	segment->width = segment->points.size ();
	segment->height = 1;
	segment->is_dense = true;	

	std::cout << "segment.size () " << segment->size () << std::endl;

/*
	double threshold = 0.03;
	double k = 0;

	for (size_t i=0;i<normals->size(); ++i)
	{
		if (normals->at(i).curvature > threshold)
		{
			segment->points.push_back (cloud->points[i]);
			++k;
		}
	}	

	segment->width = segment->points.size ();
	segment->height = 1;
	segment->is_dense = true;
*/

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Transformed Cloud"));

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> cloud_h (cloud_filtered1, 0, 0, 255);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> segment_h (segment, 255, 0, 0);


        viewer->setBackgroundColor (0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZRGBA> (cloud_filtered1, cloud_h, "Transformed Cloud1");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.3, "Transformed Cloud1");


	viewer->addPointCloud<pcl::PointXYZRGBA> (segment,segment_h, "Transformed Cloud2");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Transformed Cloud2");



	while (! viewer->wasStopped ())
	{
	viewer->spinOnce ();
	}

return(0);
        
}
