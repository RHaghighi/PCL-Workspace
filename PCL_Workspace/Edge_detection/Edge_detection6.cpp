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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

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
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-0.45, 0.45);
	pass.filter (*cloud_filtered);
	pass.setInputCloud (cloud_filtered);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-0.45, 0.45);
	pass.filter (*cloud_filtered);
	pass.setInputCloud (cloud_filtered);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered);

	// Voxel Grid Filter 
        pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
        sor.setInputCloud (cloud_filtered);
        sor.setLeafSize (0.005f, 0.005f, 0.005f);
        sor.filter (*cloud_filtered);

	std::cout << "cloud_filtered.size () " << cloud_filtered->size () << std::endl;

	// Outlier Removal Filter
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> outrem;
	outrem.setInputCloud(cloud_filtered);
	outrem.setMeanK (5);
	outrem.setStddevMulThresh (0.5);
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

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segment(new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtreeN;
	kdtreeN.setInputCloud (cloud_filtered1);

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	std::vector<float> normaldifference;
	float radius = 0.01f;
	float normaldiffsum = 0;

	
	for (size_t i = 0; i < normals->size(); ++i) 
	{
	normaldiffsum = 0;
		if ( kdtreeN.radiusSearch (cloud_filtered1->at(i), radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
		{
		for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j)
		{
		//if (sqrt(pow(normals->at(i).normal_x - normals->at(pointIdxRadiusSearch[j]).normal_x,2)+pow(normals->at(i).normal_y - normals->at(pointIdxRadiusSearch[j]).normal_y,2)+pow(normals->at(i).normal_z - normals->at(pointIdxRadiusSearch[j]).normal_z,2))<1.38)
		//normaldiffsum+=sqrt(pow(normals->at(i).normal_x - normals->at(pointIdxRadiusSearch[j]).normal_x,2)+pow(normals->at(i).normal_y - normals->at(pointIdxRadiusSearch[j]).normal_y,2)+pow(normals->at(i).normal_z - normals->at(pointIdxRadiusSearch[j]).normal_z,2)); 

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

	std::cout << "normalmax: " << normalmax << std::endl;

/*
	for (size_t i = 0; i < normaldifference.size(); ++i) 
		normaldifference.at(i) = normaldifference.at(i)/normalmax;
*/


	for (size_t i = 0; i < normaldifference.size(); ++i) 
	{
		if (normaldifference.at(i)>15.7)
		{
			segment->points.push_back (cloud_filtered1->points[i]);
		}	
	}

	segment->width = segment->points.size ();
	segment->height = 1;
	segment->is_dense = true;	

	std::cout << "segment.size () " << segment->size () << std::endl;

/*
	// PassThrough Filter
	pass.setInputCloud (segment);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-0.1, 0.1);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-0.3, 0.3);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.2);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*segment);
*/

	// Voxel Grid Filter 
        sor.setInputCloud (segment);
        sor.setLeafSize (0.03f, 0.03f, 0.03f);
        sor.filter (*segment);

	// Outlier Removal Filter
	outrem.setInputCloud(segment);
	outrem.setMeanK (5);
	outrem.setStddevMulThresh (0.5);
	outrem.filter (*segment);


	pcl::ModelCoefficients line; 
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices); 
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg; 
	seg.setOptimizeCoefficients(true); 
	seg.setModelType(pcl::SACMODEL_LINE); 
	seg.setMethodType(pcl::SAC_RANSAC); 
	seg.setDistanceThreshold(0.01); 

	int nr_points = (int) segment->points.size ();
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segment_p(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segment_p1(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segment_f(new pcl::PointCloud<pcl::PointXYZRGBA>);


	while (segment->points.size () > 0.5 * nr_points)
	{
	seg.setInputCloud(segment); 
	seg.segment(*inliers, line); 
	if (inliers->indices.size () == 0)
	{
	std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	break;
	}

	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;	
	extract.setInputCloud (segment);
	extract.setIndices (inliers);
	extract.setNegative (false);
	extract.filter (*segment_p1);

	*segment_p+=*segment_p1;

	extract.setNegative (true);
	extract.filter (*segment_f);
	segment.swap (segment_f);

	}



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
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> segment_h (segment_p, 255, 0, 0);


        viewer->setBackgroundColor (255, 255, 255);
        viewer->addPointCloud<pcl::PointXYZRGBA> (cloud_filtered1, cloud_h, "Transformed Cloud1");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "Transformed Cloud1");


	viewer->addPointCloud<pcl::PointXYZRGBA> (segment_p,segment_h, "Transformed Cloud2");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Transformed Cloud2");



	while (! viewer->wasStopped ())
	{
	viewer->spinOnce ();
	}

return(0);
        
}
