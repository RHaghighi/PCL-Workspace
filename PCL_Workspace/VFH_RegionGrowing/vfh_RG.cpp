#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "vfh.h"
#include "vfh.hpp"
//#include <pcl/features/vfh.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <string.h>
#include <pcl/common/transforms.h> 
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/features/cvfh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/features/shot.h>
#include <pcl/registration/icp.h>
#include <cstring>


boost::shared_ptr<pcl::visualization::PCLHistogramVisualizer>
            HistoVis (pcl::PointCloud<pcl::VFHSignature308>::ConstPtr cloud1, int length) 
{ 
    boost::shared_ptr<pcl::visualization::PCLHistogramVisualizer> histogram (new pcl::visualization::PCLHistogramVisualizer); 
    histogram->addFeatureHistogram(*cloud1, length, "cloud", 800, 600); 
    histogram->setBackgroundColor(255,255,255); 
    
    return (histogram); 
} 




int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr model_normals (new pcl::PointCloud<pcl::Normal> ());
        pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr scene_normals (new pcl::PointCloud<pcl::Normal> ());
        pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// Create an empty kdtree object
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
 
	// Read PCD files from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *model_cloud) != 0)
	{
		return -1;
	}
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *scene_cloud) != 0)
	{
		return -1;
	}


	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setLeafSize (0.01f, 0.01f, 0.01f);


	sor.setInputCloud (model_cloud);
	sor.filter (*model_cloud_filtered);

	std::cerr << "PointCloud size= " << model_cloud->size()  << ",     " << "Filtered PointCloud size= " << model_cloud_filtered->size()  << std::endl;

	sor.setInputCloud (scene_cloud);
	sor.filter (*scene_cloud_filtered);

	std::cerr << "PointCloud size= " << scene_cloud->size()  << ",     " << "Filtered PointCloud size= " << scene_cloud_filtered->size()  << std::endl;


	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setSearchMethod (tree);
	ne.setRadiusSearch (0.03);

        // Region growing object
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize (100);
	reg.setMaxClusterSize (10000);
	reg.setSearchMethod (tree);
	reg.setNumberOfNeighbours (10);
	reg.setSmoothnessThreshold (4.5 / 180.0 * M_PI);
	reg.setCurvatureThreshold (0.2);


	// Compute Normals
	ne.setInputCloud (model_cloud_filtered);
	ne.compute (*model_normals);

	ne.setInputCloud (scene_cloud_filtered);
	ne.compute (*scene_normals);


	// Compute model segmentation
	reg.setInputCloud (model_cloud_filtered);
	reg.setInputNormals (model_normals);


	std::vector <pcl::PointIndices> model_clusters;
	reg.extract (model_clusters);
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr model_colored_cloud = reg.getColoredCloud ();

	std::cout << "Number of model_clusters= " << model_clusters.size () << std::endl;

	// Compute scene segmentation
	reg.setInputCloud (scene_cloud_filtered);
	reg.setInputNormals (scene_normals);

	std::vector <pcl::PointIndices> scene_clusters;
	reg.extract (scene_clusters);
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr scene_colored_cloud = reg.getColoredCloud ();

	std::cout << "Number of scene_clusters= " << scene_clusters.size () << std::endl;

     pcl::PointCloud<pcl::PointXYZ>::Ptr model123 (new pcl::PointCloud<pcl::PointXYZ>);

        /// Extract model segmentations into point clouds

	int j = 0;
	std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > model_segment;

	std::vector < pcl::PointCloud<pcl::Normal>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::Normal>::Ptr > > model_normal_segment;

	for (std::vector<pcl::PointIndices>::const_iterator it = model_clusters.begin (); it != model_clusters.end (); ++it)
	{
	if (it->indices.size()>200)
	{
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr normal_cloud_cluster (new pcl::PointCloud<pcl::Normal>);	

		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		{
				cloud_cluster->points.push_back (model_cloud_filtered->points[*pit]); 	
				normal_cloud_cluster->points.push_back (model_normals->points[*pit]); 
		}

		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		normal_cloud_cluster->width = normal_cloud_cluster->points.size ();
		normal_cloud_cluster->height = 1;
		normal_cloud_cluster->is_dense = true;

 if (j==0 or j==112)
*model123 = *model123 + *cloud_cluster; 

 

		model_segment.push_back(cloud_cluster);
		model_normal_segment.push_back(normal_cloud_cluster);
	
                std::cout << "model Cluster "  <<  j   << " size=  "  << model_segment[j]->size ()  << std::endl;

		j++;
	}
	}

pcl::io::savePCDFileASCII ("model6_modified.pcd", *model123);



        /// Extract scene segmentations into point clouds

	j = 0;
	std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > scene_segment;


	std::vector < pcl::PointCloud<pcl::Normal>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::Normal>::Ptr > > scene_normal_segment;



     pcl::PointCloud<pcl::PointXYZ>::Ptr scene123 (new pcl::PointCloud<pcl::PointXYZ>);


	for (std::vector<pcl::PointIndices>::const_iterator it = scene_clusters.begin (); it != scene_clusters.end (); ++it)
	{
	if (it->indices.size()>200)
	{
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr normal_cloud_cluster (new pcl::PointCloud<pcl::Normal>);

		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		{
				cloud_cluster->points.push_back (scene_cloud_filtered->points[*pit]); 	
				normal_cloud_cluster->points.push_back (scene_normals->points[*pit]); 
		}

		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		normal_cloud_cluster->width = normal_cloud_cluster->points.size ();
		normal_cloud_cluster->height = 1;
		normal_cloud_cluster->is_dense = true;

 if (j==0 or j==118)
*scene123 = *scene123 + *cloud_cluster; 


		scene_segment.push_back(cloud_cluster);
		scene_normal_segment.push_back(normal_cloud_cluster);
	
                std::cout << "scene Cluster "  <<  j   << " size=  "  << scene_segment[j]->size ()  << std::endl;
		j++;
	}
	}


pcl::io::savePCDFileASCII ("scene2_modified.pcd", *scene123);



	  // Create the VFH estimation class, and pass the input dataset+normals to it
	pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setSearchMethod (tree);

	// Output datasets

	std::vector < pcl::PointCloud<pcl::VFHSignature308>::Ptr, Eigen::aligned_allocator <pcl::PointCloud<pcl::VFHSignature308>::Ptr > > model_vfhs;

	std::vector < pcl::PointCloud<pcl::VFHSignature308>::Ptr, Eigen::aligned_allocator <pcl::PointCloud<pcl::VFHSignature308>::Ptr > > scene_vfhs;

	pcl::PointCloud<pcl::VFHSignature308>::Ptr model_descriptor (new pcl::PointCloud<pcl::VFHSignature308>);


	pcl::PointCloud<pcl::PointXYZ>::Ptr model_segments_centers (new pcl::PointCloud<pcl::PointXYZ>);
	model_segments_centers->width = model_segment.size ();
	model_segments_centers->height = 1;
	model_segments_centers->is_dense = true;
	model_segments_centers->points.resize (model_segments_centers->width * model_segments_centers->height);


	float dep_x=0;
	float dep_y=0;
	float dep_z=0;

	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

	// Compute model VFH 
	for (size_t i=0; i<model_segment.size (); ++i)
	{

		transform = Eigen::Matrix4f::Identity();

		dep_x=0;
		dep_y=0;
		dep_z=0;

		pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud_segment (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud_segment_transformed (new pcl::PointCloud<pcl::PointXYZ> ());
		

		*model_cloud_segment=*model_segment[i];

		for (size_t j=0;j<model_cloud_segment->size();++j)
		{
			dep_x += model_cloud_segment->at(j).x;
			dep_y += model_cloud_segment->at(j).y;
			dep_z += model_cloud_segment->at(j).z; 
		}

		dep_x = dep_x/model_cloud_segment->size();
		dep_y = dep_y/model_cloud_segment->size();
		dep_z = dep_z/model_cloud_segment->size();


		transform (0,3) = -dep_x;
		transform (1,3) = -dep_y;
		transform (2,3) = -dep_z;

/*
                std::cout << "c_x:  " << dep_x << "c_y:  " << dep_y << "c_z:  " << dep_z << std::endl;

                Eigen::Vector4f centr;
                pcl::compute3DCentroid(*model_cloud_segment,centr); 
                
                std::cout << "centroid:  " << centr  << std::endl;
*/

		pcl::transformPointCloud (*model_cloud_segment, *model_cloud_segment_transformed, transform);

/*
                Eigen::Vector4f centr1;
                pcl::compute3DCentroid(*model_cloud_segment_transformed,centr1); 
                
                std::cout << "centroid1:  " << centr1  << std::endl;
*/

                //
   		model_segments_centers->at(i).x=dep_x;
   		model_segments_centers->at(i).y=dep_y;
   		model_segments_centers->at(i).z=dep_z;



		// Compute Normals
		pcl::PointCloud<pcl::Normal>::Ptr model_normals_segment_transformed (new pcl::PointCloud<pcl::Normal> ());
		ne.setInputCloud (model_cloud_segment_transformed);
		ne.compute (*model_normals_segment_transformed);


		pcl::PointCloud<pcl::VFHSignature308>::Ptr transformed_vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
		vfh.setInputCloud (model_cloud_segment_transformed);
		vfh.setInputNormals (model_normals_segment_transformed);
		//vfh.setInputNormals (model_normal_segment[i]);



		vfh.compute (*transformed_vfhs);
	if (i==0){
               for (int ii=180;ii<308;++ii)
                   std::cout << transformed_vfhs->points[0].histogram[ii] << ",  ";
	}

 	std::cout << std::endl;

		model_vfhs.push_back(transformed_vfhs);

		model_descriptor->points.push_back(transformed_vfhs->at(0));

	}

	std::cout <<  "model segments centers:" << std::endl;
	for (size_t i=0;i< model_segments_centers->points.size ();++i)
	{
		std::cout <<  model_segments_centers->at(i) << std::endl;
	}


	pcl::PointCloud<pcl::VFHSignature308>::Ptr scene_descriptor (new pcl::PointCloud<pcl::VFHSignature308>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_segments_centers (new pcl::PointCloud<pcl::PointXYZ>);
	scene_segments_centers->width = scene_segment.size ();
	scene_segments_centers->height = 1;
	scene_segments_centers->is_dense = true;
	scene_segments_centers->points.resize (scene_segments_centers->width * scene_segments_centers->height);




	// Compute scene VFH 
	for (size_t i=0; i<scene_segment.size (); ++i)
	{

		transform = Eigen::Matrix4f::Identity();

		dep_x=0;
		dep_y=0;
		dep_z=0;

		pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud_segment (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud_segment_transformed (new pcl::PointCloud<pcl::PointXYZ> ());

		*scene_cloud_segment=*scene_segment[i];

		for (size_t j=0;j<scene_cloud_segment->size();++j)
		{
			dep_x += scene_cloud_segment->at(j).x;
			dep_y += scene_cloud_segment->at(j).y;
			dep_z += scene_cloud_segment->at(j).z; 
		}

		dep_x = dep_x/scene_cloud_segment->size();
		dep_y = dep_y/scene_cloud_segment->size();
		dep_z = dep_z/scene_cloud_segment->size();


		transform (0,3) = -dep_x;
		transform (1,3) = -dep_y;
		transform (2,3) = -dep_z;


		pcl::transformPointCloud (*scene_cloud_segment, *scene_cloud_segment_transformed, transform);


                //
   		scene_segments_centers->at(i).x=dep_x;
   		scene_segments_centers->at(i).y=dep_y;
   		scene_segments_centers->at(i).z=dep_z;


		// Compute Normals
		pcl::PointCloud<pcl::Normal>::Ptr scene_normals_segment_transformed (new pcl::PointCloud<pcl::Normal> ());
		ne.setInputCloud (scene_cloud_segment_transformed);
		ne.compute (*scene_normals_segment_transformed);


		pcl::PointCloud<pcl::VFHSignature308>::Ptr transformed_vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
		vfh.setInputCloud (scene_cloud_segment_transformed);
		vfh.setInputNormals (scene_normals_segment_transformed);
		//vfh.setInputNormals (scene_normal_segment[i]);

		vfh.compute (*transformed_vfhs);
		scene_vfhs.push_back(transformed_vfhs);

		scene_descriptor->points.push_back(transformed_vfhs->at(0));

	}

	std::cout <<  "scene segments centers:" << std::endl;
	for (size_t i=0;i< scene_segments_centers->points.size ();++i)
	{
		std::cout <<  scene_segments_centers->at(i) << std::endl;
	}




	//  Find Model-Scene Correspondences with KdTree

	pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

	pcl::KdTreeFLANN<pcl::VFHSignature308> match_search;
	match_search.setInputCloud (model_descriptor);

	for (size_t i = 0; i < scene_descriptor->size (); ++i)
	{
		std::vector<int> neigh_indices (1);
		std::vector<float> neigh_sqr_dists (1);

		int found_neighs = match_search.nearestKSearch (scene_descriptor->at (i), 1, neigh_indices, neigh_sqr_dists);

		if (found_neighs == 1 && neigh_sqr_dists[0] < 2000.0f)
		{
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back (corr);
		}	
	


	}

	std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

	for (size_t i=0; i<model_scene_corrs->size () ; ++i)
		std::cout << "Model-scene Correspondence " << i << " :   " << model_scene_corrs->at(i) << std::endl;


	//for (size_t i=0; i < 308; ++i)		
		//std::cout << model_vfhs[0]->at(0).histogram[i] << ", ";



	// Saving data 

        //save model cloud

        std::stringstream model_filename;
        model_filename << "cloud_descriptor_" << argv[1];
	std::string model_filename_string = model_filename.str();
        model_filename_string = model_filename_string.substr (0,model_filename_string.length()-4);
	model_filename_string += ".txt";
	const char* model_filename_char = model_filename_string.c_str();

	std::ofstream FILE1(model_filename_char);

        for (size_t i=0; i<model_segments_centers->size(); ++i)
        {
              	FILE1 << "cloud(" << i+1 << "," << 1 << ")=" << model_segments_centers->at(i).x << ";" << std::endl;
		FILE1 << "cloud(" << i+1 << "," << 2 << ")=" << model_segments_centers->at(i).y << ";" << std::endl;
		FILE1 << "cloud(" << i+1 << "," << 3 << ")=" << model_segments_centers->at(i).z << ";" << std::endl;
	}

        //save model descriptor

        for (size_t i=0; i<model_vfhs.size(); ++i)
        {
		for (size_t j=0; j < 308; ++j)
		{
              		FILE1 << "Descriptor(" << i+1 << "," << j+1 << ")=" << model_vfhs[i]->at(0).histogram[j] << ";" << std::endl;
       		}
	}

       	//save scene cloud

        std::stringstream scene_filename;
        scene_filename << "cloud_descriptor_" << argv[2];
	std::string scene_filename_string = scene_filename.str();
        scene_filename_string = scene_filename_string.substr (0,scene_filename_string.length()-4);
	scene_filename_string += ".txt";
	const char* scene_filename_char = scene_filename_string.c_str();

	std::ofstream FILE2(scene_filename_char);


	//std::ofstream FILE("cloud_descriptor.txt");

        for (size_t i=0; i<scene_segments_centers->size(); ++i)
        {
              	FILE2 << "cloud(" << i+1 << "," << 1 << ")=" << scene_segments_centers->at(i).x << ";" << std::endl;
		FILE2 << "cloud(" << i+1 << "," << 2 << ")=" << scene_segments_centers->at(i).y << ";" << std::endl;
		FILE2 << "cloud(" << i+1 << "," << 3 << ")=" << scene_segments_centers->at(i).z << ";" << std::endl;
	}

        //save scene descriptor

        for (size_t i=0; i<scene_vfhs.size(); ++i)
        {
		for (size_t j=0; j < 308; ++j)
		{
              		FILE2 << "Descriptor(" << i+1 << "," << j+1 << ")=" << scene_vfhs[i]->at(0).histogram[j] << ";" << std::endl;
       		}
	}



	/// VISUALIZATION Clouds

	boost::shared_ptr<pcl::visualization::PCLVisualizer> model_viewer (new pcl::visualization::PCLVisualizer ("model Cluster viewer"));
	boost::shared_ptr<pcl::visualization::PCLVisualizer> scene_viewer (new pcl::visualization::PCLVisualizer ("scene Cluster viewer"));

        model_viewer->setBackgroundColor (255, 255, 255);
        scene_viewer->setBackgroundColor (255, 255, 255);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> model_rgb(model_colored_cloud);
	model_viewer->addPointCloud<pcl::PointXYZRGB> (model_colored_cloud,model_rgb,"model Cluster viewer");

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> scene_rgb(scene_colored_cloud);
	scene_viewer->addPointCloud<pcl::PointXYZRGB> (scene_colored_cloud,scene_rgb,"scene Cluster viewer");

        model_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "model Cluster viewer");

        scene_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene Cluster viewer");


        ////

        /// Visualization of descriptors

	std::vector < boost::shared_ptr<pcl::visualization::PCLHistogramVisualizer>, Eigen::aligned_allocator <boost::shared_ptr<pcl::visualization::PCLHistogramVisualizer> > > model_histo_viewer;

/*

	for (size_t i=0; i<model_segment.size (); ++i)
	{
		std::stringstream model;
		model << "model_histogram " << i <<  "  ( cluster size "  << model_segment[i]->size ()  << " )" ;
		const std::string model_tmp = model.str();
		const char* model_cstr = model_tmp.c_str();

		boost::shared_ptr<pcl::visualization::PCLHistogramVisualizer> model_histo_viewer_init (new pcl::visualization::PCLHistogramVisualizer); 
		model_histo_viewer.push_back(model_histo_viewer_init);
		model_histo_viewer[i]->addFeatureHistogram(*model_vfhs[i],308, model_cstr , 800, 600); 
		model_histo_viewer[i]->setBackgroundColor(255,255,255); 
	}


	std::vector < boost::shared_ptr<pcl::visualization::PCLHistogramVisualizer>, Eigen::aligned_allocator <boost::shared_ptr<pcl::visualization::PCLHistogramVisualizer> > > scene_histo_viewer;

	for (size_t i=0; i<scene_segment.size (); ++i)
	{
		std::stringstream scene;
		scene << "scene_histogram " << i  <<  "  ( cluster size "  << scene_segment[i]->size ()  << " )" ;
		const std::string scene_tmp = scene.str();
		const char* scene_cstr = scene_tmp.c_str();

		boost::shared_ptr<pcl::visualization::PCLHistogramVisualizer> scene_histo_viewer_init (new pcl::visualization::PCLHistogramVisualizer); 
		scene_histo_viewer.push_back(scene_histo_viewer_init);
		scene_histo_viewer[i]->addFeatureHistogram(*scene_vfhs[i],308, scene_cstr, 800, 600); 
		scene_histo_viewer[i]->setBackgroundColor(255,255,255); 
	}


*/


        //save model view

        std::stringstream model_view;
        model_view << argv[1];
	std::string model_filename_view = model_view.str();
        model_filename_view = model_filename_view.substr (0,model_filename_view.length()-4);
	model_filename_view += ".png";
	const char* model_filename_char_view = model_filename_view.c_str();

       	//save scene cloud

        std::stringstream scene_view;
        scene_view << argv[2];
	std::string scene_filename_view = scene_view.str();
        scene_filename_view = scene_filename_view.substr (0,scene_filename_view.length()-4);
	scene_filename_view += ".png";
	const char* scene_filename_char_view = scene_filename_view.c_str();

  

	while (!model_viewer->wasStopped ())
	{
		model_viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

        model_viewer->saveScreenshot (model_filename_char_view);
	scene_viewer->saveScreenshot (scene_filename_char_view);





  return(0);
}
