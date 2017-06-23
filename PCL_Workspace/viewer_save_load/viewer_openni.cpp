#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>


using namespace std;

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudptr (new pcl::PointCloud<pcl::PointXYZRGBA>);

boost::shared_ptr<pcl::visualization::CloudViewer> viewer (new pcl::visualization::CloudViewer("3D viewer"));



pcl::Grabber* openniGrabber;

unsigned int filesSaved = 0; 
bool saveCloud(false);  


void
grabberCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
{
	if (! viewer->wasStopped())
		viewer->showCloud(cloud);

        if (saveCloud)
	{
		stringstream stream;
		stream << "inputCloud" << filesSaved << ".pcd";
		string filename = stream.str();
		if (pcl::io::savePCDFile(filename, *cloud, true) == 0)
		{
			filesSaved++;
			cout << "Saved " << filename << "." << endl;

                pcl::visualization::PCLVisualizer viewer2 ("3D viewer");
                pcl::io::loadPCDFile(filename, *cloudptr);
                viewer2.addPointCloud (cloud);
while (! viewer2.wasStopped())
{
}

		}

		else PCL_ERROR("Problem saving %s.\n", filename.c_str());
 
		saveCloud = false;
	}

 }


// For detecting when SPACE is pressed.
void
keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
					  void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		saveCloud = true;
}
 
// Creates, initializes and returns a new viewer.
boost::shared_ptr<pcl::visualization::CloudViewer>
createViewer()
{
	boost::shared_ptr<pcl::visualization::CloudViewer> v
	(new pcl::visualization::CloudViewer("OpenNI viewer"));
	v->registerKeyboardCallback(keyboardEventOccurred);
 
	return (v);
}

int
main (int argc, char** argv)
{


    openniGrabber = new pcl::OpenNIGrabber();

    if (openniGrabber == 0)
	 return -1;


    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(&grabberCallback, _1);
		openniGrabber->registerCallback(f);

viewer = createViewer();


openniGrabber->start();



while (! viewer->wasStopped())
		boost::this_thread::sleep(boost::posix_time::seconds(1));



}
