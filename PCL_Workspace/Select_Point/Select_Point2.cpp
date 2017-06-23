#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/pcl_base.h>
#include <iostream>

using namespace std;
using namespace pcl;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef CloudT::Ptr ptrCloudT;


void PointSelection_callback(const pcl::visualization::PointPickingEvent &event, void*)
{
    cout << " point index: " << event.getPointIndex() << endl;
}

ptrCloudT GeneratePlanePointCloudXWithShift(int NumberOfPoints, float EdgeLength, int XShift)
{
    ptrCloudT Surface(new CloudT);
    int len = sqrt(NumberOfPoints);
    float step = EdgeLength / len;
    for (float x = 0; x < EdgeLength; x += step)
        for (float y = 0; y < EdgeLength; y += step)
        {
            PointT p;
            p.x = x + XShift; p.y = y;
            p.r = 255; p.g = 255; p.b = 255; p.a = 0;
            Surface->insert(Surface->end(), p);
        }

    return Surface;
}


int main()
{
    pcl::visualization::PCLVisualizer::Ptr m_PCLVisualization(new pcl::visualization::PCLVisualizer("view", true));
    m_PCLVisualization->registerPointPickingCallback(PointSelection_callback);
    ptrCloudT cloud = GeneratePlanePointCloudXWithShift(1000, 10,0);
    ptrCloudT ShiftedCloud = GeneratePlanePointCloudXWithShift(1000, 10,40);

    m_PCLVisualization->addPointCloud(cloud,"Cloud1"); 
    m_PCLVisualization->addPointCloud(ShiftedCloud, "Cloud2");
    m_PCLVisualization->spin();
}
