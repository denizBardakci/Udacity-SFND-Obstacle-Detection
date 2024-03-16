/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <thread>
std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* myLidarPtr = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = myLidarPtr->scan();
    //renderPointCloud(viewer, inputCloud, "RenderedPointCloud");
    
   // Creater point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor{};
    // segment the cloud and get the planes via Ransac
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "obsCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
    
    // Cluster the segmented Cloud of Obstacles
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);
	
	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
  
	for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
	{
  		std::cout << "cluster size ";
  		pointProcessor.numPoints(cluster);
  		renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
      	Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
    
      ++clusterId;

	}
     renderPointCloud(viewer, segmentCloud.second, "planeCloud");
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}
// This one for single pcd city block operation
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
  
  ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
  pcl::PointCloud<pcl::PointXYZI>::Ptr realInputCloud = pointProcessorI.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI.FilterCloud(realInputCloud, 0.18, Eigen::Vector4f(-10, -5.0, -2, 1), Eigen::Vector4f(30,7.0, 10, 1));

  // SegmentPlane uses custom Ransac3D function
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointProcessorI.Ransac3D(filteredCloud, 200, 0.2);

  renderPointCloud(viewer, segmentedCloud.first,"obstacleCloud", Color(1,0,0));
  renderPointCloud(viewer, segmentedCloud.second,"planeCloud", Color(0,1,0));

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.ClusteringCustom(segmentedCloud.first, 0.5, 15, 700);
  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(0,1,1), Color(1,1,0)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "Cluster " << std::to_string(clusterId) << " has a size of ";
        pointProcessorI.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstcleCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        Box box = pointProcessorI.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }

}

// PCD Streamer
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{

  pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.2, Eigen::Vector4f(-10, -5.0, -2, 1), Eigen::Vector4f(30, 7.0, 4, 1));

  // custom Ransac3D function handles segmentation
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointProcessorI->Ransac3D(filteredCloud, 100, 0.2);

  renderPointCloud(viewer, segmentedCloud.first,"obstacleCloud", Color(1,0,0));
  renderPointCloud(viewer, segmentedCloud.second,"planeCloud", Color(0,1,0));

  // Custom clustering function
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->ClusteringCustom(segmentedCloud.first, 0.33, 15, 700);

  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0),  Color(0,1,1)};

   for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
   {
        std::cout << "Cluster " << std::to_string(clusterId) << " has a size of ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstcleCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }

}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
 
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();
        viewer->spinOnce();
    }
}