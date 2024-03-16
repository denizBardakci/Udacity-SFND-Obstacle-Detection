// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <Eigen/Dense>
#include <vector>

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Voxel Grid Downsampling
    pcl::VoxelGrid<PointT> voxelGrid;
    typename pcl::PointCloud<PointT>::Ptr filteredCloud {new pcl::PointCloud<PointT>};
    voxelGrid.setInputCloud(cloud);
    // dimension of the voxel
    voxelGrid.setLeafSize(filterRes, filterRes, filterRes);
    voxelGrid.filter(*filteredCloud);
    
  // Region of Interest (CropBox)
    typename pcl::PointCloud<PointT>::Ptr boxCloud{new pcl::PointCloud<PointT>};
    pcl::CropBox<PointT> cropBox{true};
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.setInputCloud(filteredCloud);
    cropBox.filter(*boxCloud);
    
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
  	roof.setMin(Eigen::Vector4f {-1.5, -1.7, -1, 1});
    roof.setMax(Eigen::Vector4f {2.6, 1.7, -.4, 1});
    roof.setInputCloud(boxCloud);
    roof.filter(indices);
    
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(auto indice : indices)
    	inliers->indices.push_back(indice);
    
    // Remove the rooftop indices
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered{new pcl::PointCloud<PointT>};
    pcl::ExtractIndices<PointT> extract;
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.setInputCloud(boxCloud);
    extract.filter(*cloudFiltered);

  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudFiltered;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
   typename pcl::PointCloud<PointT>::Ptr obstaclesCloud (new pcl::PointCloud<PointT>());
   typename pcl::PointCloud<PointT>::Ptr segmentedPlaneCloud(new pcl::PointCloud<PointT> ());
   for (int index : inliers->indices)
   		segmentedPlaneCloud->points.push_back(cloud->points[index]);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
  	extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstaclesCloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstaclesCloud, segmentedPlaneCloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
  	pcl::PointIndices::Ptr inliers {new pcl::PointIndices()};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients()};    
    // TODO:: Fill in this function to find inliers for the cloud.
	pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    // Mandatory
   	seg.setModelType (pcl::SACMODEL_PLANE);
   	seg.setMethodType (pcl::SAC_RANSAC);
   	seg.setMaxIterations (maxIterations);
   	seg.setDistanceThreshold (distanceThreshold);
   
    // Segment the largest planar component from the input point cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
  
     if (inliers->indices.size() == 0)
    	std::cout << "Could not estimate planar model for the given dataset" << std::endl;
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

// Ransac implementation for project
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{   
  	auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliersResult{new pcl::PointIndices()};

    srand(time(NULL));

    for (size_t iteration = 0; iteration < maxIterations; ++iteration)
    {   
        std::vector<int> pointIndices;
        while (pointIndices.size() < 3)
        	pointIndices.emplace_back(rand()%(cloud->points.size()));
		auto iter = pointIndices.begin();
		PointT point1 = cloud->points.at(*iter++);
		PointT point2 = cloud->points.at(*iter++);
		PointT point3 = cloud->points.at(*iter++);

		// The helper computePlaneCoefficent has been developed for coefficient calculation purpose
		std::vector<float> coefficients = ProcessPointClouds<PointT>::computePlaneCoefficients(point1, point2, point3);

		// Access coefficients as follows:
		float A = coefficients[0];
		float B = coefficients[1];
		float C = coefficients[2];
		float D = coefficients[3];
        // Measure distance between every point and fitted plane
        pcl::PointIndices::Ptr inliersTemp{new pcl::PointIndices()};
        for (auto it = cloud->points.begin(); it != cloud->points.end(); ++it)
        {
            float pointDistance = fabs(A * it->x + B * it->y + C * it->z + D) / sqrt(A * A + B * B + C * C);
            if (pointDistance <= distanceTol)
            {
                inliersTemp->indices.push_back(std::distance(cloud->points.begin(), it));
            }
        }

        // Update the best set of inliers
      if (inliersTemp->indices.size() > inliersResult->indices.size())
        {
            inliersResult = inliersTemp;
        }
    }

    std::cout << "Ransac3D has performed\n" << std::endl;
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersResult, cloud);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac3D took " << elapsedTime.count() << " milliseconds " << elapsedTime.count() << std::endl;
    return segResult;
}

// Function to compute plane coefficients A, B, C, and D
template<typename PointT>
std::vector<float> ProcessPointClouds<PointT>::computePlaneCoefficients(const PointT& point1, const PointT& point2, const PointT& point3)
{
    float x1 = point1.x, y1 = point1.y, z1 = point1.z;
    float x2 = point2.x, y2 = point2.y, z2 = point2.z;
    float x3 = point3.x, y3 = point3.y, z3 = point3.z;

    float A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    float B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    float C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    float D = -1 * (A * x1 + B * y1 + C * z1);

    return {A, B, C, D};
}

// Next 3 function handles Clustering operations by using its helpers
template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringCustom(const typename pcl::PointCloud<PointT>::Ptr &cloud, float clusterTol, int minSize, int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();

    // Use std::unique_ptr for dynamic memory allocation
    std::unique_ptr<KdTree> tree3D(new KdTree());
    std::vector<std::vector<float>> pointsVec;
    // Prevent re-allocation of heap by reserving the amount of cloud size in advance
    pointsVec.reserve(cloud->points.size());

    for (const auto &point : cloud->points)
    {
        std::vector<float> pointVec = {point.x, point.y, point.z};
        // Insert points via insert function of quiz/cluster/kdtree.h 
        tree3D->insert(pointVec, pointsVec.size());
        pointsVec.emplace_back(pointVec);
    }

    // Perform clustering
    std::vector<std::vector<int>> clusterIndices = EuclideanCluster(pointsVec, tree3D.get(), clusterTol, minSize, maxSize);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Process clusters
    for (const auto &cluster : clusterIndices)
    {
        // Use std::unique_ptr for dynamic memory allocation
        std::unique_ptr<pcl::PointCloud<PointT>> cloudCluster(new pcl::PointCloud<PointT>());
        cloudCluster->reserve(cluster.size());

        for (int index : cluster)
        {
            cloudCluster->push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(std::move(cloudCluster));

        std::cout << "PointCloud representing the Cluster: " << clusters.back()->size() << " data points." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters." << std::endl;
    return clusters;
}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::EuclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol, float minSize, float maxSize)
{
    std::vector<std::vector<int>> clusters;
    std::vector<bool> isProcessedVector(points.size(), false);

    for (size_t i = 0; i < points.size(); i++)
    {
        if (!isProcessedVector[i])
        {
            std::vector<int> cluster;
            ClusterHelper(points, tree, distanceTol, i, cluster, isProcessedVector);
            if (cluster.size() >= minSize && cluster.size() <= maxSize)
                clusters.push_back(cluster);
        }
    }
    return clusters;
}

template <typename PointT>
void ProcessPointClouds<PointT>::ClusterHelper(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol, int pointIdx, std::vector<int> &cluster, std::vector<bool> &isProcessedVector)
{
    if (isProcessedVector[pointIdx])
    {
        return; // Skip if the point has already been processed
    }

    isProcessedVector[pointIdx] = true;
    cluster.push_back(pointIdx);

    // auto for cleaner code
    auto potentialPointClusterIdx = tree->search(points[pointIdx], distanceTol);

    for (int idx : potentialPointClusterIdx)
    {
        ClusterHelper(points, tree, distanceTol, idx, cluster, isProcessedVector);
    }
}



template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr segmentedCloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
	// Creating the KdTree object for the search method of the extraction
	 typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
 	 tree->setInputCloud (segmentedCloud);
 
	std::vector<pcl::PointIndices> cluster_indices;
 	pcl::EuclideanClusterExtraction<PointT> ec;
 	ec.setClusterTolerance (clusterTolerance); // 2cm
 	ec.setMinClusterSize (minSize);
 	ec.setMaxClusterSize (maxSize);
 	ec.setSearchMethod (tree);
 	ec.setInputCloud (segmentedCloud);
 	ec.extract (cluster_indices);
  
    for (pcl::PointIndices getIndices : cluster_indices){
      typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
      for (auto index : getIndices.indices){
      	cloudCluster->points.push_back(segmentedCloud->points[index]);
      }
      cloudCluster->width = cloudCluster->points.size();
      cloudCluster->height = 1;
      cloudCluster->is_dense = true;
      clusters.push_back(cloudCluster);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}