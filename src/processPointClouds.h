// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>
#include "quiz/cluster/kdtree.h"
#include <memory>
template<typename PointT>
class ProcessPointClouds {
public:
    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
    
    // My Segmentation method with Ransac
   std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
   // Ransac3D Helper   
   std::vector<float> computePlaneCoefficients(const PointT& point1, const PointT& point2, const PointT& point3);

    // PCL based clustering
    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
    // Custom Clustering
    std::vector<typename pcl::PointCloud<PointT>::Ptr> ClusteringCustom(const typename pcl::PointCloud<PointT>::Ptr &cloud, float clusterTolerance, int minSize, int maxSize);
    // Cluster based on euclidean
    std::vector<std::vector<int>> EuclideanCluster(const std::vector<std::vector<float>> &points,  KdTree *tree, float distanceTol, float minSize, float maxSize);
    // Cluster Helper
    void ClusterHelper(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol, int pointIdx, std::vector<int> &cluster, std::vector<bool> &isProcessedVector);
    
    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
    
};
#endif /* PROCESSPOINTCLOUDS_H_ */