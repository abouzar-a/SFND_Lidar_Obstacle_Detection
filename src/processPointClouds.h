
#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

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
#include "render/render.h"
#include <unordered_set>
#include "quiz/cluster/kdtree.h"

// Template declaration for the ProcessPointClouds class
template <typename PointT>
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
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr,typename pcl::PointCloud<PointT>::Ptr> Ransac3d_xzyI(typename pcl::PointCloud<PointT>::Ptr cloud,int maxIterations,float distanceThreshold);

    std::pair<typename pcl::PointCloud<PointT>::Ptr,typename pcl::PointCloud<PointT>::Ptr> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud,int maxIterations,float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);
    
    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

    // Euclidean clustering method
    std::vector<typename pcl::PointCloud<PointT>::Ptr> EuclideanClustering(
        typename pcl::PointCloud<PointT>::Ptr cloud, 
        float clusterTolerance, 
        int minSize, 
        int maxSize);

private:
    // Private helper function for clustering
    void clusterHelper(
        typename pcl::PointCloud<PointT>::Ptr cloud, 
        std::vector<bool>& processedPoints, 
        int index, 
        typename pcl::PointCloud<PointT>::Ptr cluster, 
        std::shared_ptr<KdTree> tree, 
        float clusterTolerance);
};

// Implementation of EuclideanClustering
template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::EuclideanClustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, 
    float clusterTolerance, 
    int minSize, 
    int maxSize)
{
    std::shared_ptr<KdTree> tree = std::make_shared<KdTree>();
  
    for (int i = 0; i < cloud->points.size(); i++)
    {
        PointT point = cloud->points[i];
        tree->insert({point.x, point.y, point.z}, i);
    }

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<bool> processedPoints(cloud->points.size(), false);    

    for (int i = 0; i < cloud->points.size(); ++i)
    {
        if (processedPoints[i])
            continue;

        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        clusterHelper(cloud, processedPoints, i, cluster, tree, clusterTolerance);

        if ((cluster->size() >= minSize) && (cluster->size() <= maxSize))
        {
            cluster->width = cluster->size();
            cluster->height = 1;
            cluster->is_dense = true;
            clusters.push_back(cluster);
        }
    }

    std::cout << "Clustering found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

// Implementation of clusterHelper
template <typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(
    typename pcl::PointCloud<PointT>::Ptr cloud, 
    std::vector<bool>& processedPoints, 
    int index, 
    typename pcl::PointCloud<PointT>::Ptr cluster, 
    std::shared_ptr<KdTree> tree, 
    float clusterTolerance)
{
    // Mark the current point as processed
    processedPoints[index] = true;

    // Add the current point to the cluster
    cluster->points.push_back(cloud->points[index]);

    // Perform a radius search to find all neighbors within clusterTolerance
    std::vector<int> nearbyPoints = tree->search({cloud->points[index].x, cloud->points[index].y, cloud->points[index].z}, clusterTolerance);

    // Iterate through each nearby point
    for (int id : nearbyPoints)
    {
        if (!processedPoints[id])  // If the point has not been processed yet
        {
            clusterHelper(cloud, processedPoints, id, cluster, tree, clusterTolerance);
        }
    }
}

#endif // PROCESSPOINTCLOUDS_H_



