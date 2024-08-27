// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "processPointClouds.h"
#include <unordered_set>



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

    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    //std::cout << typeid(vg).name() << std::endl; 
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes , filterRes);
    vg.filter(*cloudFiltered);
  
    typename pcl::PointCloud<PointT>:: Ptr cloudRegion (new pcl::PointCloud<PointT>);
  
    pcl::CropBox<PointT> region(true);
    region.setMin (minPoint);
    region.setMax (maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);
  
    std::vector<int> indices;
  
    pcl::CropBox<PointT> roof(true);
    roof.setMin (Eigen::Vector4f (-1.5, -1.7 , -1 , 1));
    roof.setMax (Eigen::Vector4f (2.6, 1.7 , -4 , 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);
  
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point : indices)
      inliers -> indices.push_back(point);
  
    pcl::ExtractIndices<PointT>extract;
    extract.setInputCloud (cloudRegion);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr obs_cloud (new pcl::PointCloud<PointT> ());
  typename pcl::PointCloud<PointT>::Ptr plane_cloud (new pcl::PointCloud<PointT> ());

  for (int idx: inliers->indices) {
    plane_cloud->points.push_back(cloud->points[idx]);
  }

  // create extraction object
  pcl::ExtractIndices<PointT> extract;
  // Extract the plane
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*obs_cloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obs_cloud, plane_cloud);
    return segResult;
}



template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    pcl::SACSegmentation<PointT> seg;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "PCL-RANSAC plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}



template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Ransac3d_xzyI(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,float distanceTol)



{
    auto startTime= std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

  
    while(maxIterations--)
    {

        std::unordered_set<int> inliers;
        while (inliers.size()<3)
            inliers.insert(rand()%(cloud->points.size()));
        float x1,y1,x2,y2,z1,z2,x3,y3, z3;

        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr ++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr ++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        std::vector<float> v1 = {(x2-x1),(y2-y1),(z2-z1)};
        std::vector<float> v2 = {(x3-x1),(y3-y1),(z3-z1)};

        std::vector<float> cros_prod = {(y2-y1)*(z3-z1)-(z2-z1)*(y3-y1),
                                        (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1),
                                        (x2-x1)*(y3-y1)-(y2-y1)*(x2-x1)};

        float ii = cros_prod[0];
        float jj = cros_prod[1];
        float kk = cros_prod[2];
        std::complex<float> i,j,k;
        std::complex<float> d = -(x1*ii + jj*y1 + kk*z1);

        for (int index = 0 ; index <cloud->points.size() ; index ++)
        {
            if (inliers.count(index)>0)
                continue;
             pcl::PointXYZI point = cloud->points[index];
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;

            float dist = fabs(ii*x4+jj*y4+kk*z4+d)/sqrt(ii*ii +jj*jj+kk*kk);
            if (dist <= distanceTol)
                inliers.insert(index);
        }
        if (inliers.size()>inliersResult.size())
            inliersResult= inliers;

    }


    auto endTime= std::chrono::steady_clock::now();
    auto elapseTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout<< "Ransac Algorthm took: "<< elapseTime.count()<<"microseconds"<<std::endl;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (auto i : inliersResult) {
        inliers->indices.push_back(i);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    return segResult;
}


template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud,int maxIterations, float distanceThreshold)
{
    auto startTime=std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    while(maxIterations--)
    {
    std::unordered_set<int> inliers;
    while (inliers.size()<3)
        inliers.insert(rand()%(cloud->points.size()));

    float x1,y1,z1,x2,y2,z2,x3,y3,z3;
    auto itr= inliers.begin();   

    x1=cloud->points[*itr].x;   
	y1=cloud->points[*itr].y;
	z1=cloud->points[*itr].z;
	++itr;
	x2=cloud->points[*itr].x;
	y2=cloud->points[*itr].y;
	z2=cloud->points[*itr].z;
	++itr;
	x3=cloud->points[*itr].x;
	y3=cloud->points[*itr].y;
	z3=cloud->points[*itr].z;
	
	float a= (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
	float b= (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
	float c= (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
	float d= -(a*x1+b*y1+c*z1);	

    for(int index=0;index<cloud->points.size(); index++)
    {

        if (inliers.count(index)>0)
         continue;

        PointT point=cloud->points[index];
        float x4=point.x;
        float y4=point.y;
        float z4=point.z;

        float d =fabs(a*x4+b*y4+c*z4+d)/sqrt(a*a+b*b+c*c);

       if(d <= distanceThreshold)
         inliers.insert(index);
    }
    if(inliers.size()>inliersResult.size())
      inliersResult=inliers;
    }
    
    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for(int index=0; index<cloud->points.size();index++)
    {

        PointT point=cloud->points[index];
        if(inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else       
            cloudOutliers->points.push_back(point);
    }          
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers,cloudInliers);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);
  

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);

    for (pcl::PointIndices getIndices: clusterIndices)
    {
      typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
      for (int index: getIndices.indices)
        cloudCluster->points.push_back (cloud->points[index]);


      cloudCluster->width = cloudCluster->points.size ();
      cloudCluster->height = 1;
      cloudCluster->is_dense = true;

      clusters.push_back(cloudCluster);
    }
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

// Euclidean clustering method is defined in processPointClouds.h and kdtree is in this path src/quiz/cluster

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


