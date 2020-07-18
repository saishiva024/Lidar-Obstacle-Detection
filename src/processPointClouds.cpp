// PCL lib Functions for processing point clouds 

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
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);

    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudROI (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);

    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudROI);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);

    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudROI);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};

    
    for(int index:indices)
    {
        inliers->indices.push_back(index);
    }

    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloudROI);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudROI);
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudROI;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud {new pcl::PointCloud<PointT>()};
    typename pcl::PointCloud<PointT>::Ptr planeCloud {new pcl::PointCloud<PointT>()};

    for(int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);extract.setNegative(true);
    extract.filter(*obstacleCloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlaneT(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersOut;

	srand(time(NULL));

	while(maxIterations--)
    {
      std::unordered_set<int> inliers;

      while(inliers.size() < 3)
      {
          inliers.insert(rand() % (cloud->points.size()));
      }

      float x1, y1, z1;
      float x2, y2, z2;
      float x3, y3, z3;

      auto iterator = inliers.begin();

      x1 = cloud->points[*iterator].x;
      y1 = cloud->points[*iterator].y;
      z1 = cloud->points[*iterator].z;

      iterator++;

      x2 = cloud->points[*iterator].x;
      y2 = cloud->points[*iterator].y;
      z2 = cloud->points[*iterator].z;

      iterator++;
      
      x3 = cloud->points[*iterator].x;
      y3 = cloud->points[*iterator].y;
      z3 = cloud->points[*iterator].z;
      
      float A = ((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1));
      float B = ((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1));
      float C = ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1));
      float D = -(A * x1 + B * y1 + C * z1);
      
      
      for(int index = 0; index < cloud->points.size(); index++)
      {
      	if (inliers.count(index) > 0) 
        {
            continue;
        }

        float x4 = cloud->points[index].x;
        float y4 = cloud->points[index].y;
        float z4 = cloud->points[index].z;
        
        float dist = fabs(A * x4 + B * y4 + C * z4 + D) / sqrt(A * A + B * B + C * C);

        if (dist <=  distanceTol) 
        {
            inliers.insert(index);
        }
      }

      if(inliers.size() > inliersOut.size())
      {
           inliersOut = inliers;
      } 
    }
	
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		if(inliersOut.count(index))
		{
			planeCloud->points.push_back(cloud->points[index]);
		}
		else
		{
			obstacleCloud->points.push_back(cloud->points[index]);
		}
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segmentedOut (obstacleCloud, planeCloud);

    return segmentedOut;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};

    pcl::SACSegmentation<PointT> seg;
    
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if(inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model with the given dataset" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelperT(int index, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	processed[index] = true;

	cluster.push_back(index);

	std::vector<int> nearPoints = tree->search(points[index], distanceTol);

	for(int id : nearPoints)
	{
		if(!processed[id])
		{
			clusterHelperT(id, points, cluster, processed, tree, distanceTol);
		}
	}
}


template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanClusterT(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;

	std::vector<bool> processed(points.size(), false);

	int point_index = 0;

	while(point_index < points.size())
	{
		if(processed[point_index])
		{
			point_index++;
			continue;
		}

		std::vector<int> cluster;

		clusterHelperT(point_index, points, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);

		point_index++;
	}
    
	return clusters;
 
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::EuclieanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    KdTree* tree = new KdTree;

    std::vector<std::vector<float>> points;

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    int id = 0;

    for(auto point : cloud->points)
    {
        std::vector<float> floatPoint {point.x, point.y, point.z};
        tree->insert(floatPoint, id);
        points.push_back(floatPoint);

        id++;
    }

    std::vector<std::vector<int>>clusterIndices {euclideanClusterT(points, tree, clusterTolerance)};

    for(auto indexes : clusterIndices)
    {
        if((indexes.size() < minSize) || (indexes.size() > maxSize))
        {
            continue;
        }

        typename pcl::PointCloud<PointT>::Ptr cluster {new pcl::PointCloud<PointT>};

        for(auto index : indexes)
        {
            cluster->points.push_back(cloud->points[index]);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;

    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);

    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(pcl::PointIndices getIndices : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices)
        {
            cloudCluster->points.push_back(cloud->points[index]);
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