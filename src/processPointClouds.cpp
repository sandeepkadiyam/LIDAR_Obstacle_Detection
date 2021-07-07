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
    typename pcl::PointCloud<PointT>::Ptr filteredCloud (new pcl::PointCloud<PointT> ());

    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*filteredCloud);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT> ());

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filteredCloud);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);

    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for(int point : indices) {
        inliers->indices.push_back(point);
    }

    /*typename pcl::PointCloud<PointT>::Ptr roofTop(new pcl::PointCloud<PointT> ());
    for (int index : inliers->indices) {
        roofTop->points.push_back(cloudRegion[index]);
    }*/

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obstaclesCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr roadCloud (new pcl::PointCloud<PointT> ());

    for(int index : inliers->indices) {
        roadCloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstaclesCloud);



    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstaclesCloud, roadCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());



    // Custom implentation of the RANSAC algorithm for a 3D plane.
    
    std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	for (int i = 0; i < maxIterations; ++i) {

		std::unordered_set <int> inlier_points;

		while(inlier_points.size() < 3) {
			inlier_points.insert(rand() % (cloud->points.size()));
		}

		/*int random_number1 = rand() % (cloud->points.size());
		do
		{
			int random_number2 = rand() % (clouds->points.size());
		} while (random_number1 == random_number2);*/

		auto itr = inlier_points.begin();

		float x1 = cloud->points[*itr].x;
		float y1 = cloud->points[*itr].y;
		float z1 = cloud->points[*itr].z;
		itr++;
		float x2 = cloud->points[*itr].x;
		float y2 = cloud->points[*itr].y;
		float z2 = cloud->points[*itr].z;
		itr++;
		float x3 = cloud->points[*itr].x;
		float y3 = cloud->points[*itr].y;
		float z3 = cloud->points[*itr].z;

		float a = ((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1));
		float b = ((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1));
		float c = ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1));
		float d = -((a * x1) + (b * y1) + (c * z1));

		for (int j = 0; j < (cloud->points.size()); ++j) {

			if (inlier_points.count(j) > 0){
				continue;
			}
			
			float x = (cloud->points[j].x);
			float y = (cloud->points[j].y);
			float z = (cloud->points[j].z);

			/*float numerator = ((a * x) + (b * y) + (c * z) + d);
			float denominator = sqrt((a * a) + (b * b) + (c * c));*/

			float distance = fabs((a * x) + (b * y) + (c * z) + d)/ sqrt((a * a) + (b * b) + (c * c));


			if (distance < distanceThreshold) {
				inlier_points.insert(j);
			}
		}

		if (inlier_points.size() > inliersResult.size()) {
			inliersResult = inlier_points;
		}
	}

    inliers->indices.assign(inliersResult.begin(), inliersResult.end());
    
    // TODO:: Fill in this function to find inliers for the cloud.
    /*pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);*/
    if(inliers->indices.size() == 0){
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
void ProcessPointClouds<PointT>:: clusterHelper(int indice, const std::vector<std::vector<float>> points, std::vector<int>& cluster, KdTree* tree, std::vector<bool>& processed, float distanceTol) {

	processed[indice] = true;
	cluster.push_back(indice);

	std::vector<int> nearby_points = tree->search3D(points[indice], distanceTol);

	for(auto index : nearby_points) {
		if(!processed[index]) {
			clusterHelper(index, points, cluster, tree, processed, distanceTol);
		}
	}
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{


    KdTree* tree = new KdTree;

    std::vector<std::vector<float>> points;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters;

    for (int i=0; i<(cloud->points.size()); i++) {
        std::vector<float> point = {(cloud->points[i].x), (cloud->points[i].y), (cloud->points[i].z)};
        points.push_back(point);
    	tree->insert3D(point,i);
    }

    std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);

	int i = 0;
	while(i < (points.size())) {
		if(processed[i]) {
			i++;
			continue;
		}

		std::vector<int> cluster;
		clusterHelper(i, points, cluster, tree, processed, clusterTolerance);
		clusters.push_back(cluster);
		i++;
	}

    for (std::vector<int> cluster : clusters) {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

        if ((cluster.size() < minSize) || (cluster.size() > maxSize)) {
            continue;
        }

        for (int index : cluster) {
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        cloudClusters.push_back(cloudCluster);
    }
 
	return cloudClusters;

    
    // Time clustering process
    /*auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for(pcl::PointIndices getIndices : cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

        for (int index : getIndices.indices) {
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

    return clusters;*/
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

