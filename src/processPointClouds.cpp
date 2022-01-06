// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(const typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setInputCloud(cloud_filtered);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.filter(*cloudRegion);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setInputCloud(cloudRegion);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for (auto indice : indices)
        inliers->indices.push_back(indice);

    pcl::ExtractIndices<PointT> eifilter(true);
    eifilter.setInputCloud(cloudRegion);
    eifilter.setIndices(inliers);
    eifilter.setNegative(true);
    eifilter.filterDirectly(cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    // copy the point cloud of obstacle
    typename pcl::PointCloud<PointT>::Ptr inlierPoints (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr inlierPoints_neg (new pcl::PointCloud<PointT>);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    extract.setNegative(true);
    extract.filter(*inlierPoints_neg);

    extract.setNegative(false);
    extract.filter(*inlierPoints);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(inlierPoints_neg, inlierPoints);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	std::random_device dev;
	std::mt19937 eng(dev());
	std::uniform_int_distribution<int> dist(0, cloud->size()-1);

    float distanceThreshold2 = pow(distanceThreshold, 2);
    std::unordered_set<int> inliersResult;

    while (--maxIterations)
    {
        // extract radnom 3 points from the cloud
        std::unordered_set<int> inliers;
        while (inliers.size() < 3)
            inliers.insert(dist(eng));

        auto itr = inliers.begin();
        int idx1 = *itr;
        auto p = cloud->points[idx1];
        float x1 = p.x; float y1 = p.y; float z1 = p.z;
        itr++;

        int idx2 = *itr;
        p = cloud->points[idx2];
        float x2 = p.x; float y2 = p.y; float z2 = p.z;
        itr++;

        int idx3 = *itr;
        p = cloud->points[idx3];
        float x3 = p.x; float y3 = p.y; float z3 = p.z;

        // find coefficient of the plane
        float a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
        float b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
        float c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
        float d = -(a*x1 + b*y1 + c*z1);

        // calculate distance from every point to this plane
        for (int i=0; i<cloud->points.size(); i++)
        {
            float x = cloud->points[i].x;
            float y = cloud->points[i].y;
            float z = cloud->points[i].z;

            float dis = pow(a*x + b*y + c*z + d, 2) / (a*a + b*b + c*c);

            if (dis <= distanceThreshold2)
                inliers.insert(i);
        }

        if (inliers.size() > inliersResult.size())
            inliersResult = std::move(inliers);
    }

    inliers->indices.insert(inliers->indices.begin(), inliersResult.begin(), inliersResult.end());

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    auto itr = cloud->points.begin();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr kdtree (new pcl::search::KdTree<PointT>());
    kdtree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setInputCloud(cloud);
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(kdtree);
    ec.extract(cluster_indices);

    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); it++)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster_point (new pcl::PointCloud<PointT>);
        for (auto itr = it->indices.begin(); itr != it->indices.end(); itr++)
            cluster_point->points.push_back(cloud->points[*itr]);
        cluster_point->width = cluster_point->points.size();
        cluster_point->height = 1;
        cluster_point->is_dense = true;

        clusters.push_back(cluster_point);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    int n = cloud->points.size();
    std::vector<bool> processed(n, false);

    // Build KDtree
    KDTree<PointT>* tree = new KDTree<PointT>(cloud, 3);

    for (int i=0; i<n; i++)
    {
        if (!processed[i])
        {
            std::vector<int> cluster_arr;
            euclideanClusterHelper(i, cloud, cluster_arr, tree, clusterTolerance, processed);
            if (cluster_arr.size() >= minSize && cluster_arr.size() <= maxSize)
            {
                typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
                for (auto i : cluster_arr)
                    cluster->points.push_back(cloud->points[i]);
                clusters.push_back(cluster);
            }
        }
    }

    return clusters;

}

template<typename PointT>
void ProcessPointClouds<PointT>::euclideanClusterHelper(int id, typename pcl::PointCloud<PointT>::Ptr inputCloud, std::vector<int>& cluster, KDTree<PointT> *tree, float clusterTolerance, std::vector<bool> &processed)
{
    processed.at(id) = true;
    cluster.push_back(id);
    for (int pt_id : tree->Search(inputCloud->points[id], clusterTolerance))
    {
        if (!processed[pt_id])
            euclideanClusterHelper(pt_id, inputCloud, cluster, tree, clusterTolerance, processed);
    }

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
BoxQ ProcessPointClouds<PointT>::QBoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    /*
     * pcl::transformPointCloud(*inputCloud, *outputCloud, affineTransformationMatrix(4x4);
     * ex)
     *  p = <x y z>: a single example point in a inputCloud
     *  e1, e2, e3: eigen vectors,
     *  c = <c1 ,c2, c3>: centroid
     *
     * p = <x, y, z>   ==> < e1.dot(p) - c1, e2.dot(p) - c2, e3.dot(p) - c3 >
     *         affine Mat           p                       new_p
     *  | e1_i  e1_j  e1_k  -c1 | | x |     | e1_i*x + e1_j*y + e1_k*z - c1*1 |   => e1.dot(p) - c1
     *  | e2_i  e2_j  e2_k  -c2 | | y |     | e2_i*x + e2_j*y + e2_k*z - c2*1 |   => e2.dot(p) - c2
     *  | e3_i  e3_j  e3_k  -c3 | | z |     | e3_i*x + e3_j*y + e3_k*z - c3*1 |   => e3.dot(p) - c3
     *  | 0     0     0      1  | | 1 |     | 0 * x  + 0 * y  + 0 * z  +  1   |   => 1
     *  Project points to eigen vectors + Move clouds to origin(0,0)
     * */


    BoxQ box;

    Eigen::Vector4f pcaCentroid_xyz;
    pcl::compute3DCentroid(*cluster, pcaCentroid_xyz);

    /// xy orientation where z-plane is flat.
    typename pcl::PointCloud<PointT>::Ptr cluster_xy(new pcl::PointCloud<PointT>);
    for (int i=0; i<cluster->points.size(); i++)
    {
        PointT point;
        point.x = cluster->points[i].x;
        point.y = cluster->points[i].y;
        point.z = 1; // makes z flat

        cluster_xy->points.push_back(point);
    }
    cluster_xy->width = cluster_xy->points.size();
    cluster_xy->height = 1;

    Eigen::Vector4f centroid_xy;
    pcl::compute3DCentroid(*cluster_xy, centroid_xy);

    Eigen::Matrix3f covariance_xy;
    pcl::computeCovarianceMatrixNormalized(*cluster_xy, centroid_xy, covariance_xy) ;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver_xy(covariance_xy, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA_xy = eigen_solver_xy.eigenvectors();
    eigenVectorsPCA_xy.col(2) = eigenVectorsPCA_xy.col(0).cross(eigenVectorsPCA_xy.col(1));

    Eigen::Matrix4f projectionTransform_xyz(Eigen::Matrix4f::Identity());
    projectionTransform_xyz.block<3,3>(0,0) = eigenVectorsPCA_xy.transpose();
    projectionTransform_xyz.block<3,1>(0,3) = -1.f * (projectionTransform_xyz.block<3,3>(0,0) * pcaCentroid_xyz.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointProjected_xy (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointProjected_xy, projectionTransform_xyz);

    PointT minPoint_xy, maxPoint_xy;
    pcl::getMinMax3D(*cloudPointProjected_xy, minPoint_xy, maxPoint_xy);
    const Eigen::Vector3f meanDiagonal_xy = 0.5f * (maxPoint_xy.getVector3fMap() + minPoint_xy.getVector3fMap());

    const Eigen::Quaternionf bboxQuaternion_xyz(eigenVectorsPCA_xy);
    const Eigen::Vector3f bboxTransform_xyz = eigenVectorsPCA_xy * meanDiagonal_xy + pcaCentroid_xyz.head<3>();

    box.bboxQuaternion = bboxQuaternion_xyz;
    box.bboxTransform = bboxTransform_xyz;
    box.cube_length = maxPoint_xy.x - minPoint_xy.x;
    box.cube_width = maxPoint_xy.y - minPoint_xy.y;
    box.cube_height = maxPoint_xy.z - minPoint_xy.z;

    /*
    ///xyz orientation

    /// Compute Principal Components
    // get centroid
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);

    // get cov matrix
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);

    // get normal eigen vectors from cov matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigenSolver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    /// Transform the original cloud to the origin(0,0) where the principal components correspond to the axes

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    /// Final Transform
    // since eigen vectors are orthonormal(because cov matrix is a symmetric matrix),
    // Matrix consists of eigen vectors represents rotationMatrix
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
    const Eigen::Vector3f bboxTransform = pcaCentroid.head<3>() + eigenVectorsPCA * meanDiagonal;
    */

    /*
    box.bboxQuaternion = bboxQuaternion;
    box.bboxTransform = bboxTransform;
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;
    */


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

