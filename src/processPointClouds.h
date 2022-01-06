// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/features/statistical_multiscale_interest_region_extraction.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <random>
#include <unordered_set>
#include "render/box.h"

template<typename PointT>
struct Node
{
    int id{0};
    PointT point;
    Node* left {nullptr};
    Node* right {nullptr};

    Node() = default;
    explicit Node(int id_, PointT point_) : id(id_), point(point_), left(nullptr), right(nullptr)
    {}
    ~Node()
    {
        delete left;
        delete right;
    }
};

template<typename PointT>
struct KDTree
{
    Node<PointT>* root;
    int dim;
    KDTree() : root(nullptr) {}
    ~KDTree() { delete root; }
    KDTree(typename pcl::PointCloud<PointT>::Ptr inputCloud, int dim_)
    {
        root = nullptr;
        dim = dim_;
        BuildTree(inputCloud);
    }

    void SetDimension(int dim_)
    {
        dim = dim_;
    }

    void BuildTree(typename pcl::PointCloud<PointT>::Ptr inputCloud)
    {
        if (root != nullptr)
            delete root;

        std::vector<std::pair<int, PointT>> cloudPoints;
        for (int i=0; i<inputCloud->points.size(); i++)
            cloudPoints.push_back(std::make_pair(i, inputCloud->points[i]));
        std::cout << "d" << std::endl;
        root = buildTreeHelper(cloudPoints.begin(), cloudPoints.end(), 0);
    }

    Node<PointT>* buildTreeHelper(typename std::vector<std::pair<int, PointT>>::iterator start, typename std::vector<std::pair<int, PointT>>::iterator end, int depth)
    {
        if (start >= end)
            return nullptr;

        int axis = depth % dim;
        auto cmp = [axis](const std::pair<int, PointT>& p1, const std::pair<int, PointT>& p2)
        {
            return p1.second.data[axis] < p2.second.data[axis];
        };

        std::size_t len = end - start;
        auto mid = start + len/2;

        std::nth_element(start, mid, end, cmp);
        while (mid > start && (*(mid-1)).second.data[axis] == (*(mid)).second.data[axis])
            mid--;

        Node<PointT>* newNode = new Node<PointT>((*mid).first, (*mid).second);
        newNode->left = buildTreeHelper(start, mid, depth+1);
        newNode->right = buildTreeHelper(mid+1, end, depth+1);

        return newNode;
    }

    void Insert(int id, PointT point)
    {
        Node<PointT>* newNode = new Node<PointT>(id, point);

        if (root == nullptr)
        {
            root = newNode;
            return;
        }

        Node<PointT>* currNode = root;
        Node<PointT>* prevNode = currNode;
        int depth = 0;
        bool isLeft = false;

        while (currNode != nullptr)
        {
            int axis= depth % dim;
            prevNode = currNode;
            if (point.data[axis] <= currNode->point.data[axis])
            {
                isLeft = true;
                currNode = currNode->left;
            }
            else if(point.data[axis] > currNode->point.data[axis])
            {
                isLeft = false;
                currNode = currNode->right;
            }
            depth++;
        }

        if (isLeft) prevNode->left = newNode;
        else prevNode->right = newNode;
    }

    std::vector<int> Search(PointT target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(root, 0, ids, target, distanceTol);
        return ids;
    }
    void searchHelper(Node<PointT>* node, int depth, std::vector<int>& ids, PointT target, float distanceTol)
    {
        if (node != nullptr)
        {
            // Check if the current node is within the boundary square.
            bool flag = true;
            for (int i=0; i<dim; i++)
            {
                flag = (node->point.data[i] >= (target.data[i] - distanceTol)) && (node->point.data[i] < (target.data[i] + distanceTol));
                if (!flag) break;
            }

            if (flag)
            {
                float distance2 = 0.0;
                for (int i=0; i<dim; i++)
                    distance2 += pow(node->point.data[i] - target.data[i], 2);
                if (distance2 <= pow(distanceTol, 2)) ids.push_back(node->id);
            }

            // check across boundary
            int axis = depth % dim;
            if ((target.data[axis] - distanceTol) < node->point.data[axis])
                searchHelper(node->left, depth+1, ids, target, distanceTol);
            if ((target.data[axis] + distanceTol) > node->point.data[axis])
                searchHelper(node->right, depth+1, ids, target, distanceTol);
        }
    }
};



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

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr inputCloud, float clusterTolerance, int minSize, int maxSize);

    void euclideanClusterHelper(int id, typename pcl::PointCloud<PointT>::Ptr inputCloud, std::vector<int>& cluster, KDTree<PointT>* tree, float clusterTolerance, std::vector<bool>& processed);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);
    BoxQ QBoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */