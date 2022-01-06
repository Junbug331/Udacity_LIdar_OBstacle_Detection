/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <random>
#include <tuple>
#include <functional>
#include <type_traits>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5); // [-1.0, 1.0)
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5); // [-1.0, 1.0)
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx; // i + ( 0.6 * rand[-1.0, 1.0) )
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIteration, float distanceTol)
{
    std::random_device dev;
    std::mt19937_64 eng(dev());
    std::uniform_int_distribution<int> dist(0, cloud->points.size()-1);
    std::unordered_set<int> inliersResult;

    while (--maxIteration)
    {
        // extract random 3 points from the cloud
        int maxCount = 0;
        std::unordered_set<int> inliers;
        while (inliers.size() < 3)
            inliers.insert(dist(eng));

        auto itr = inliers.begin();
        auto p = cloud->points[*itr];
        float x1 = p.x;
        float y1 = p.y;
        float z1 = p.z;
        itr++;
        p = cloud->points[*itr];
        float x2 = p.x;
        float y2 = p.y;
        float z2 = p.z;
        itr++;
        p = cloud->points[*itr];
        float x3 = p.x;
        float y3 = p.y;
        float z3 = p.z;

        //normal vector n
        // find coefficient a = n_i, b = n_j, c = n_k
        float a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
        float b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
        float c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
        float d = -(a*x1 + b*y1 + c*z1);

        for (int i=0; i<cloud->points.size(); i++)
        {
            float x = cloud->points[i].x;
            float y = cloud->points[i].y;
            float z = cloud->points[i].z;

            float dis = fabs(a*x + b*y + c*z + d) / sqrt(a*a + b*b + c*c);

            if (dis <= distanceTol)
            {
                inliers.insert(i);
            }
        }

        if (inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
    }
    return inliersResult;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::random_device dev;
    std::mt19937_64 eng(dev());
    std::uniform_int_distribution<int> dist(0, cloud->points.size()-1);

    std::tuple<int, int> tp1;

	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	int max_count = 0;
	
	// TODO: Fill in this function

	// For max iterations
	for (int i=0; i<maxIterations; i++)
    {
        // Randomly sample subset and fit line
        std::unordered_set<int> inliers;
        while (inliers.size() < 2)
            inliers.insert(dist(eng));

        // extract line coefficients of line equation
        auto itr = inliers.begin();
        auto p1 = cloud->points.at(*itr);
        itr++;
        auto p2 = cloud->points.at(*itr);

        float x1 = p1.x;
        float y1 = p1.y;
        float x2 = p2.x;
        float y2 = p2.y;

        float A = y1 - y2;
        float B = x2 - x1;
        float C = x1*y2 - y1*x2;

        // Measure distance between every point and fitted line
        for (int j=0; j<cloud->points.size(); j++)
        {
            if (inliers.find(j) != inliers.end())
                continue;

            float x = cloud->points.at(j).x;
            float y = cloud->points.at(j).y;
            float dis = fabs(A*x + B*y + C) / sqrt(pow(A,2)+pow(B,2));

            // If distance is smaller than threshold count it as inlier
            if (dis <= distanceTol)
                inliers.insert(j);
        }
        if (inliers.size() > max_count)
        {
            max_count = inliers.size();
            inliersResult = inliers;
        }
    }


	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 500, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
