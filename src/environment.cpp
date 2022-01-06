/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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
    Lidar* lidar = new Lidar(cars, 0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderRays(viewer, lidar->position, inputCloud);
    //renderPointCloud(viewer, inputCloud, "lidar_render.pcd");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);

    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    //renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 2.1, 4, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for (auto cluster : cloudClusters)
    {
        std::cout << "cluster size: " << cluster->points.size();
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        BoxQ boxq = pointProcessor.QBoundingBox(cluster);
        renderBox(viewer, boxq, clusterId);
        clusterId++;
    }

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

/* Loading real PCD files */
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    typename pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    Eigen::Vector4f minPoint = {-8.f, -4.f, -3.f, 1};
    Eigen::Vector4f maxPoint = {30.f, 6.5f, 0.f, 1};
    inputCloud = pointProcessorI->FilterCloud(inputCloud, 0.1f, minPoint, maxPoint);

    // segment obstacles and plane(road)
    auto segmentCloud = pointProcessorI->SegmentPlane(inputCloud, 100, 0.2);
    //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5f, 10, 1000);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->euclideanCluster(segmentCloud.first, 0.5, 10, 1000);

    std::cout << "There are " << cloudClusters.size() << " clusters\n";

    int clusterID = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,0,1), Color(1,1,0)};
    for (int i=0; i<cloudClusters.size(); i++)
    {
        std::cout << "Cluster size: " << cloudClusters[i]->points.size() << std::endl;
        renderPointCloud(viewer, cloudClusters[i], "obstCloud"+std::to_string(clusterID), colors[i % colors.size()]);
        Box box = pointProcessorI->BoundingBox(cloudClusters[i]);
        BoxQ boxq = pointProcessorI->QBoundingBox(cloudClusters[i]);
        renderBox(viewer, box, clusterID);
        renderBox(viewer, boxq, clusterID);
        clusterID++;
    }


    Box roof_box;
    roof_box.x_min = -1.5; roof_box.x_max = 2.6; roof_box.y_min = -1.7; roof_box.y_max = 1.7; roof_box.z_min = -1.f; roof_box.z_max = .4f;
    renderBox(viewer, roof_box, 1001,  Color(0.5, 0.0, 0.5));
    //renderPointCloud(viewer, inputCloud, "inputCloud");
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // Filters InputCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered = pointProcessorI->FilterCloud(inputCloud, 0.1f, Eigen::Vector4f(-8.f, -4.f, -3.f, 1), Eigen::Vector4f(30.f, 6.5f, 0.f, 1));

    // Segment the cloud
    auto segmentCloud = pointProcessorI->SegmentPlane(cloud_filtered, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.second, "PlaneCloud", Color(0, 1, 0));

    // Cluster Obstacles
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->euclideanCluster(segmentCloud.first, 0.5f, 10, 1000);

    float sober_xy[9] = {-1, -2, 1,
                         -2, 0, 2,
                         -1, 2, 1};

    int clusterID = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,0,1), Color(1,1,0)};

    for (int i=0; i<cloudClusters.size(); i++)
    {
        //std::cout << "Cluster size: " << cloudClusters[i]->points.size() << std::endl;
        renderPointCloud(viewer, cloudClusters[i], "obstCloud"+std::to_string(clusterID), colors[i % colors.size()]);
        Box box = pointProcessorI->BoundingBox(cloudClusters[i]);
        renderBox(viewer, box, clusterID);
        //BoxQ boxq = pointProcessorI->QBoundingBox(cloudClusters[i]);
        //renderBox(viewer, boxq, clusterID);
        clusterID++;
    }
    Box roof_box;
    roof_box.x_min = -1.5; roof_box.x_max = 2.6; roof_box.y_min = -1.7; roof_box.y_max = 1.7; roof_box.z_min = -1.f; roof_box.z_max = .4f;
    renderBox(viewer, roof_box, 1001,  Color(0.5, 0.0, 0.5));

}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {

        // Clear Viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacles detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();


        viewer->spinOnce ();
    }
}
