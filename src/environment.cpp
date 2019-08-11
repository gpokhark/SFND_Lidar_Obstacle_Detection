/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;

    bool render_ray = false;
    bool render_PointCloud = true;

    bool render_obst = false;
    bool render_plane = false;

    bool render_clusters = true;
    bool render_box = true;

    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar *lidar = new Lidar(cars, 0); // Lidar pointer type initiating on heap using new keyword

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // Generate PCL point cloud
    if (render_ray)
        renderRays(viewer, lidar->position, inputCloud);
    if (render_PointCloud)
        renderPointCloud(viewer, inputCloud, "inputCloud");

    // TODO:: Create point processor
    // ProcessPointClouds<pcl::PointXYZ> pointProcessor; // normal way to instatiat point processor
    // instantiate on heap
    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    int maxIterations = 100;
    float distanceTolerance = 0.2;
    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud, maxIterations, distanceTolerance);
    auto segmentCloud = pointProcessor->SegmentPlane(inputCloud, maxIterations, distanceTolerance);

    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));

    if (render_obst)
        renderPointCloud(viewer, segmentCloud.first, "obsCloud", Color(1, 0, 0));
    if (render_plane)
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    float clusterTolerance = 3.0;
    int minClusterSize = 15;
    int maxClusterSize = 600;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, clusterTolerance, minClusterSize, maxClusterSize);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if (render_clusters)
        {
            std::cout << "cluster size ";
            pointProcessor->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        }

        if (render_box)
        {
            Box box = pointProcessor->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }

        ++clusterId;
    }
}

// Process load single frame pcd file
// *** for streaming data continuously *** //
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)

// *** for stand alone data file ***//
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
    // RENDER OPTIONS
    bool render_PointCloud = false;
    bool filtered_PointCloud = false;

    bool render_obst = false;
    bool render_plane = false;

    bool render_clusters = true;
    bool render_box = true;

    // // *** for stand alone data file ***//
    // // instantiate on heap
    // ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // // *** for stand alone data file ***//

    // Generate PCL point cloud
    if (render_PointCloud)
        renderPointCloud(viewer, inputCloud, "inputCloud");

    // Experiment with the ? values and find what works best
    float filterResolution = 0.3;
    Eigen::Vector4f minPoint(-10, -5, -2, 1);
    Eigen::Vector4f maxPoint(30, 8, 1, 1);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, filterResolution, minPoint, maxPoint);
    if (filtered_PointCloud)
        renderPointCloud(viewer, filterCloud, "filterCloud");

    int maxIterations = 100;
    float distanceTolerance = 0.2;

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, maxIterations, distanceTolerance);
    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));

    if (render_obst)
        renderPointCloud(viewer, segmentCloud.first, "obsCloud", Color(1, 0, 0));
    if (render_plane)
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    float clusterTolerance = 0.5;
    int minClusterSize = 15;
    int maxClusterSize = 600;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, clusterTolerance, minClusterSize, maxClusterSize);

    // Redering box around host vehicle
    Box host_box = {-1.5, -1.2, -1, 2.6, 1.2, -0.4};
    renderBox(viewer, host_box, 0, Color(0.5, 0.5, 0.5), 0.8);

    int clusterId = 1;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        if (render_clusters)
        {
            std::cout << "cluster size";
            pointProcessorI->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        }

        if (render_box)
        {
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }

        ++clusterId;
    }
}
//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // // *** for stand alone data file *** //
    // //simpleHighway(viewer);
    // cityBlock(viewer);

    // while (!viewer->wasStopped())
    // {
    //     viewer->spinOnce();
    // }
    // // *** for stand alone data file *** //

    // *** for streaming data contiuously *** //
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while(!viewer->wasStopped())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
    // *** for streaming data contiuously *** //
}
