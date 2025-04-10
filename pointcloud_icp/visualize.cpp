#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h> 
#include <Eigen/Dense>

int main()
{
    // Point cloud pointers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);

    // Load point clouds
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("cloud1.pcd", *cloud_source) == -1 ||
        pcl::io::loadPCDFile<pcl::PointXYZ>("cloud2.pcd", *cloud_target) == -1 ||
        pcl::io::loadPCDFile<pcl::PointXYZ>("aligned_cloud.pcd", *cloud_aligned) == -1)
    {
        PCL_ERROR("Couldn't read one or more PCD files\n");
        return -1;
    }

    std::cout << "Loaded point counts:\n";
    std::cout << "  Source (original): " << cloud_source->size() << std::endl;
    std::cout << "  Target: " << cloud_target->size() << std::endl;
    std::cout << "  Aligned: " << cloud_aligned->size() << std::endl;

    Eigen::Matrix4f offset = Eigen::Matrix4f::Identity();
    offset(0, 3) = -0.05f;  
    pcl::transformPointCloud(*cloud_source, *cloud_source, offset);

    // Create PCL visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("ICP Full Visualization"));
    viewer->setBackgroundColor(0, 0, 0);

    // Original source cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud_source, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_source, source_color, "source cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source cloud");

    // Target cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud_target, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_target, target_color, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");

    // Result cloud 
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligned_color(cloud_aligned, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_aligned, aligned_color, "aligned cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligned cloud");

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }

    return 0;
}
