#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>

int main()
{
    // Define point cloud types
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

    // Create point cloud objects
    PointCloudT::Ptr cloud_in(new PointCloudT);   // Source
    PointCloudT::Ptr cloud_out(new PointCloudT);  // Target
    PointCloudT::Ptr cloud_aligned(new PointCloudT); // Result

    // Load source and target PCD files
    if (pcl::io::loadPCDFile<PointT>("cloud1.pcd", *cloud_in) == -1 ||
        pcl::io::loadPCDFile<PointT>("cloud2.pcd", *cloud_out) == -1)
    {
        PCL_ERROR("Could not load input files.\n");
        return -1;
    }

    std::cout << "Loaded " << cloud_in->size() << " source points." << std::endl;
    std::cout << "Loaded " << cloud_out->size() << " target points." << std::endl;

    // Apply ICP
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    icp.align(*cloud_aligned);

    // Check result
    if (icp.hasConverged())
    {
        std::cout << "ICP converged." << std::endl;
        std::cout << "Score: " << icp.getFitnessScore() << std::endl;
        std::cout << "Transformation matrix:\n" << icp.getFinalTransformation() << std::endl;

        // Save aligned result
        pcl::io::savePCDFileBinary("aligned_cloud.pcd", *cloud_aligned);
        std::cout << "Aligned cloud saved to 'aligned_cloud.pcd'" << std::endl;
    }
    else
    {
        std::cout << "ICP did not converge." << std::endl;
    }

    return 0;
}
