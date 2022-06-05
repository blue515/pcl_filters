#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/keypoints/iss_3d.h>
#include "resolution.h"

int main ()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZRGB> ("/home/blue/ply2pcd/char_inliers_passthrough_y.pcd",*cloud);

    /*cloud->width  = 5;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (auto& point: *cloud)
    {
        point.x = 1024 * rand () / (RAND_MAX + 1.0f);
        point.y = 1024 * rand () / (RAND_MAX + 1.0f);
        point.z = 1024 * rand () / (RAND_MAX + 1.0f);
    }*/

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;
    /*for (const auto& point: *cloud)
        std::cerr << "    " << point.x << " "
                            << point.y << " "
                            << point.z << std::endl;*/

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    //pass.setFilterFieldName ("y");
    //pass.setFilterLimits (-0.3, 0.3);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (1.1, 1.9);
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_filtered);
    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB>("/home/blue/ply2pcd/char_inliers_passthrough.pcd", *cloud_filtered, false);
    //std::cerr << "Cloud after filtering: " << std::endl;
    /*for (const auto& point: *cloud_filtered)
        std::cerr << "    " << point.x << " "
                            << point.y << " "
                            << point.z << std::endl;*/

    return (0);
}
