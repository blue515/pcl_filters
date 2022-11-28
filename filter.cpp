#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>    //直方圖可視化1
#include <pcl/visualization/pcl_plotter.h>             //直方圖可視化2

#define passthrough 0
#define statistical 0
#define downsampling 0
#define normal 0
#define keypoints 0

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
//typedef pcl::PointXYZ PointT1;

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passx (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passy (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud2_downsampling (new pcl::PCLPointCloud2());
    pcl::PointCloud<PointT>::Ptr cloud3(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud3_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<PointT>::Ptr cloud4(new pcl::PointCloud<PointT>);
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<PointT> keys;
    pcl::PCDReader reader;
    pcl::PCDWriter writer;
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    //viewer.setBackgroundColor(0,255,255);
    pcl::visualization::PCLPlotter plotter;

    // Fill in the cloud data
    reader.read<pcl::PointXYZ> ("/home/blue/blender/test/mug_tissue.pcd",*cloud1);

    // Create the passthrough object
    #ifdef passthrough
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud1 << std::endl;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud1);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.5, 1.3);
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_passx);
    std::cerr << "Cloud after passx: " << std::endl;
    std::cerr << *cloud_passx << std::endl;

    pass.setInputCloud (cloud_passx);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-0.2, 0.3);
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_passz);
    std::cerr << "Cloud after passz: " << std::endl;
    std::cerr << *cloud_passz << std::endl;

    pass.setInputCloud (cloud_passz);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-0.3, 0.1);
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_passy);
    std::cerr << "Cloud after passy: " << std::endl;
    std::cerr << *cloud_passy << std::endl;
    
    writer.write<pcl::PointXYZ> ("/home/blue/blender/test/mug_tissue_pass.pcd", *cloud_passy, false);
    #endif
    pcl::visualization::PointCloudColorHandlerCustom<PointT> yellow(cloud_passy,255,255,0);
    viewer.addPointCloud(cloud_passy,yellow,"cloud1");

    //Create the statistical object
    #ifdef statistical
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_passx);
    sor.setMeanK (30);     //50
    sor.setStddevMulThresh (1.0);     //1.0
    sor.filter (*cloud_inliers);
    std::cerr << "Cloud after statistical(inliers): " << std::endl;
    std::cerr << *cloud_inliers << std::endl;
    sor.setNegative (true);
    sor.filter (*cloud_outliers);
    std::cerr << "Cloud after statistical(outliers): " << std::endl;
    std::cerr << *cloud_outliers << std::endl;
    writer.write<pcl::PointXYZ> ("/home/blue/blender/test/mug_tissue_inliers.pcd", *cloud_inliers, false);
    #endif
    //Create the downsampling object
    #ifdef downsampling
    reader.read("/home/blue/blender/test/mug_tissue_inliers.pcd",*cloud2);
    std::cerr << "PointCloud before downsampling: " << cloud2->width * cloud2->height << " data points (" << pcl::getFieldsList (*cloud2) << ")." << std::endl;
    pcl::VoxelGrid<pcl::PCLPointCloud2> down;
    down.setInputCloud (cloud2);
    down.setLeafSize (0.05f, 0.05f, 0.05f);     //0.03f,0.03f,0.03f
    down.filter (*cloud2_downsampling);
    std::cerr << "PointCloud after downsampling: " << cloud2_downsampling->width * cloud2_downsampling->height << " data points (" << pcl::getFieldsList (*cloud2_downsampling) << ")." << std::endl;
    writer.write("/home/blue/blender/test/mug_tissue_pass.pcd", *cloud2_downsampling,Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
    #endif
    //normal
    #ifdef normal
    pcl::io::loadPCDFile("/home/blue/blender/test/mug_tissue_pass.pcd",*cloud3);
    //pcl::NormalEstimation<PointT,PointNT> nest;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> nest;
    nest.setInputCloud(cloud3);
    nest.setSearchMethod(tree_n);
    nest.setRadiusSearch(0.05 /* resolution*/);    //0.05
    //nest.setKSearch(50);      // 设置拟合时采用的点数 //50
    //nest.setInputCloud(cloud3);
    //nest.setSearchSurface(cloud3);
    nest.compute(*cloud3_normals);
    std::cout << "compute normal\n";
    for (size_t i = 0; i < cloud3_normals->points.size(); ++i)
    {	// 生成时只生成了法向量，没有将原始点云信息拷贝，为了显示需要复制原信息
	// 也可用其他方法进行连接，如：pcl::concatenateFields
	cloud3_normals->points[i].x = cloud3->points[i].x;
	cloud3_normals->points[i].y = cloud3->points[i].y;
	cloud3_normals->points[i].z = cloud3->points[i].z;
    }
    #endif
    //keypoints(harris)
    #ifdef keypoints
    /*pcl::io::loadPCDFile("/home/blue/ply2pcd/jar/jar9_passthrough.pcd", *cloud4);
    std::cout << "original cloud size : " << cloud4->size() << std::endl;
    pcl::HarrisKeypoint3D<PointT, pcl::PointXYZI> detector;
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_temp(new pcl::PointCloud<pcl::PointXYZI>);
    detector.setNonMaxSupression(true);
    detector.setRadiusSearch(0.05);    //0.03
    detector.setThreshold(1E-6);
    detector.setSearchMethod(tree); 
    detector.setInputCloud(cloud4);
    detector.compute(*keypoints_temp);
    pcl::console::print_highlight("Detected %d points !\n", keypoints_temp->size());
    pcl::copyPointCloud(*keypoints_temp, *keys);
    writer.write("/home/blue/ply2pcd/jar/jar9_keypoint_harris.pcd",*keys,false);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(keys,255,0,0);
    viewer.addPointCloud(keys,red,"cloud2");*/
    pcl::io::loadPCDFile("/home/blue/blender/test/mug_tissue_pass.pcd", *cloud4);
    std::cout << "original cloud size : " << cloud4->size() << std::endl;
    const float min_scale = 0.001f;
    const int n_octaves = 5; //3
    const int n_scales_per_octave = 6; //4
    const float min_contrast = 0.001f;
    pcl::SIFTKeypoint<pcl::PointNormal, PointT> sift; //PointT 可以是 pcl::PointWithScale包含尺度信息
    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud3_normals);
    sift.compute(keys);
    copyPointCloud(keys,*cloud_temp);
    std::cout << "Resulting sift points are of size :" << cloud_temp -> size() << std::endl;
    pcl::io::savePCDFileASCII("/home/blue/blender/test/mug_tissue_keypoint(sift).pcd",*cloud_temp);
    //pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    //pcl::visualization::PointCloudColorHandlerCustom<PointT> yellow(cloud4,255,255,0);
    //viewer.addPointCloud(cloud4,yellow,"cloud2");
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(cloud_temp,0,255,0);
    //viewer.addPointCloud(cloud_temp,keypoints_color_handler,"keypoints");
    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,7,"keypoints");
    #endif

    //可視化
    //pcl::visualization::PointCloudColorHandlerCustom<PointT> yellow(cloud_passy,255,255,0);
    //viewer.addPointCloud(cloud3,"cloud3");
    //int level = 4;      // 多少条法向量集合显示成一条  //50
    //float scale = 0.1;   // 法向量长度  //0.1
    //viewer.addPointCloudNormals<PointNT>(cloud3_normals, level, scale, "normals");
    viewer.spin();
    system("pause");
    return (0);
}
