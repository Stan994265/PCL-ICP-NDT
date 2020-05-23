#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h> 
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/time.h> 

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
int
main(int argc, char** argv)
{
    //加载房间的第一次扫描
    pcl::console::TicToc time;     //申明时间记录
    time.tic ();       //time.tic开始  time.toc结束时间
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/0.pcd", *target_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file 0.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << target_cloud->size() << " data points from 0.pcd" << std::endl;
    //加载从新视角得到的房间的第二次扫描
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/1.pcd", *input_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file 1.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << input_cloud->size() << " data points from 1.pcd" << std::endl;
    //将输入的扫描过滤到原始尺寸的大概x0%以提高匹配的速度。
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud(input_cloud);
    approximate_voxel_filter.filter(*filtered_cloud);
    std::cout << "Filtered cloud contains " << filtered_cloud->size()
        << " data points from 1.pcd" << std::endl;
    //初始化正态分布变换（NDT）
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    //设置依赖尺度NDT参数
    //为终止条件设置最小转换差异
    ndt.setTransformationEpsilon(0.0001);
    //为More-Thuente线搜索设置最大步长
    ndt.setStepSize(0.1);
    //设置NDT网格结构的分辨率（VoxelGridCovariance）
    ndt.setResolution(0.5);
    //设置匹配迭代的最大次数
    ndt.setMaximumIterations(50);
    // 设置要配准的点云
    ndt.setInputSource(input_cloud);
    //设置点云配准目标
    ndt.setInputTarget(target_cloud);

    //计算需要的刚体变换以便将输入的点云匹配到目标点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*output_cloud);
    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
        << " score: " << ndt.getFitnessScore() << std::endl;
    //使用创建的变换对未过滤的输入点云进行变换
    pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());
    cout<<ndt.getFinalTransformation()<<endl;
    time.toc ();        //时间
    cout<<"\n"<<time.toc ()<<endl;
    pcl::visualization::PCLVisualizer viewer ("NDT demo");
  
    
  // 创建两个观察视点
  int v1 (0);
  int v2 (1);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

  // 定义显示的颜色信息
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // 原始的点云设置为白色的
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (target_cloud, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                             (int) 255 * txt_gray_lvl);
  
  viewer.addPointCloud (target_cloud, cloud_in_color_h, "cloud_in_v1", v1);//设置原始的点云都是显示为白色
  
  viewer.addPointCloud (target_cloud, cloud_in_color_h, "cloud_in_v2", v2);
 
  // 转换后的点云显示为绿色
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (input_cloud, 0, 255, 0);
  viewer.addPointCloud (input_cloud, cloud_tr_color_h, "cloud_tr_v1", v1);
 
  // ICP配准后的点云为红色
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_ndt_color_h (output_cloud, 255, 0, 0);
  viewer.addPointCloud (output_cloud, cloud_ndt_color_h, "cloud_ndt_v2", v2);
  //viewer.addCoordinateSystem (1.0);
  // 加入文本的描述在各自的视口界面
 //在指定视口viewport=v1添加字符串“white 。。。”其中"icp_info_1"是添加字符串的ID标志，（10，15）为坐标16为字符大小 后面分别是RGB值
  viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "NDT_info_1", v1);
  viewer.addText ("White: Original point cloud\nRed: NDT aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "NDT_info_2", v2);

  
 // viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

  // 设置背景颜色
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

  // 设置相机的坐标和方向
  viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.setSize (1280, 1024);  // 可视化窗口的大小

  

   
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return (0);
}

