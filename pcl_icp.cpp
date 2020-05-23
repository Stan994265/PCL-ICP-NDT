#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>    
#include <pcl/point_types.h>  
#include <pcl/registration/icp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/convolution.h>
#include <pcl/search/kdtree.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
int
main (int argc,
      char** argv)
{
  
  pcl::console::TicToc time;     //申明时间记录
  time.tic ();       //time.tic开始  time.toc结束时间
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/0.pcd", *cloud_in ) == -1)
    {
        PCL_ERROR("Couldn't read file 0.pcd \n");
        return (-1);
    }
  std::cout << "Loaded " << cloud_in->size() << " data points from 0.pcd" << std::endl;


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tr(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/1.pcd", *cloud_tr) == -1)
    {
        PCL_ERROR("Couldn't read file 1.pcd \n");
        return (-1);
    }
  std::cout << "Loaded " << cloud_tr->size() << " data points from 1.pcd" << std::endl;
//filter  
   /* 
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.5, 0.5, 0.5);
    approximate_voxel_filter.setInputCloud(cloud_tr);
    approximate_voxel_filter.filter(*filtered_cloud);
    std::cout << "Filtered cloud contains " << filtered_cloud->size()
        << " data points from 1.pcd" << std::endl;
	*/

//sampling
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
  voxelgrid.setLeafSize(0.3f, 0.3f, 0.3f);
  voxelgrid.setInputCloud(cloud_in);
  voxelgrid.filter(*downsampled);
  *cloud_in = *downsampled;
  voxelgrid.setInputCloud(cloud_tr);
  voxelgrid.filter(*downsampled);
  *cloud_tr = *downsampled;
/*cropbox
    Eigen::Vector4f min_point;
    Eigen::Vector4f max_point;
    pcl::CropBox<pcl::PointXYZ> box_filter;       
    min_point<<-1,-1,-1,1.0;
    max_point<<1,1,1,1.0;
    box_filter.setMin(min_point);
    box_filter.setMax(max_point);
    box_filter.setInputCloud (cloud_in);
    //box_filter.setUserFilterValue(0.1f);
    //box_filter.setKeepOrganized(true); 
    box_filter.setNegative(false);
    box_filter.filter (*cloud_in);*/
//     box_filter.setInputCloud (cloud_in);
//     //box_filter.setUserFilterValue(0.1f);
//     //box_filter.setKeepOrganized(true); 
//     box_filter.setNegative(false);
//     box_filter.filter (*cloud_in);
//  gaussian
// 	pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>::Ptr kernel(new pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>);
// 	(*kernel).setSigma(4);
// 	(*kernel).setThresholdRelativeToSigma(4);
// 	std::cout << "Kernel made" << std::endl;
// 	//Set up the KDTree
// 	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
// 	(*kdtree).setInputCloud(cloud_in);
// 	std::cout << "KdTree made" << std::endl;
// 	//Set up the Convolution Filter
// 	pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> convolution;
// 	convolution.setKernel(*kernel);
// 	convolution.setInputCloud(cloud_in);
// 	convolution.setSearchMethod(kdtree);
// 	convolution.setRadiusSearch(100);
// 	std::cout << "Convolution Start" << std::endl;
// 	convolution.convolve(*cloud_in);
// 	std::cout << "Convoluted" << std::endl;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZ>);	
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>icp;
  //pcl::IterativeClosestPointWithNormals<filtered_cloud,PointCloudT> icp;

  icp.setInputSource (cloud_tr);   //设置输入的点云
  icp.setInputTarget (cloud_in);    //目标点云

  icp.setMaxCorrespondenceDistance(100);  
  icp.setTransformationEpsilon(1e-10); 
  icp.setEuclideanFitnessEpsilon(0.001); 
  icp.setMaximumIterations (1000);  
  Eigen::AngleAxisf init_rotation(1, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(0.3, 0.3, 0.3);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
  icp.align(*cloud_icp,init_guess);

  cout<<icp.getFinalTransformation()<<endl;
  std::cout << "fitness: " << icp.getFitnessScore() << std::endl << std::endl;

//addCoordinateSystem添加转移矩阵
  Eigen::Affine3f t;
  Eigen::Matrix4f a;
  a=icp.getFinalTransformation();
  t.matrix() = a;


  time.toc ();        //时间
  cout<<"\n"<<time.toc ()<<endl;
  // 可视化ICP的过程与结果
  pcl::visualization::PCLVisualizer viewer ("ICP demo");
  // 创建两个观察视点
  int v1 (0);
  int v2 (1);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

  // 定义显示的颜色信息
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // 原始的点云设置为白色的
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in,  255, 255,
                                                                               255);
  viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);//设置原始的点云都是显示为白色
  viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

  // 转换后的点云显示为绿色
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 0, 255, 0);
  viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

  // ICP配准后的点云为红色
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 255, 0, 0);
  viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);
  viewer.addCoordinateSystem (1.0,0,0,0,"cloud_icp_v2",v2);
  viewer.addCoordinateSystem (1.0,0,0,0,"cloud_in_v2",v2);
  viewer.addCoordinateSystem (1.0,0,0,0,"cloud_in_v1",v1);
  viewer.addCoordinateSystem (1.0,0,0,0,"cloud_tr_v1",v1);
  viewer.addCoordinateSystem (1.0,t,"cloud_icp_v2",v2);


  // 加入文本的描述在各自的视口界面
  //在指定视口viewport=v1添加字符串“white 。。。”其中"icp_info_1"是添加字符串的ID标志，（10，15）为坐标16为字符大小 后面分别是RGB值
  viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
  viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);


  // 设置背景颜色
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
//      viewer.setBackgroundColor (txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, v1);
//      viewer.setBackgroundColor (txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, v2);

  // 设置相机的坐标和方向
  viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.setSize (1280, 1024);  // 可视化窗口的大小


  // 显示
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
   boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    
  }
  return (0);
}
