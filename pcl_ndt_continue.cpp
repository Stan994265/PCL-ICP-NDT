#include <iostream>
#include <fstream>
#include <boost/format.hpp> 
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>  
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int
main (int argc,
      char** argv)
{
   Eigen::AngleAxisf Init_R(1, Eigen::Vector3f::UnitZ());
   Eigen::Translation3f Init_t(0, 0, 0);
   Eigen::Affine3f t ;
   t.matrix()= (Init_R * Init_t).matrix();
   pcl::visualization::PCLVisualizer viewer ("NDT_Continue demo");
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tr(new pcl::PointCloud<pcl::PointXYZ>);
   
   
   //提前加载第一帧数据
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_init (new pcl::PointCloud<pcl::PointXYZ>);
   if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/9.pcd", *cloud_init ) == -1)
    {
        PCL_ERROR("Couldn't read file room_scan1.pcd \n");
        return (-1);
    }
    
    
    // 定义显示的颜色信息
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // 原始的点云设置为白色的
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_init, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                             (int) 255 * txt_gray_lvl);
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level); 
  viewer.setSize (1280, 1024);  // 可视化窗口的大小
  viewer.addPointCloud (cloud_init, cloud_in_color_h, "cloud_in_v1");//设置原始的点云都是显示为白色

  for(int i=9;i<29;i++) {
  boost::format fmt("../%s/%d.%s");
  pcl::io::loadPCDFile<pcl::PointXYZ>((fmt % "data" % (i) % "pcd").str(), *cloud_in );
  pcl::io::loadPCDFile<pcl::PointXYZ>((fmt % "data" % (i + 1) % "pcd").str(), *cloud_tr);

  pcl::console::TicToc time;     //申明时间记录
  time.tic ();       //time.tic开始  time.toc结束时间
 
//sampling
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
  voxelgrid.setLeafSize(0.3f, 0.3f, 0.3f);
  voxelgrid.setInputCloud(cloud_in);
  voxelgrid.filter(*downsampled);
  *cloud_in = *downsampled;
  voxelgrid.setInputCloud(cloud_tr);
  voxelgrid.filter(*downsampled);
  cloud_tr = downsampled;
//初始化正态分布变换（NDT）
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setTransformationEpsilon(0.001);
  ndt.setStepSize(0.1);
  ndt.setResolution(0.5);
  ndt.setMaximumIterations(50);
  ndt.setInputSource(cloud_tr);
  ndt.setInputTarget(cloud_in);

//赋初值
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ndt(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::AngleAxisf init_rotation(1, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(0.3, 0.3, 0.3);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
  ndt.align(*cloud_ndt);

  cout<<ndt.getFinalTransformation()<<endl;
  cout << "fitness: " << ndt.getFitnessScore() << endl;

//addCoordinateSystem添加转移矩阵
  Eigen::Matrix4f a;
  Eigen::Matrix4f b;
  a=ndt.getFinalTransformation();
  b=a*t.matrix();
  t.matrix()=b;

  time.toc ();        
  cout<<"\n"<<time.toc ()<<endl;

  viewer.addCoordinateSystem (1.0,t,"cloud_ndt");
  
  } 
     while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
   boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    
  }


  
  return (0);
}
