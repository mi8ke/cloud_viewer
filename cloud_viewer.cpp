#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

int
main ()
{
  // Loading second scan of room from new perspective.
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("room_scan2.pcd", *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << input_cloud->size () << " data points from room_scan2.pcd" << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  //   //... populate cloud
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (input_cloud);
    while (!viewer.wasStopped ())
    {
    }

  return (0);
}


//   // Filtering input scan to roughly 10% of original size to increase speed of registration.
//   pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
//   approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
//   approximate_voxel_filter.setInputCloud (input_cloud);
//   approximate_voxel_filter.filter (*filtered_cloud);
//   std::cout << "Filtered cloud contains " << filtered_cloud->size ()
//             << " data points from room_scan2.pcd" << std::endl;


//   // Saving transformed input cloud.
//   pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

//   // Initializing point cloud visualizer
//   pcl::visualization::PCLVisualizer::Ptr
//   viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//   viewer_final->setBackgroundColor (0, 0, 0);

//   // Coloring and visualizing target cloud (red).
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
//   target_color (target_cloud, 255, 0, 0);
//   viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
//   viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
//                                                   1, "target cloud");

//   // Coloring and visualizing transformed input cloud (green).
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
//   output_color (output_cloud, 0, 255, 0);
//   viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
//   viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
//                                                   1, "output cloud");

//   // Starting visualizer
//   viewer_final->addCoordinateSystem (1.0, "global");
//   viewer_final->initCameraParameters ();

//   // Wait until visualizer window is closed.
//   while (!viewer_final->wasStopped ())
//   {
//     viewer_final->spinOnce (100);
//     std::this_thread::sleep_for(100ms);
//   }


