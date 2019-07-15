#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

ros::Publisher pub;

pcl::visualization::CloudViewer viewer ("Viewer");

void normalizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr givenPointCloud){
  // Find the planar coefficients for floor plane
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr floor_inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(givenPointCloud);
  seg.segment(*floor_inliers, *coefficients);
  std::cerr << "Floor Plane Model coefficients: " << coefficients->values[0] << " "
                                    << coefficients->values[1] << " "
                                    << coefficients->values[2] << " "
                                    << coefficients->values[3] << std::endl;

  Eigen::Matrix<float, 1, 3> floor_plane_normal_vector, xy_plane_normal_vector, rotation_vector;

  floor_plane_normal_vector[0] = coefficients->values[0];
  floor_plane_normal_vector[1] = coefficients->values[1];
  floor_plane_normal_vector[2] = coefficients->values[2];

  std::cout << floor_plane_normal_vector << std::endl;

  xy_plane_normal_vector[0] = 0.0;
  xy_plane_normal_vector[1] = 0.0;
  xy_plane_normal_vector[2] = 1.0;

  std::cout << xy_plane_normal_vector << std::endl;

  rotation_vector = xy_plane_normal_vector.cross (floor_plane_normal_vector);
  std::cout << "Rotation Vector: "<< rotation_vector << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  float theta = -acos(floor_plane_normal_vector.dot(xy_plane_normal_vector)/sqrt( pow(coefficients->values[0],2)+ pow(coefficients->values[1],2) + pow(coefficients->values[2],2)));

  rotation_vector.normalize();

  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  transform_2.translation() << 0, 0, 30;
  transform_2.rotate(Eigen::AngleAxisf (theta, rotation_vector));
  std::cout << "Transformation matrix: " << std::endl << transform_2.matrix() << std::endl;
  pcl::transformPointCloud(*givenPointCloud, *transformed_cloud, transform_2);

  viewer.showCloud(transformed_cloud);

}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Convert incoming pointCloud:
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  normalizePointCloud(temp_cloud);

  // Do data processing here...
  output = *input;

  // Publish the data.
  pub.publish (output);
}

int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "pointCloudNode");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}

