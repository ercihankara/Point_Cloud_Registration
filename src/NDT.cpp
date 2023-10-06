#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>

#include <string>
#include <iostream>
#include <iomanip>

// run roscore

using namespace std;

int main(int argc, char** argv){
    // Initialize the ROS node
    ros::init(argc, argv, "point_cloud");
    ros::NodeHandle nh;

    // create variables to store alignment error and convergence time
    double ndt_alignment_error = 0.0;
    double sum_ndt_alignment_error = 0.0;
    double avrg_ndt_alignment_error = 0.0;
    double ndt_convergence_time = 0.0;

    // create variable for number of decimal points to be used in trajectory file
    int dec = 9;

    // define the step size for downsampling
    int step_size = 5;

    // enter the address of the rosbag file
    string scenario = "/home/ercihan/catkin_ws/src/point_cloud/scenario_2";

    // define topics for LiDAR and IMU data
    string lidar_topic = "/center/pandar";
    string imu_topic = "/gps/imu";

    // create message containers to store the data received from the ROS topics
    sensor_msgs::PointCloud2::Ptr lidar_msg(new sensor_msgs::PointCloud2);
    sensor_msgs::Imu::Ptr imu_msg(new sensor_msgs::Imu);

    // create point cloud containers and filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;

    // create variables for trajectory estimation
    // initial position (x, y, z)
    Eigen::Vector3d position(0.0, 0.0, 0.0);
    // initial velocity (vx, vy, vz)
    Eigen::Vector3d velocity(0.0, 0.0, 0.0);
    // initial orientation (quaternion)
    Eigen::Quaterniond orientation(1.0, 0.0, 0.0, 0.0);
    // previous timestamp in seconds
    double previous_timestamp = 0.0;

    // open the ROSBAG file for reading
    rosbag::Bag bag;
    bag.open(scenario + ".bag", rosbag::bagmode::Read);

    // define a file for saving the trajectory data
    std::ofstream trajectory_file("/home/ercihan/catkin_ws/src/point_cloud/trajectory_scenario_2_ndt.txt");
    // create the column names in the file
    trajectory_file << "#timestamp" << " " << "x" << " " << "y" << " " << "z" << " " << "q_x" << " "
        << "q_y" << " " << "q_z" << " " << "q_w" << endl;

    // create variable to keep track of the message count, and error
    int message_count = 0;
    int error_count = 0;

    // read messages from the ROSBAG file
    // check for the topic names through the file
    rosbag::View view(bag);
    ros::Time ndt_start_time = ros::Time::now();
    for(const rosbag::MessageInstance& msg : view){
        // skip messages based on the step size, downsample
        if(message_count % step_size != 0){
            message_count++;
            continue;
        }
        if(msg.getTopic() == lidar_topic){
            lidar_msg = msg.instantiate<sensor_msgs::PointCloud2>();
            // process LiDAR data here using PCL !!!
            pcl::fromROSMsg(*lidar_msg, *cloud_source);

            // if this is the first LiDAR message, set it as the initial target
            if(cloud_target->empty()){
                *cloud_target = *cloud_source;
                continue;
            }

            // create an NDT object
            pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
            ndt.setInputSource(cloud_source);
            ndt.setInputTarget(cloud_target);

            // set NDT parameters if needed
            ndt.setMaximumIterations(10); // Decrease
            ndt.setTransformationEpsilon(0.1); // Increase
            ndt.setResolution(2.0); // Increase
            // ndt.setMaximumStepSize(0.1); // Decrease
            approximate_voxel_filter.setLeafSize(1, 1, 1); // Adjust the leaf size as needed -> MAKE IT ONE, increase
            approximate_voxel_filter.setInputCloud(cloud_source);
            approximate_voxel_filter.filter(*cloud_source);

            // perform the point cloud registration, alignment
            ndt.align(*cloud_aligned);

            // update the target cloud with the aligned source cloud, cumulative
            *cloud_target += *cloud_aligned;

            // calculate alignment error using RMSE for NDT
            // iterate through each point in the aligned point cloud and calculate
            // the difference in position (dx, dy, dz) between the aligned point cloud and the target point cloud
            int valid_correspondences = 0;
            for(size_t i = 0; i < cloud_aligned->points.size(); ++i){
                if (!isfinite(cloud_aligned->points[i].x)) continue;
                double dx = cloud_aligned->points[i].x - cloud_target->points[i].x;
                double dy = cloud_aligned->points[i].y - cloud_target->points[i].y;
                double dz = cloud_aligned->points[i].z - cloud_target->points[i].z;
                ndt_alignment_error += dx * dx + dy * dy + dz * dz;
                // count the number of valid point correspondences between the source and aligned point clouds
                // a valid correspondence means both source and aligned points at the same index have finite (non-NaN) coordinates
                valid_correspondences++;
            }
            // perform error related operations
            ndt_alignment_error = sqrt(ndt_alignment_error / valid_correspondences);
            sum_ndt_alignment_error += ndt_alignment_error;
            cout << "error: " << error_count << endl;
            error_count++;
        }
        else if(msg.getTopic() == imu_topic){
            imu_msg = msg.instantiate<sensor_msgs::Imu>();
            // process IMU data here to estimate the trajectory !!!
            // extract linear acceleration and angular velocity from IMU message
            Eigen::Vector3d linear_acceleration;
            Eigen::Vector3d angular_velocity;

            // read linear acceleration
            linear_acceleration.x() = imu_msg->linear_acceleration.x;
            linear_acceleration.y() = imu_msg->linear_acceleration.y;
            linear_acceleration.z() = imu_msg->linear_acceleration.z;

            // read angular velocity
            angular_velocity.x() = imu_msg->angular_velocity.x;
            angular_velocity.y() = imu_msg->angular_velocity.y;
            angular_velocity.z() = imu_msg->angular_velocity.z;

            // get the timestamp of the IMU message
            double timestamp = imu_msg->header.stamp.toSec();

            // calculate time difference since the previous IMU message
            double delta_time = timestamp - previous_timestamp;

            // update position using velocity and linear acceleration (Euler integration), Dx = v*Dt + 0.5*a*t^2
            position += velocity * delta_time + 0.5 * linear_acceleration * delta_time * delta_time;

            // update velocity using linear acceleration, Dv = a*Dt
            velocity += linear_acceleration * delta_time;

            // Update orientation using angular velocity (Euler integration)
            Eigen::Quaterniond orientation_update(1.0, angular_velocity.x() * delta_time / 2.0,
                                                angular_velocity.y() * delta_time / 2.0,
                                                angular_velocity.z() * delta_time / 2.0);
            orientation = orientation * orientation_update;

            // store the current timestamp as the previous timestamp for the next iteration
            previous_timestamp = timestamp;

            // save the trajectory data to the file
            trajectory_file << std::fixed << std::setprecision(dec) << timestamp << " "
                << std::setprecision(dec) << position.x() << " "
                << std::setprecision(dec) << position.y() << " "
                << std::setprecision(dec) << position.z() << " "
                << std::setprecision(dec) << orientation.x() << " "
                << std::setprecision(dec) << orientation.y() << " "
                << std::setprecision(dec) << orientation.z() << " "
                << std::setprecision(dec) << orientation.w() << std::endl;
        }
        message_count++;
    }
    ros::Time ndt_end_time = ros::Time::now();
    ndt_convergence_time = (ndt_end_time - ndt_start_time).toSec();

    // save the aligned point cloud
    pcl::io::savePCDFileASCII("/home/ercihan/catkin_ws/src/point_cloud/point_cloud_scenario_2_ndt.pcd", *cloud_aligned);

    std::cout << "processing data is done!" << endl;

    // print average alignment error for NDT
    avrg_ndt_alignment_error = sum_ndt_alignment_error/error_count;
    ROS_INFO("NDT Average Alignment Error: %f", avrg_ndt_alignment_error);
    // print convergence time for NDT
    ROS_INFO("NDT Convergence Time: %f seconds", ndt_convergence_time);

    // close the trajectory file
    trajectory_file.close();

    // close the ROSBAG file
    bag.close();

    // // publish the aligned point cloud and estimated trajectory, for RViz
    // ros::Publisher map_publisher = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_map", 1);
    // ros::Publisher trajectory_publisher = nh.advertise<nav_msgs::Path>("trajectory", 1);

    // sensor_msgs::PointCloud2 cloud_msg;
    // pcl::toROSMsg(*cloud_aligned, cloud_msg);
    // cloud_msg.header.frame_id = "map"; // Set the frame ID as needed

    // while(map_publisher.getNumSubscribers() == 0) {
    //     ROS_INFO_ONCE("Waiting for subscribers to connect...");
    //     ros::Duration(1.0).sleep();
    // }

    // // publish the point cloud map for RViz visualisation
    // map_publisher.publish(cloud_msg);

    // spin to keep the node alive
    ros::spin();

    return 0;
}
