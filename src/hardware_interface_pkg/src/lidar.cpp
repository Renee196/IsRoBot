#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <random>

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar");
    ros::NodeHandle nh;

    ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>("lidar_data", 10);

    ros::Rate rate(10);

    while (ros::ok()) {
        sensor_msgs::LaserScan lidar_data;

        // Simulated Lidar data for demonstration
        lidar_data.header.stamp = ros::Time::now();
        lidar_data.angle_min = -M_PI;
        lidar_data.angle_max = M_PI;
        lidar_data.angle_increment = 0.01;
        lidar_data.time_increment = 0.1;
        lidar_data.scan_time = 0.1;
        lidar_data.range_min = 0.0;
        lidar_data.range_max = 10.0;

        for (double angle = lidar_data.angle_min; angle <= lidar_data.angle_max; angle += lidar_data.angle_increment) {
            lidar_data.ranges.push_back(std::abs(std::sin(angle)) + 0.5 * ((double) rand() / RAND_MAX));
        }

        lidar_pub.publish(lidar_data);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

