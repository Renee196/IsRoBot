#include <ros/ros.h>
#include <sensor_msgs/Imu>
#include <random>

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu");
    ros::NodeHandle nh;

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);

    ros::Rate rate(10);

    while (ros::ok()) {
        sensor_msgs::Imu imu_data;

        // Simulated IMU data for demonstration
        imu_data.header.stamp = ros::Time::now();
        imu_data.angular_velocity.x = 0.1 * ((double) rand() / RAND_MAX);
        imu_data.angular_velocity.y = 0.2 * ((double) rand() / RAND_MAX);
        imu_data.angular_velocity.z = 0.3 * ((double) rand() / RAND_MAX);
        imu_data.linear_acceleration.x = 0.5 * ((double) rand() / RAND_MAX);
        imu_data.linear_acceleration.y = 1.0 * ((double) rand() / RAND_MAX);
        imu_data.linear_acceleration.z = 1.5 * ((double) rand() / RAND_MAX);

        imu_pub.publish(imu_data);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

