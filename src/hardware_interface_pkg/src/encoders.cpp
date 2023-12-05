#include <ros/ros.h>
#include <custom_msgs/MoveMotor.h>
#include <random>

int main(int argc, char** argv) {
    ros::init(argc, argv, "encoder");
    ros::NodeHandle nh;

    ros::Publisher encoder_pub = nh.advertise<custom_msgs::MoveMotor>("encoder_data", 10);

    ros::Rate rate(10);

    while (ros::ok()) {
        custom_msgs::MoveMotor encoder_data;

        // Simulated Encoder data for demonstration
        encoder_data.motor_id = 1;
        encoder_data.position = 100.0 + 5.0 * ((double) rand() / RAND_MAX);
        encoder_data.velocity = 2.0 + 1.0 * ((double) rand() / RAND_MAX);

        encoder_pub.publish(encoder_data);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

