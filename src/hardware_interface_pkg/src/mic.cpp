#include <ros/ros.h>
#include <custom_msgs/AudioData.h>
#include <random>

int main(int argc, char** argv) {
    ros::init(argc, argv, "mic");
    ros::NodeHandle nh;

    ros::Publisher mic_pub = nh.advertise<custom_msgs::AudioData>("mic_data", 10);

    ros::Rate rate(10);

    while (ros::ok()) {
        custom_msgs::AudioData mic_data;

        // Simulated Microphone data for demonstration
        mic_data.audio_signal = 0.5 * sin(2.0 * M_PI * 1.0 * ros::Time::now().toSec());

        mic_pub.publish(mic_data);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

