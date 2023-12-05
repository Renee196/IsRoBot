#include <ros/ros.h>
#include <custom_msgs/AudioData.h>
#include <iostream>

void audioCallback(const custom_msgs::AudioData::ConstPtr& msg) {
    // Simulated Speaker audio processing logic
    std::cout << "Received audio signal: " << msg->audio_signal << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "speaker_node");
    ros::NodeHandle nh;

    ros::Subscriber audio_sub = nh.subscribe("mic_data", 10, audioCallback);

    ros::spin();

    return 0;
}

