#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "sidecamera");
    ros::NodeHandle nh;

    ros::Publisher camera_pub = nh.advertise<sensor_msgs::SideImage>("camera_image", 10);

    ros::Rate rate(10);

    while (ros::ok()) {
        sensor_msgs::Image side_camera_image;

        // Simulated Camera image for demonstration
        cv::Mat image(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
        cv::circle(image, cv::Point(320, 240), 50, cv::Scalar(0, 0, 255), -1);

        cv_bridge::CvImage cv_image;
        cv_image.encoding = "bgr8";
        cv_image.image = image;
        cv_image.toImageMsg(side_camera_image);

        camera_pub.publish(side_camera_image);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

