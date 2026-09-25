#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <string>

class ImageToVideo
{
public:
    ImageToVideo()
    {
        ros::NodeHandle private_nh("~");
        std::string image_topic;
        std::string output_file;
        private_nh.param<std::string>("image_topic", image_topic, "/camera_rect/image_rect");
        private_nh.param<std::string>("output_file", output_file, "output.avi");
        image_transport::ImageTransport it(nh);
        sub = it.subscribe(image_topic, 1, &ImageToVideo::imageCallback, this);

        video_writer = cv::VideoWriter(output_file, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, cv::Size(640, 480));
        if (!video_writer.isOpened())
        {
            ROS_FATAL_STREAM("Failed to open video output: " << output_file);
            ros::shutdown();
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            if (!frame.empty())
            {
                video_writer.write(frame);
                ROS_DEBUG("Image received and written to video");
            }
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

private:
    ros::NodeHandle nh;
    image_transport::Subscriber sub;
    cv::VideoWriter video_writer;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_to_video_node");
    ImageToVideo itv;
    ros::spin();
    return 0;
}
